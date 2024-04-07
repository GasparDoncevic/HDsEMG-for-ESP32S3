/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
//TODO:
// Implement data copying from SRAM to datastructure for espnow data
// Implement reciver tasks for recieveing data and parsing data
// 
/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_example.h"

#define ESPNOW_MAXDELAY 512

//USER DEFINES
#define  DEVICE_ROLE_SENDER 1
#define  DEVICE_ROLE_RECIEVER 0
#define  IMAGE_SIZE 16
#define ESPNOW_SEND_TASK_PRIORITY 5
#define ESPNOW_DATA_PREP_TASK_PRIORITY 4
//USER DEFINES


//USER PRIVATE VARIABLES
bool run_example = false;
bool device_role = DEVICE_ROLE_SENDER;


TaskHandle_t espnow_send_data_taskHandle = NULL;
TaskHandle_t espnow_data_prep_taskHandle = NULL;
TaskHandle_t TEST_espnow_stage_data_taskhandle = NULL;
static QueueHandle_t image_queue = NULL; // queue for raw image data, between memory location and data prep task
static QueueHandle_t espnow_com_queue = NULL; // queue for send data, between data prep task and send task
SemaphoreHandle_t semaphore_send = NULL;
SemaphoreHandle_t semaphore_receive = NULL;

static const char *USER_TAG = "user_espnow";
example_espnow_send_param_t *send_parameters = NULL;
//USER PRIVATE VARIABLES
static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };
static uint16_t s_image_data[IMAGE_SIZE] = {0xAA};
//static uint8_t device_role = DEVICE_ROLE_SENDER;

static void example_espnow_deinit(example_espnow_send_param_t *send_param);

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    /* Fill all remaining bytes after the data with random values */
    esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(50 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_PERIOD_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                example_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {
                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            example_espnow_data_prepare(send_param);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 8048, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(s_example_espnow_queue);
    esp_now_deinit();
}

//BEGINING OF USER CODE
static esp_err_t user_espnow_init(void)
{
    // This is now a global variable
    //example_espnow_send_param_t *send_param;

    //creating semaphores for sending and receiving
    semaphore_send = xSemaphoreCreateBinary();
    semaphore_receive = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphore_send);
    xSemaphoreGive(semaphore_receive);

    s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_LOGI(USER_TAG, "Starting espnow init");
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_receive_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_example_espnow_queue);
        vSemaphoreDelete(semaphore_send);
        vSemaphoreDelete(semaphore_receive);
        esp_now_deinit();
        return ESP_FAIL;
    }
    ESP_LOGI(USER_TAG, "Adding peer info");
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_parameters = malloc(sizeof(example_espnow_send_param_t));
    if (send_parameters == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(send_parameters, 0, sizeof(example_espnow_send_param_t));
    send_parameters->unicast = false;
    send_parameters->broadcast = true;
    send_parameters->state = 0;
    send_parameters->magic = device_role;
    send_parameters->count = CONFIG_ESPNOW_SEND_COUNT;
    send_parameters->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_parameters->len = CONFIG_ESPNOW_SEND_LEN;
    send_parameters->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_parameters->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_parameters);
        vSemaphoreDelete(s_example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_parameters->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
    //example_espnow_data_prepare(send_param);

    //xTaskCreate(example_espnow_task, "example_espnow_task", 8048, send_parameters, 4, NULL);
    return ESP_OK;
}

// This callback function needs to verify if the data has been sent succesfully then unblock send task
// If the send was successfull then destroy the previously sent data in queue
// Data destruction is performed by dropping the data from a queue into a unused variable
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // example_espnow_event_t evt;
    // example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    // evt.id = EXAMPLE_ESPNOW_SEND_CB;
    // memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    // send_cb->status = status;
    // if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
    //     ESP_LOGW(TAG, "Send send queue fail");
    // }
    void * data_dump = NULL;
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        xQueueReceive(espnow_com_queue, data_dump, portMAX_DELAY);
    }
    xSemaphoreGive(semaphore_send);
}

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    // example_espnow_event_t evt;
    // example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    // uint8_t * mac_addr = recv_info->src_addr;
    // uint8_t * des_addr = recv_info->des_addr;
    image_data_raw image_data;

    if (recv_info->src_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(USER_TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(recv_info->des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(USER_TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(USER_TAG, "Receive unicast ESPNOW data");
    }
    uint32_t *received_data = malloc(len);
    memcpy(received_data, data, len); 
    ESP_LOGD(USER_TAG, "stored received data locally and sending to queue");
    image_data.data = received_data;
    image_data.len = len;
    xQueueSend(espnow_com_queue, &image_data, portMAX_DELAY);
}
/* Prepare ESPNOW data to be sent. */
//for now very similar to data structure to example code, just gutted the filler random numbers 
// This funciton should be called only when message queue is empty which block data_send task
// accesses memory location dedicated for ADC image data storage 
//This task sends data to message queue which will be read by sending task
void espnow_data_prep_task(void *pv_parameters)
{
    //example_espnow_send_param_t *send_param = NULL;
    espnow_data *buf = send_parameters->buffer;
    xQueueReceive(image_queue, buf->payload, portMAX_DELAY);

    
    assert(send_parameters->len >= sizeof(espnow_data));

    buf->type = IS_BROADCAST_ADDR(send_parameters->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_parameters->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_parameters->magic;
    // /* Fill all remaining bytes after the data with random values */
    // esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_parameters->len);

    send_parameters->buffer = buf;
    //is there a way to notify if the queue is full?
    xQueueSend(espnow_com_queue, send_parameters, portMAX_DELAY);
}
//This task needs to peek messages containing image data and send it using esp_now_send
// Meggase data sould be dequeued in callback function
void espnow_send_data_task(void *pv_parameters)
{
    example_espnow_send_param_t *send_param = NULL;
    for(;;)
    {
        ESP_LOGD(USER_TAG ,"Taking taking send semaphore: send_data_task");
        xSemaphoreTake(semaphore_send, portMAX_DELAY);
        ESP_LOGD(USER_TAG ,"Taking data from queue: send_data_task");
    
        xQueuePeek(espnow_com_queue, send_param, portMAX_DELAY);
        ESP_LOGD(TAG ,"Sending image data via ESPNOW: send_data_task");
        if (esp_now_send(send_param->dest_mac, send_param->buffer, sizeof(send_param->len)) != ESP_OK)
        {
            ESP_LOGE(USER_TAG, "Failed to send data to destination address: send_data_task");
        }
    }
}

void TEST_espnow_stage_data_task(void *pv_parameters)
{
    for(;;)
    {   
        ESP_LOGI(USER_TAG, "data staging task: Generating new data");
        xQueueSend(image_queue, s_image_data, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

// This function should be called from the main app and should initialize both the send and data prep tasks 
void xinit_send_data_tasks()
{
    
    example_espnow_send_param_t *pv_parameters = NULL;
    xTaskCreatePinnedToCore(*espnow_send_data_task, "espnow_send_data_task", 4000, pv_parameters, ESPNOW_SEND_TASK_PRIORITY, espnow_send_data_taskHandle, 0);
    xTaskCreatePinnedToCore(*espnow_data_prep_task, "espnow_data_prep_task", 4000, pv_parameters, ESPNOW_DATA_PREP_TASK_PRIORITY, espnow_data_prep_taskHandle, 0);
    xTaskCreatePinnedToCore(*TEST_espnow_stage_data_task, "TEST_espnow_stage_data_task", 2000, pv_parameters, ESPNOW_SEND_TASK_PRIORITY,TEST_espnow_stage_data_taskhandle, 0);
    return;
}

//This task should grab the ponter for the received data and stage it for parsing
// Image queue will be used for sending to data parsing task
// This task should chech data integrity
void espnow_receive_data_task()
{   for(;;)
    {
        image_data_raw image_data;
        uint8_t *data = NULL;
        ESP_LOGD(USER_TAG, "Reading data from com_queue for data receptioon task: receive_data_task");
        xQueueReceive(espnow_com_queue, &image_data, portMAX_DELAY);

        //This portion should be for checking the crc value of the data

        //This portion should be for checking the crc value of the data

        ESP_LOGD(USER_TAG, "sending data to for data receptioon task: receive_data_task");
        xQueueSend(image_queue, &image_data, portMAX_DELAY);

    }
    
}

//After this task is done with parsing it should send the data off-chip and free the allocated memory
// This should integrate with USB communication to PC for data transfer
// At this point the received data should be sent on to the nexto core and free the allocated memory
void espnow_parse_data_task()
{
    for(;;)
    {
        image_data_raw *image_data = NULL;
        xQueueReceive(image_queue, image_data, portMAX_DELAY);
        ESP_LOGI(USER_TAG, "received image size of %d", image_data->len);
        for(int i = 0; i < image_data->len; i++)
        {
            ESP_LOGI(USER_TAG, "\n The %dnth received data is %d", i, *(image_data->data+i));
        }
        
        free(image_data->data);

    }
}

void xinit_receive_data_tasks()
{
    example_espnow_send_param_t *pv_parameters = NULL;
    xTaskCreatePinnedToCore(*espnow_receive_data_task, "espnow_receive_data_task", 4000, pv_parameters, ESPNOW_SEND_TASK_PRIORITY, espnow_send_data_taskHandle, 0);
    xTaskCreatePinnedToCore(*espnow_parse_data_task, "espnow_parse_data_task", 4000, pv_parameters, ESPNOW_DATA_PREP_TASK_PRIORITY, espnow_data_prep_taskHandle, 0);
    return;
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    if (run_example == true)
    {   
        //Example implementation
        example_wifi_init();
        example_espnow_init();
    }else
    {
        // Start of user implementation
        ESP_LOGI(USER_TAG, "Starting User code in main app");
        user_espnow_init();
        switch (device_role)
        {
        case DEVICE_ROLE_SENDER:
            xinit_send_data_tasks();
            break;
        case DEVICE_ROLE_RECIEVER:
            xinit_receive_data_tasks();
            break;
        default:
            ESP_LOGE(USER_TAG, "This device does not have an assigned role. \nCheck periferal connections and program code");
            break;
        }
    }
    
}

