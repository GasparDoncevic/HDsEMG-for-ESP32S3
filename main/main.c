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
#include "main.h"
#include "AFE_controll.h"



#define ESPNOW_MAXDELAY 512

//USER DEFINES
#define  DEVICE_ROLE_SENDER 1
#define  DEVICE_ROLE_RECIEVER 0
#define  IMAGE_SIZE 16
#define ESPNOW_SEND_TASK_PRIORITY 4
#define ESPNOW_DATA_PREP_TASK_PRIORITY 5
#define TEST_GENERATE_DATA_TASK_PRIORITY 6
//USER DEFINES


//USER PRIVATE VARIABLES
bool run_example = false;
uint8_t device_role = DEVICE_ROLE_RECIEVER;

TaskHandle_t espnow_send_data_taskHandle = NULL;
TaskHandle_t espnow_data_prep_taskHandle = NULL;
TaskHandle_t TEST_espnow_stage_data_taskhandle = NULL;
TaskHandle_t init_tasks_handle = NULL;
TaskHandle_t Handle_Task_AFE_init_tasks = NULL;
TaskHandle_t Handle_TEST_data_transfer_send = NULL;
TaskHandle_t Handle_TEST_data_transfer_recv = NULL;

QueueHandle_t queue_image = NULL; // queue for raw image data, between memory location and data prep task
static QueueHandle_t queue_espnow_stage = NULL; // queue for send data, between data prep task and send task
SemaphoreHandle_t semaphore_send = NULL;
SemaphoreHandle_t semaphore_receive = NULL;
//SemaphoreHandle_t semaphore_image = NULL;

static const char *USER_TAG = "user_espnow";
//USER PRIVATE VARIABLES
static const char *TAG = "espnow_example";

static QueueHandle_t s_example_espnow_queue;

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };


static void example_espnow_deinit(example_espnow_send_param_t *send_param);

//function prototypes
void TEST_espnow_data_print(espnow_data_t* data);

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

    if(semaphore_receive == NULL)
    {
        ESP_LOGE(USER_TAG, "Failed receive semaphore create");
        return ESP_FAIL;
    }
    if(semaphore_send == NULL)
    {
        ESP_LOGE(USER_TAG, "Failed send semaphore create");
        return ESP_FAIL;
    }

    /* s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (s_example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    } */

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
    // esp_now_rate_config_t *config = malloc(sizeof(esp_now_rate_config_t));
    // config->phymode = WIFI_PHY_MODE_HT40; // HT stand for high throughput with a bandiwth of 40MHz
    // config->rate = WIFI_PHY_RATE_MAX;
    // config->dcm = false;
    // config->ersu = false;
    //esp_now_set_peer_rate_config(peer->peer_addr, config);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);
    //free(config);


    // Setting queues
    ESP_LOGI(USER_TAG, "Size of espnow data: %d", sizeof(espnow_data_t));
    queue_espnow_stage = xQueueCreate(200, sizeof(uint32_t));
    queue_image = xQueueCreate(200, sizeof(uint32_t)); // this hardcoded value needs to be changed for a datatype of image_data
    
    // Setting queues

    return ESP_OK;
}

// This callback function needs to verify if the data has been sent succesfully then unblock send task
// If the send was successfull then destroy the previously sent data in queue
// Data destruction is performed by dropping the data from a queue into a unused variable
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // example_espnow_event_t evt;
    // example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
    ESP_LOGI(USER_TAG, "Entered send callback function");
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    espnow_send_param_t *data_dump = NULL;
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(USER_TAG, "Removing successfully sent data from queue");
        xQueueReceive(queue_espnow_stage, &data_dump, portMAX_DELAY);
        ESP_LOGI(USER_TAG, "freeing sent data from memory at location %p", data_dump->buffer);
        ESP_LOGI(USER_TAG, "freeing send_param from memory at location %p", data_dump);
        //TEST_espnow_data_print(data_dump->buffer);
        free((espnow_data_t *)(data_dump->buffer));
        free((espnow_send_param_t *)data_dump);
    }
    //ESP_LOGI(USER_TAG, "Giving back semaphore");
    xSemaphoreGive(semaphore_send);
    ESP_LOGI(USER_TAG, "Exiting send callback function");

    return;
}

static void espnow_receive_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    // example_espnow_event_t evt;
    // example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    // uint8_t * mac_addr = recv_info->src_addr;
    // uint8_t * des_addr = recv_info->des_addr;
    // image_data_raw_t image_data;

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
    ESP_LOGD(USER_TAG, "Length of received data is %d", len);
    memcpy(received_data, data, len); 
    ESP_LOGD(USER_TAG, "stored received data locally and sending to queue");
    //image_data.data = received_data;
    //image_data.len = len;
    xQueueSend(queue_espnow_stage, &received_data, portMAX_DELAY);

    return;
}
/* Prepare ESPNOW data to be sent. */
//for now very similar to data structure to example code, just gutted the filler random numbers 
// This funciton should be called only when message queue is empty which block data_send task
// accesses memory location dedicated for ADC image data storage 
//This task sends data to message queue which will be read by sending task
// Allocates memory for espnow_data which will be freed in the send callback function
void espnow_data_prep_task(void *pv_parameters)
{
    for(;;)
    {
        
        //A new send parameters is allocated and will be freed in send callback function
        
        espnow_send_param_t* send_parameters = malloc(sizeof(espnow_send_param_t));
        if (send_parameters == NULL) {
            ESP_LOGE(TAG, "Malloc send parameter fail");
        }
        ESP_LOGD(USER_TAG, "data_prep_task: creating new instance of send parameters at address %p", send_parameters);
        //memset(send_parameters, 0, sizeof(espnow_send_param_t));
        //send_parameters->unicast = false;
        send_parameters->broadcast = true;
        //send_parameters->state = 0;
       // send_parameters->count = CONFIG_ESPNOW_SEND_COUNT;
        //send_parameters->delay = CONFIG_ESPNOW_SEND_DELAY;
        send_parameters->len = sizeof(espnow_data_t);
        send_parameters->buffer = malloc(sizeof(espnow_data_t));
        if (send_parameters == NULL) {
            ESP_LOGE(USER_TAG, "Malloc buffer fail");
        }
        //ESP_LOGI(USER_TAG, "data_prep_task: creating new instance of espnow_data at address %p", send_parameters->buffer);
        //memset(send_parameters->buffer, 0, sizeof(espnow_data_t));
        memcpy(send_parameters->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
        
        
        //espnow_data *buf = (espnow_data *) send_parameters->buffer;
        image_data_raw_t *image_data = NULL;
        //ESP_LOGI(USER_TAG, "data_prep_task: Taking image semaphore");
        //xSemaphoreTake(semaphore_image, portMAX_DELAY/portTICK_PERIOD_MS);
        ESP_LOGI(USER_TAG, "data_prep_task: Taking from image queue");
        if (pdFAIL == xQueueReceive(queue_image, &image_data, portMAX_DELAY))
        {
            ESP_LOGE(USER_TAG, "Failed to indefinitely block on queue recieve");
        }
        //ESP_LOGI(USER_TAG, "data_prep_task: giving image semaphore");
        //xSemaphoreGive(semaphore_image);
        espnow_data_t *buf = send_parameters->buffer;

        ESP_LOGD(USER_TAG, "data_prep_task: Creating a new espnow_data packet on location %p", buf);
        ESP_LOGD(USER_TAG, "data_prep_task: Copying data from location %p to new location %p", image_data, (void *) &(buf->payload));
        ESP_LOGD(USER_TAG, "The fetched image data is located on %p", &(image_data->data));

        memcpy(&(buf->payload), &(image_data->data), AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*2);
        buf->len_payload = image_data->len;
        ESP_LOGD(USER_TAG, "data_prep_task: freeing allocated image data on location %p", image_data);
        free(image_data);

        ESP_LOGD(USER_TAG, "size of send_parameters->len: %d", send_parameters->len);
        ESP_LOGD(USER_TAG, "size of espnow_data: %d", sizeof(espnow_data_t));
        assert(send_parameters->len >= sizeof(*buf));
        

        buf->type = IS_BROADCAST_ADDR(send_parameters->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
        buf->state = 0;
        buf->seq_num = s_example_espnow_seq[buf->type]++;
        buf->crc = 0;
        buf->magic = device_role;
        buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_parameters->len);

        //send_parameters->buffer = buf;
        ESP_LOGD(USER_TAG, "Data_prep_task: Address of buf: %p; address of send_parameter->buffer: %p", buf, send_parameters->buffer);
        //TEST_espnow_data_print(buf);
        //TEST_espnow_data_print(send_parameters->buffer);
        //is there a way to notify if the queue is full?
        xQueueSend(queue_espnow_stage, &send_parameters, portMAX_DELAY/portTICK_PERIOD_MS);
        }
    
}
//This task needs to peek messages containing image data and send it using esp_now_send
// Meggase data sould be dequeued in callback function
void espnow_send_data_task(void *pv_parameters)
{

    //example_espnow_send_param_t *send_param = NULL;
    for(;;)
    {
        espnow_send_param_t *send_parameters = NULL;
        //ESP_LOGI(USER_TAG ,"send_data_task: Taking send semaphore");
        xSemaphoreTake(semaphore_send, portMAX_DELAY);

        ESP_LOGI(USER_TAG ,"send_data_task: Taking data from queue");
        xQueuePeek(queue_espnow_stage, &send_parameters, portMAX_DELAY);
        TEST_espnow_data_print(send_parameters->buffer);
        ESP_LOGD(USER_TAG, "send_data_task: address of received send_parameters %p", send_parameters);
        ESP_LOGD(USER_TAG, "send_data_task: Send espnow_data from location %p", (send_parameters->buffer));
        TEST_espnow_data_print(send_parameters->buffer);
        if (esp_now_send(send_parameters->dest_mac, (uint8_t *)(send_parameters->buffer), send_parameters->len) != ESP_OK)
        {
            ESP_LOGE(USER_TAG, "send_data_task: Failed to send data to destination address");
            xSemaphoreGive(semaphore_send);
            vTaskDelay(10000/portTICK_PERIOD_MS);
        }
    }
}

// Task generate
void TEST_espnow_stage_data_task(void *pv_parameters)
{
    for(;;)
    {   
        image_data_raw_t *image = malloc(sizeof(image_data_raw_t));
        memset(&(image->data), 0x0A, AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*2);
        image->len = AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH;
        ESP_LOGI(USER_TAG, "data generating task: generating new image data at location %p", (void *) image);
        //ESP_LOGI(USER_TAG, "data staging task: taking image semaphore");
        //xSemaphoreTake(semaphore_image, portMAX_DELAY/portTICK_PERIOD_MS);
        ESP_LOGI(USER_TAG, "data generating task: putting image into queue");
        xQueueSend(queue_image, &image, portMAX_DELAY/portTICK_PERIOD_MS);
        //ESP_LOGI(USER_TAG, "data staging task: giving image semaphore");
        //xSemaphoreGive(semaphore_image);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

// This function should be called from the main app and should initialize both the send and data prep tasks 
void xinit_send_data_tasks()
{
    // Creating image queue semaphore
    // semaphore_image = xSemaphoreCreateBinary();
    // xSemaphoreGive(semaphore_image);
    // if(semaphore_image == NULL)
    // {
    //     ESP_LOGE(USER_TAG, "image queue semaphore was not created");
    // }
    uint8_t task_count = 0;
    espnow_send_param_t *pv_parameters = NULL;
    if (queue_espnow_stage == NULL)
    {
        ESP_LOGE(USER_TAG, "espnow staging queue was not initialized properly");
    }
    if (queue_image == NULL)
    {
        ESP_LOGE(USER_TAG, "image queue was not initalized properly");
    }

    if(xTaskCreatePinnedToCore(espnow_send_data_task, "espnow_send_data_task", 3000, pv_parameters, ESPNOW_SEND_TASK_PRIORITY, &espnow_send_data_taskHandle, 0) == pdPASS) task_count++;
    if(xTaskCreatePinnedToCore(espnow_data_prep_task, "espnow_data_prep_task", 3000, pv_parameters, ESPNOW_DATA_PREP_TASK_PRIORITY, &espnow_data_prep_taskHandle, 0) == pdPASS) task_count++;
    //if(xTaskCreatePinnedToCore(TEST_espnow_stage_data_task, "TEST_espnow_stage_data_task", 3000, pv_parameters, TEST_GENERATE_DATA_TASK_PRIORITY, &TEST_espnow_stage_data_taskhandle, 1) == pdPASS) task_count++;
    if(task_count == 2)
    {
        ESP_LOGI(USER_TAG, "All tasks were created successfully!");
    }else
    {
        ESP_LOGE(USER_TAG, "There was an error during task creation!");
    }
    // Checking if all mesage queues are properly initialized
    

    //This part of code should also create the tasks for AD converter

    return;
}

//This task should grab the ponter for the received data and stage it for parsing
// Image queue will be used for sending to data parsing task
// This task should chech data integrity
void espnow_receive_data_task()
{   for(;;)
    {
        espnow_data_t *espnow_data;
        //uint8_t *data = NULL;
        ESP_LOGD(USER_TAG, "Reading data from com_queue for data receptioon task: receive_data_task");
        xQueueReceive(queue_espnow_stage, &espnow_data, portMAX_DELAY);

        //This portion should be for checking the crc value of the data

        //This portion should be for checking the crc value of the data

        ESP_LOGD(USER_TAG, "sending data to for data receptioon task: receive_data_task");
        xQueueSend(queue_image, &espnow_data, portMAX_DELAY);

    }
    
}

//After this task is done with parsing it should send the data off-chip and free the allocated memory
// This should integrate with USB communication to PC for data transfer
// At this point the received data should be sent on to the nexto core and free the allocated memory
void espnow_parse_data_task()
{
    for(;;)
    {
        espnow_data_t *image_data = NULL;
        xQueueReceive(queue_image, &image_data, portMAX_DELAY);
        //ESP_LOGI(USER_TAG, "received image size of %d", image_data->len);
        //TODO: Send Data via serial connection to PC, would be wise to implement message queue and shift job to task on other core
        for(int i = 0; i < image_data->len_payload; i++)
        {
            ESP_LOGI(USER_TAG, "The %dnth received data is 0x%x \n", i, (uint16_t)(image_data->payload[i]));
        }
        free(image_data);
       

    }
}

void xinit_receive_data_tasks()
{
    example_espnow_send_param_t *pv_parameters = NULL;
    if (queue_espnow_stage == NULL)
    {
        ESP_LOGE(USER_TAG, "espnow staging queue was not initialized properly");
    }
    if (queue_image == NULL)
    {
        ESP_LOGE(USER_TAG, "image queue was not initalized properly");
    }
    uint8_t tasks = 0;
    if (xTaskCreatePinnedToCore(espnow_receive_data_task, "espnow_receive_data_task", 3000, pv_parameters, ESPNOW_SEND_TASK_PRIORITY, &espnow_send_data_taskHandle, 0)) tasks++;
    if (xTaskCreatePinnedToCore(espnow_parse_data_task, "espnow_parse_data_task", 3000, pv_parameters, ESPNOW_DATA_PREP_TASK_PRIORITY, &espnow_data_prep_taskHandle, 0)) tasks++;

    if (tasks == 2)
    {
        ESP_LOGI(USER_TAG, "successfully created receiveing tasks");
    }
    

    //This part of the code should also create the tasks for the USB communication with external PC
    return;
}

void init_tasks()
{        
    // Start of user implementation
    ESP_LOGI(USER_TAG, "Starting User code in main app");
    example_wifi_init();

    user_espnow_init();
    if (device_role == DEVICE_ROLE_SENDER)
    {
        ESP_LOGI(USER_TAG, "Starting sender tasks");
        xTaskCreatePinnedToCore(Task_init_AFE_tasks, "TASK_init_AFE_tasks", 4000, NULL, configMAX_PRIORITIES-1, &Handle_Task_AFE_init_tasks, 1);
        xinit_send_data_tasks();
    }else if(device_role == DEVICE_ROLE_RECIEVER)
    {
        ESP_LOGI(USER_TAG, "Starting receiver tasks");
        xinit_receive_data_tasks();
    }else
    {
        ESP_LOGE(USER_TAG, "This device does not have an assigned role. \nCheck periferal connections and program code");
    }

    // Task deletes itself after finishing initialization
    vTaskDelete(NULL);
}

//this is a test function to display all of the data in the in the allocated espnow data structure
void TEST_espnow_data_print(espnow_data_t* data)
{
    ESP_LOGD(USER_TAG, "The address of the structure is %p", data);
    if (data->type == EXAMPLE_ESPNOW_DATA_BROADCAST)
    {
        ESP_LOGD(USER_TAG, "Boradcast type is BROADCAST");
    }else if(data->type == EXAMPLE_ESPNOW_DATA_UNICAST)
    {
        ESP_LOGD(USER_TAG, "Boradcast type is UNICAST");
    }

    ESP_LOGD(USER_TAG, "Payload size is %d", data->len_payload);
    // max size of the structure for now is 42 and the last 32 bytes are payload data packed in uint16_t type
    for(int i = 0; i < data->len_payload; i++)
    {
        ESP_LOGD(USER_TAG, "the %dth payload data is 0x%x", i ,(uint16_t)(data->payload[i]));
    }   

    return;
}

void TEST_core_data_transfer_recv(QueueHandle_t queue)
{
    uint32_t i = 0;
    for(;;)
    {
        
        uint32_t * value = malloc(sizeof(uint32_t));
        *value = i;
        ESP_LOGI(USER_TAG, "Sending allocated data on location %p witch value %ud from core 0", value, (unsigned int)*value);
        xQueueSend(queue, &value, portMAX_DELAY);
        i++;
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    

}
void TEST_core_data_transfer_send(QueueHandle_t queue)
{
    for(;;)
    {   
        uint32_t * recv_val = NULL;
        xQueueReceive(queue, &recv_val, portMAX_DELAY);
        ESP_LOGI(USER_TAG, "Recieved data on on core 1 from address %p with data %ud", recv_val, (unsigned int)*recv_val);
        vTaskDelay(200/portTICK_PERIOD_MS); 
    }

}

void TEST_core_data_transfer_init()
{
     QueueHandle_t queue_TEST = xQueueCreate(200, sizeof(uint32_t));
     xTaskCreatePinnedToCore(TEST_core_data_transfer_send, "Send_data", 3000, queue_TEST, 4, &Handle_TEST_data_transfer_send, 0);
     xTaskCreatePinnedToCore(TEST_core_data_transfer_recv, "Recieve_data", 3000, queue_TEST, 4, &Handle_TEST_data_transfer_recv, 1);

    for(;;)
    {
        vTaskDelay(300000/portTICK_PERIOD_MS);
    }
     
}
void TEST_espnow_transfer()
{
    example_wifi_init();
    user_espnow_init();
    uint8_t task_count = 0;
    if(device_role == DEVICE_ROLE_SENDER)
    {
       espnow_send_param_t *pv_parameters = NULL;
        if (queue_espnow_stage == NULL)
        {
            ESP_LOGE(USER_TAG, "espnow staging queue was not initialized properly");
        }
        if (queue_image == NULL)
        {
            ESP_LOGE(USER_TAG, "image queue was not initalized properly");
        }

        if(xTaskCreatePinnedToCore(espnow_send_data_task, "espnow_send_data_task", 3000, pv_parameters, ESPNOW_SEND_TASK_PRIORITY, &espnow_send_data_taskHandle, 0) == pdPASS) task_count++;
        if(xTaskCreatePinnedToCore(espnow_data_prep_task, "espnow_data_prep_task", 3000, pv_parameters, ESPNOW_DATA_PREP_TASK_PRIORITY, &espnow_data_prep_taskHandle, 0) == pdPASS) task_count++;
        if(xTaskCreatePinnedToCore(TEST_espnow_stage_data_task, "TEST_espnow_stage_data_task", 3000, pv_parameters, TEST_GENERATE_DATA_TASK_PRIORITY, &TEST_espnow_stage_data_taskhandle, 0) == pdPASS) task_count++;
        if(task_count == 3)
        {
            ESP_LOGI(USER_TAG, "All tasks were created successfully!");
        }else
        {
            ESP_LOGE(USER_TAG, "There was an error during task creation!");
        }

    }else if(device_role == DEVICE_ROLE_RECIEVER)
    {
        xinit_receive_data_tasks();
    }
    
}


void Start_App()
{
    xTaskCreatePinnedToCore(init_tasks, "init_tasks", 3000, NULL, configMAX_PRIORITIES-1, &init_tasks_handle,0);
    if(device_role == DEVICE_ROLE_SENDER)
    {
        
    }
    
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

    // Creating task for testing espnow
    
    //Creating task for initializing and testing SPI
    //TEST_GPIO();
    //TEST_CLKSRC();
    //TEST_SPI();
    //Start_App();
    TEST_espnow_transfer();
    //TEST_GPtimer();
    //xTaskCreatePinnedToCore(TEST_core_data_transfer_init, "Test_core_transfer", 3000, NULL, configMAX_PRIORITIES-1, &Handle_Task_AFE_init_tasks, 0);
    
    

    return;
    
}

