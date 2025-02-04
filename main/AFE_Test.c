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
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "AFE_controll.h"
#include "AFE_config.h"


// strucutre for data which will be recieved by spi slave
typedef struct
{
    uint8_t data[AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET];
} AFE_data_t;


//Task handles for production code
extern TaskHandle_t Handle_Task_AFE_init;
extern TaskHandle_t Handle_Task_Stage_data;
extern TaskHandle_t Handle_TASK_Get_data;
extern TaskHandle_t Handle_Task_AFE_init_tasks;
//Task Handles for TESTS
TaskHandle_t Handle_TEST_spi_loop = NULL;
TaskHandle_t Handle_Task_TEST_loopback_receiver = NULL;
TaskHandle_t Handle_Task_TEST_loopback_sender = NULL;
TaskHandle_t Handle_Task_TEST_GPIO = NULL;
TaskHandle_t Handle_Task_TEST_CLKSRC = NULL;
TaskHandle_t Handle_TEST_Task_gen_data = NULL;
TaskHandle_t Handle_TEST_AFE_subsystem = NULL;
TaskHandle_t Handle_TEST_AFE_commands = NULL;
TaskHandle_t Handle_TEST_GPtimer = NULL;
TaskHandle_t Handle_TEST_Task_listen = NULL;
//uint32_t TEST_data = 0;

// TEST global variables THESE SHOULD BE COMMENTED OUT WHEN NOT TESTING
uint8_t data_mock[AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET] = {0xAA};
spi_transaction_t transaction_mock = {
        .rx_buffer = NULL,
        .tx_buffer = &data_mock,
        .length = AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET*8,
        .rxlength = 0,
        .flags = 0
    };
// TEST global variables THESE SHOULD BE COMMENTED OUT WHEN NOT TESTING


extern int AFE_Send_Command(spi_device_handle_t spi_device, retry will_retry, uint8_t address, uint8_t reg_value);
extern uint16_t AFE_command_get_response(spi_device_handle_t spi_device, spi_transaction_t * transaction_get_response);
extern esp_err_t AFE_set_SPI_controll_mode(bool SPI_mode);
extern esp_err_t AFE_config_clk_source();
extern esp_err_t AFE_Init_Sync_timer();
extern void AFE_sync_chain();
extern void Timer_sync_alarm();
extern esp_err_t AFE_Init_Sync_timer();
extern esp_err_t AFE_reset(bool use_spi);

extern QueueHandle_t queue_AFE_data;
extern QueueHandle_t queue_image;
extern SemaphoreHandle_t semaphore_sync;
extern SemaphoreHandle_t semaphore_spi;


const char *TAG_AFE_TEST = "AFE_TEST";

extern spi_slave_interface_config_t device_spi_slave;
extern spi_device_interface_config_t device_spi_master[AFE_NUM_OF_ADC];
extern spi_bus_config_t bus_spi_slave, bus_spi_master;
extern spi_device_handle_t spi_master[AFE_NUM_OF_ADC];
extern uint8_t spi_master_CS_pins[4];
extern gptimer_handle_t gptimer;


// This TEST task is created when the testing GPTimer is used to generate data
// This TEST tasks takes a semapohore and sends mock data using the spi master
// This function assumes the spi master and slave are connected during testing
void TEST_Task_generate_data_w_SPI()
{
    for(;;)
    {
        ESP_LOGV(TAG_AFE_TEST, "Taking semaphore to generate new data and send to SPI");
        xSemaphoreTake(semaphore_sync, portMAX_DELAY);
        ESP_LOGV(TAG_AFE_TEST, "Generator semaphore Taken");
       // ESP_LOGI(TAG_AFE_TEST, "Sending transaciton located on %p which has data on %p", &transaction_mock, &data_mock);
       /*  for (uint8_t i = 0; i < sizeof(AFE_data_t); i++)
        {
            ESP_LOGI(TAG_AFE_TEST, "Data to send is 0x%x", data_mock[i] ); 
        } */

        if (ESP_OK != spi_device_queue_trans(spi_master[0], &transaction_mock, 0))
        {
            ESP_LOGE(TAG_AFE_TEST, "Failed to queue data to spi master ");
        }else
        {
            ESP_LOGI(TAG_AFE_TEST, "sending new data via SPI");
        }
        // a taskYield is needed here to be invoked
        //taskYIELD();
        //vTaskDelay(1000);
        
    }
}


// This is a function called in the GPTimer ISR and gives the semaphore so that the blocking task
// TEST_Task_generate_data_w_SPI() can generate one new mock data
void TEST_MOCK_AFE_create_data()
{
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //ESP_LOGI(TAG_AFE_TEST, "Giving back generator semaphore");
    if (pdTRUE != xSemaphoreGiveFromISR(semaphore_sync, &xHigherPriorityTaskWoken))
    {
        //ESP_LOGE(TAG_AFE_TEST, "Failed to give back semaphore");
        return;
    }
    //spi_device_transmit(spi_master[0], &transaction_mock);
    // a yield is needed is a higher priority task is awoken
    // which it is, because the semaphoreGive always triggers a high priority task to generate data
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    

    return;
}

 /* void Timer_sync_alarm()
{
    // this code is commented out during testing without AFE hardware
    //AFE_sync_chain();
    AFE_sync_chain();
    //ESP_LOGI(TAG_AFE, "Hello from timer alarm");
    //TEST_GPTimer_hello();
    //TEST_MOCK_AFE_create_data();
    //TEST_data++;
}  */

/* esp_err_t AFE_Init_Sync_timer()
{
    ESP_LOGI(TAG_AFE_TEST, "Starting GPtimer init");
    gptimer_config_t gptimer_config;
    gptimer_config.clk_src = GPTIMER_CLK_SRC_APB;// giving the timer the max possible clock of 80MHz
    gptimer_config.direction = GPTIMER_COUNT_UP;
    gptimer_config.resolution_hz = 1000*10;
    gptimer_config.intr_priority = 0;
    gptimer_alarm_config_t gptimer_alarm;
    gptimer_alarm.alarm_count = 10000/TEST_GEN_ODR; // sets the value when the alarm triggers, a second will have 10000 ticks, and the needed mock ODR is 2kSPs (2000)
    gptimer_alarm.flags.auto_reload_on_alarm = true; // when alarm is triggered, count is reloaded
    gptimer_alarm.reload_count = 0; // count autoreloads to 0

    ESP_ERROR_CHECK(gptimer_new_timer(&gptimer_config, &gptimer));
    gptimer_event_callbacks_t callback; 
    callback.on_alarm = (void*)Timer_sync_alarm;

    ESP_LOGI(TAG_AFE_TEST, "starting timer");
    // Generating semaphore which will be used for the task to generate mock data
    semaphore_sync = xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &callback, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &gptimer_alarm));
    ESP_ERROR_CHECK(gptimer_start(gptimer));


    // generating task which will send new data via SPI
    xTaskCreatePinnedToCore(TEST_Task_generate_data_w_SPI, "Generate_data_w_spi", 3000, NULL, PRIORITY_TASK_STAGE_DATA +5, &Handle_TEST_Task_gen_data, 1);
    ESP_LOGI(TAG_AFE_TEST, "GPTimer and task configured");

    return ESP_OK;
} */

// Function intended for seeing if the spi pins really do output the correct data and if the signals are correctly routed
// spi3 is slave, spi2 is master
void Task_TEST_loopback_sender()
{   
    bool use_command = false;
    spi_transaction_t data_spi2;
    memset(&data_spi2, 0, sizeof(spi_transaction_t));
    uint8_t data[2] = {0x04, 0x04};
    data_spi2.length = 16;
    data_spi2.tx_buffer = &data;
    data_spi2.rx_buffer = NULL;
    for(;;)
    {   
        if(use_command == false)
        {
            ESP_LOGI(TAG_AFE_TEST, "Sending data via spi on master device");
            spi_device_transmit(spi_master[0], &transaction_mock);
            vTaskDelay(500/portTICK_PERIOD_MS);
            data[1]++;
        }else
        {
            ESP_LOGI(TAG_AFE_TEST, "Sending command via command API");
            AFE_Send_Command(spi_master[0], NO_RETRY, data[0], data[1]);
            vTaskDelay(500/portTICK_PERIOD_MS);
            data[1]++;
        }
        
    }
}
void Task_TEST_loopback_receiver()
{
    spi_slave_transaction_t data_spi3;
    memset(&data_spi3, 0, sizeof(spi_slave_transaction_t));
    uint8_t data[AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET];
    data_spi3.length = sizeof(AFE_data_t)*8;
    data_spi3.rx_buffer = &data;
    data_spi3.tx_buffer = NULL;
    spi_slave_transaction_t * spi_result = NULL;
    //A delay needs to be added for the slave configuration
    vTaskDelay(50/portTICK_PERIOD_MS);
    for(;;)
    {
        /* ESP_LOGI(TAG_AFE_TEST, "Recieving data via spi on slave device");
        spi_slave_transmit(SPI3_HOST, &data_spi3, portMAX_DELAY);        
        ESP_LOGI(TAG_AFE_TEST, "Recieved data is 0x%x", recieved_data); 
        vTaskDelay(1); */
        ESP_LOGI(TAG_AFE_TEST, "Recieving data via spi on slave device");
        spi_slave_queue_trans(SPI3_HOST, &data_spi3, portMAX_DELAY);

        
        spi_slave_get_trans_result(SPI3_HOST, &spi_result, portMAX_DELAY);
        ESP_LOGI(TAG_AFE_TEST, "Recieved new packet");
        for (uint8_t i = 0; i < sizeof(AFE_data_t)/2; i++)
        {
            ESP_LOGD(TAG_AFE_TEST, "Recieved data is 0x%x", *((uint16_t *)(spi_result->rx_buffer) +1) ); 
        }
        
        vTaskDelay(100);
    }
}
void TEST_spi_loopback()
{
    
    
    xTaskCreatePinnedToCore(Task_TEST_loopback_receiver, "Receiver", 3000, NULL, 3, &Handle_Task_TEST_loopback_receiver, 0);
    xTaskCreatePinnedToCore(Task_TEST_loopback_sender, "Sender", 3000, NULL, 3, &Handle_Task_TEST_loopback_sender, 1);
    for(;;)
    {
        vTaskDelete(NULL);    
        //ESP_LOGI(TAG_AFE_TEST, "Waiting a bit before sending");
        //vTaskDelay(500/portTICK_PERIOD_MS);
        //vTaskDelay(1);
        //data++;
    }
    

}

void TEST_SPI()
{
    ESP_LOG_LEVEL_SET(TAG_AFE_TEST, ESP_LOG_DEBUG);
    // Filling up test resource
    memset(&data_mock, 0x03, AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET); 
    // Filling up test resource
    xTaskCreatePinnedToCore(Task_AFE_init, "Task_AFE_Init", 4000, NULL, configMAX_PRIORITIES-1, &Handle_Task_AFE_init, 1);
    xTaskCreatePinnedToCore(TEST_spi_loopback, "TEST_spi_loop", 3000, NULL, 6, &Handle_TEST_spi_loop, 1);

}

void Task_TEST_GPIO()
{
    uint8_t mode = 0;
    for(;;)
    { 
        mode++;
        mode = mode % 2;
        ESP_LOGI(TAG_AFE_TEST, "Changing controll mode %d", mode);
        if (ESP_OK != AFE_set_SPI_controll_mode(mode)) ESP_LOGE(TAG_AFE_TEST, "Failed to toggle SPI mode pin");
        vTaskDelay(500/portTICK_PERIOD_MS);
    }

}

void TEST_GPIO()
{
    xTaskCreatePinnedToCore(Task_AFE_init, "Task_AFE_Init", 4000, NULL, configMAX_PRIORITIES-1, &Handle_Task_AFE_init, 1);
    xTaskCreatePinnedToCore(Task_TEST_GPIO, "Task_TEST_GPIO", 3000, NULL, 6, &Handle_Task_TEST_GPIO, 1);
}
void Task_TEST_CLKSRC()
{
    ESP_LOGI(TAG_AFE_TEST, "Configuring Clock source");
    AFE_config_clk_source();
    ESP_LOGI(TAG_AFE_TEST, "Clock source configured");
    for(;;)
    {
        vTaskDelay(300/portTICK_PERIOD_MS);
    }
}

void TEST_CLKSRC()
{
    xTaskCreatePinnedToCore(Task_AFE_init, "Task_AFE_Init", 4000, NULL, configMAX_PRIORITIES-1, &Handle_Task_AFE_init, 1);
    xTaskCreatePinnedToCore(Task_TEST_CLKSRC, "Task_TEST_CLKSRC", 3000, NULL, 6, &Handle_Task_TEST_CLKSRC, 1);
}

void TEST_GPtimer()
{
    AFE_Init_Sync_timer();
    return;
}

void TEST_AFE_subsystem()
{
        // Setting up test resources
        memset(&data_mock, 0x03, AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET); 
        // Setting up test resources
        xTaskCreatePinnedToCore(Task_init_AFE_tasks, "AFE_init_tasks", 3000, NULL, configMAX_PRIORITIES-2, &Handle_Task_AFE_init_tasks, 1);
     return;   
}

void TEST_AFE_commands()
{
    // Setting up test resources
    memset(&data_mock, 0x03, AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET); 
    // Setting up test resources
    xTaskCreatePinnedToCore(Task_AFE_init, "AFE_init", 3000, NULL, configMAX_PRIORITIES-2, &Handle_Task_AFE_init_tasks, 1);

    for(;;)
    {
        
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

/* void TEST_TASK_HW_AFE_command_loop()
{
    uint8_t gpio_write = 0x01;
    esp_err_t result;

    AFE_Send_Command(spi_master[0], NO_RETRY, MASK_ADC_WRITE|ADDRESS_ADC_GPIO_CONTROL, 0x01);
    for(;;)
    {
        
        result = AFE_Send_Command(spi_master[0], RETRY, MASK_ADC_WRITE|ADDRESS_ADC_GPIO_WRITE, gpio_write);
        if(result != ESP_OK)
        {
            ESP_LOGE(TAG_AFE_TEST, "Failed to send data");
            return;
        }
        gpio_write++;
        gpio_write = gpio_write % 2;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

}

void TEST_HW_AFE_command_loop()
{
    // Setting up test resources
    memset(&data_mock, 0x03, AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET); 
    // Setting up test resources
    xTaskCreatePinnedToCore(Task_AFE_init, "AFE_init", 3000, NULL, configMAX_PRIORITIES-2, &Handle_Task_AFE_init_tasks, 1);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    xTaskCreatePinnedToCore(TEST_TASK_HW_AFE_command_loop, "HW_AFE_command_loop", 3000, NULL, PRIORITY_TASK_SEND_CMD, &Handle_Task_AFE_init_tasks, 1);
    return;
}
 */
void TEST_TASK_listen()
{
    esp_err_t result;
    spi_transaction_t transaction;
    uint16_t data_resp = 0;
    transaction.length = 16;
    transaction.rx_buffer = &data_resp;
    transaction.tx_buffer = NULL;
    transaction.rxlength = 16;
    esp_log_level_set("*", ESP_LOG_DEBUG);
    result = AFE_set_SPI_controll_mode(true);
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG_AFE_TEST, "Failed to set SPI controll mode");
        return;
    }
    vTaskDelay(10000/portTICK_PERIOD_MS);

    for(;;)
    {
        AFE_reset(false);
        vTaskDelay(10/portTICK_PERIOD_MS);
        //AFE_config();
        
        
        /** Toggling GPIO pins on ADC1 */
        AFE_Send_Command(spi_master[0], NO_RETRY, MASK_ADC_READ | ADDRESS_ADC_CHIP_STATUS, 0x00);
        AFE_command_get_response(spi_master[0], &transaction);
        ESP_LOGD(TAG_AFE_TEST, "Toggling pins on ADC1");
        AFE_Send_Command(spi_master[0], NO_RETRY, MASK_ADC_WRITE | 0x0F, 0x0f);
        AFE_command_get_response(spi_master[0], &transaction);
        vTaskDelay(50/portTICK_PERIOD_MS);

        AFE_Send_Command(spi_master[0], NO_RETRY, MASK_ADC_WRITE | 0x0F, 0x00);
        AFE_command_get_response(spi_master[0], &transaction);
        
        ESP_LOGD(TAG_AFE_TEST, "Toggling pins on ADC2");
        vTaskDelay(50/portTICK_PERIOD_MS);
        AFE_Send_Command(spi_master[1], NO_RETRY, MASK_ADC_WRITE | 0x0F, 0x0f);
        AFE_command_get_response(spi_master[1], &transaction);
        vTaskDelay(50/portTICK_PERIOD_MS);

        AFE_Send_Command(spi_master[1], NO_RETRY, MASK_ADC_WRITE | 0x0F, 0x00);
        AFE_command_get_response(spi_master[1], &transaction);

        //AFE_command_get_response(spi_master[0], &transaction);
        //ESP_LOGD(TAG_AFE_TEST, "Got response %x", data_resp);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

/**
 * @brief This function configures the esp32s3 preipherals and starts listening for the output of the connected ADC
 */
void TEST_listen()
{
    xTaskCreatePinnedToCore(Task_AFE_init, "Task_AFE_Init", 4000, NULL, configMAX_PRIORITIES-1, &Handle_Task_AFE_init, 1);
    xTaskCreatePinnedToCore(TEST_TASK_listen, "Task_listen", 3000,NULL, 6, &Handle_TEST_Task_listen, 1);

    return;
}