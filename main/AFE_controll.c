//this .c file is dedicated for tasks and code which will controll, access and store data from an external ADC daisy-chain
// Todo: create tasks for collecting all data from the daisy-chain
//      create task for generating conssitent sampling time pulse
//      create init function for initializing the whole daisy chain
// all init functions will be called in the main app, and all tasks will be assigned to core 1
// First the bus must be initialized then the devices
// TODO: Add a function that implements a reset by send a pulse to RESET_PIN

/*LIST OF ALL GPIO PINS USED FOR SPI COMMUNICATION AND PIN CONFIG
    SPI SLAVE 
        CS0     GPIO_7
        MISO    GPIO_4
        MOSI    GPIO_5
        CLK     GPIO_6

    SPI MASTER
        CS0     GPIO_10
        CS1     GPIO_9
        CS2     GPIO_14
        CS3     GPIO_8
        MISO    GPIO_11
        MOSI    GPIO_13
        CLK     GPIO_12

    FORMATX PINS
        FORMAT0_PIN     GPIO_17
        FORMAT1_PIN     GPIO_14
    
    XTAL PINS
        XTAL_P  GPIO_20
        XTAL_N  GPIO_21
    
    /PIN / SPI  GPIO_38

    RESET PIN   GPIO_18
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
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "AFE_controll.h"
#include "AFE_config.h"



#define PRIORITY_TASK_GET_DATA 5
#define PRIORITY_TASK_STAGE_DATA 5
#define PRIORITY_TASK_SEND_CMD 6
#define SIZE__SPI_SLAVE_QUEUE 300

// strucutre for data which will be recieved by spi slave
typedef struct
{
    uint8_t data[AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET];
} AFE_data_t;


//Task handles for production code
TaskHandle_t Handle_Task_AFE_init = NULL;
TaskHandle_t Handle_Task_Stage_data = NULL;
TaskHandle_t Handle_TASK_Get_data = NULL;

//Task Handles for TESTS
TaskHandle_t Handle_TEST_spi_loop = NULL;
TaskHandle_t Handle_Task_TEST_loopback_receiver = NULL;
TaskHandle_t Handle_Task_TEST_loopback_sender = NULL;
TaskHandle_t Handle_Task_TEST_GPIO = NULL;
TaskHandle_t Handle_Task_TEST_CLKSRC = NULL;
TaskHandle_t Handle_TEST_Task_gen_data = NULL;
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



QueueHandle_t queue_AFE_data = NULL;
extern QueueHandle_t queue_image;
SemaphoreHandle_t semaphore_generator = NULL;
SemaphoreHandle_t semaphore_spi = NULL;


static const char *TAG_AFE = "AFE";

static spi_slave_interface_config_t device_spi_slave;
static spi_device_interface_config_t device_spi_master[AFE_NUM_OF_ADC];
static spi_bus_config_t bus_spi_slave, bus_spi_master;
static spi_device_handle_t spi_master[AFE_NUM_OF_ADC];
static uint8_t CS_pins_master[4] = {SPI_MASTER_CS0, SPI_MASTER_CS1, SPI_MASTER_CS2, SPI_MASTER_CS3}; // These GPIO pins need to be manually picked from datasheet
static gptimer_handle_t gptimer = NULL;


// This function is intended to be called after every config command sent to ADC to read the response in case an error code appears
// The AFE Sends a response in the next frame after a command is sent. This function is intened to get that response.
// It uses the master spi device used for configuring the AFE and should be only after reset and in AFE_Send_Command function
uint16_t AFE_command_get_response(spi_device_handle_t spi_device, spi_transaction_t * transaction_get_response)
{
    //configuring response transaciton
    // this transaction is used for getting the response in the next frame after the command
    uint16_t response = 0;
    transaction_get_response->length = 16;
    transaction_get_response->rx_buffer = &response; 
    transaction_get_response->tx_buffer = NULL;
    transaction_get_response->rxlength = 16;
    spi_device_transmit(spi_device, transaction_get_response);
    ESP_LOGI(TAG_AFE, "Got response %x", response);
    return response;
}

// Function for sending commands to the AFE over a SPI device
// Returns a generic ESP_IDF result as a return error type 
// TODO This needs to implement critical secitons for transmits?
// This sends in a blocking way
esp_err_t AFE_Send_Command(spi_device_handle_t spi_device, retry will_retry, uint8_t address, uint8_t reg_value)
{
    spi_transaction_t transaction_command, transaction_response;
    //configuring command transaction
    // this transaction is used for transmitting the command
    transaction_command.length = 16;
    transaction_command.rx_buffer = NULL;
    uint8_t command[2] = {address, reg_value};
    transaction_command.tx_buffer = &command;
    transaction_command.rxlength = 0;
    transaction_command.flags = 0;
    //configuring response transaciton
    // this transaction is used for getting the response in the next frame after the command
    // This should be done in command_get_response, but is kept here to avoid unreadable code in 
    // retry pattern
    uint16_t response = 0;
    transaction_response.tx_buffer = NULL;
    transaction_response.rxlength = 16;
    transaction_response.rx_buffer = &response;
    transaction_response.flags = 0;
    esp_err_t result;
    uint8_t cmd_attempts_ADC = 0;

    ESP_LOGI(TAG_AFE, "Sending data 0x%x", (int)(command[0]<<8 | command[1]));

    // Implemeted a retry-pattern which is optional defined by passed function parameter
    do 
    {
        //Attempts sending the same command 3 times then quits AFE config 
        if (cmd_attempts_ADC >=3)
        {
            ESP_LOGE(TAG_AFE, "All command attempts failed, exiting routine");
            return ESP_FAIL;
        }
        cmd_attempts_ADC++;
        ESP_LOGI(TAG_AFE, "Transmitting command");
        
        result = spi_device_transmit(spi_device, &transaction_command);
        
        ESP_LOGI(TAG_AFE, "Command sent");

        //Checking to see if retrying failed commands is enabled
        if (will_retry == RETRY) AFE_command_get_response(spi_device, &transaction_response);
    }while((uint16_t)ADC_ERROR_CODE == (uint16_t)(response));

    if(ESP_OK != result) ESP_LOGE(TAG_AFE, "SPI command failed");

    return result;

}

// Set the FORMATx pins to configure the count of DOUT lines to be used for spi data
// for now this is locked out of using other DOUT modes, later a parameter will be added for configurability
// This function is hardcoded to using only 1 DOUT pin 
esp_err_t AFE_set_dout_format()
{
    esp_err_t result;
    result = gpio_set_level(FORMAT0_pin, 1);
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG_AFE, "Failed to set FORMAT0 pin");
        return result;
    }
    result = gpio_set_level(FORMAT1_pin, 1);
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG_AFE, "Failed to set FORMAT1 pin");
        return result;
    }

    return ESP_OK;

}

// Simple wrapper function for setting the control mode for the AFE
esp_err_t AFE_set_SPI_controll_mode(bool SPI_mode)
{
    return gpio_set_level(CONTROL_MODE_pin, SPI_mode);
}


// FUnction for sending reset signal to AFE
// param defines if a pulse to a pin or a SPI command should be used
// TODO: need to verify if the reset pin matches pinout and function
esp_err_t AFE_reset(bool use_spi)
{
    esp_err_t result;
    // This method assumes all of the ADC have a reset pin connected to the same GPIO pin
    if(use_spi == false)
    {
        result = gpio_set_level(RESET_pin, 0);
        if(result != ESP_OK)
            {
                ESP_LOGE(TAG_AFE, "Failed to toggle RESET pin");
                return result;
            }
        vTaskDelay(500);
        gpio_set_level(RESET_pin, 1);

    }else
    {
        // When reseting the AFE, each ADC needs to be reset by hand
        for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
        {
            // Retry MUST BE DISABLED because reset via spi requires two consecutive write commands, so no dummy sends are allowed
            result = AFE_Send_Command(spi_master[device], NO_RETRY, MASK_ADC_WRITE | ADDRESS_ADC_DATA_CONTROL, MASK_ADC_DC_SPI_RESET_SEQ1);
            if(result != ESP_OK)
            {
                ESP_LOGE(TAG_AFE, "Failed to send reset command");
                return result;
            }
            // Retry MUST BE DISABLED because reset via spi requires two consecutive write commands, so no dummy sends are allowed
            result = AFE_Send_Command(spi_master[device], NO_RETRY, MASK_ADC_WRITE | ADDRESS_ADC_DATA_CONTROL, MASK_ADC_DC_SPI_RESET_SEQ2);
            if(result != ESP_OK)
            {
                ESP_LOGE(TAG_AFE, "Failed to send reset command");
                return result;
            }
        }
    }
    return ESP_OK;
}

// This function gereates Sync pulse for the first ADC because it is the one which propagates the sync pulse to the rest of the chain
// The syn cpulse is generated by two consecutive writes to the reset bit in a register, so retrying must be dissabled
esp_err_t AFE_sync()
{
    esp_err_t result;
    result = AFE_Send_Command(spi_master[0], NO_RETRY, MASK_ADC_WRITE | ADDRESS_ADC_DATA_CONTROL, MASK_ADC_DC_SPI_SYNC_LOW);
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG_AFE, "Failed to send first sync state");
        return result;
    }

    result = AFE_Send_Command(spi_master[0], NO_RETRY, MASK_ADC_WRITE | ADDRESS_ADC_DATA_CONTROL, MASK_ADC_DC_SPI_SYNC_HIGH);
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG_AFE, "Failed to send second sync state");
        return result;
    }
    return ESP_OK;
}

// This function generates two sync pulses neded to sync the AFE daisy chain (contains 2 or more ADCs)
esp_err_t AFE_sync_chain()
{
    esp_err_t result;
    result = AFE_sync();
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG_AFE, "Failed to send first sync command");
        return result;
    }
    result = AFE_sync();
    if(result != ESP_OK)
    {
        ESP_LOGE(TAG_AFE, "Failed to send second sync command");
        return result;
    }
    return result;
}

// This TEST task is created when the testing GPTimer is used to generate data
// This TEST tasks takes a semapohore and sends mock data using the spi master
// This function assumes the spi master and slave are connected during testing 
void TEST_Task_generate_data_w_SPI()
{
    for(;;)
    {
        ESP_LOGI(TAG_AFE, "Taking semaphore to generate new data and send to SPI");
        xSemaphoreTake(semaphore_generator, portMAX_DELAY);
        ESP_LOGI(TAG_AFE, "Generator semaphore Taken");
       // ESP_LOGI(TAG_AFE, "Sending transaciton located on %p which has data on %p", &transaction_mock, &data_mock);
       /*  for (uint8_t i = 0; i < sizeof(AFE_data_t); i++)
        {
            ESP_LOGI(TAG_AFE, "Data to send is 0x%x", data_mock[i] ); 
        } */

        if (ESP_OK != spi_device_queue_trans(spi_master[0], &transaction_mock, 0))
        {
            ESP_LOGE(TAG_AFE, "Failed to queue data to spi master ");
        }else
        {
            ESP_LOGI(TAG_AFE, "sending new data via SPI");
        }
        vTaskDelay(1);
        
    }
}


// This is a function called in the GPTimer ISR and gives the semaphore so that the blocking task
// TEST_Task_generate_data_w_SPI() can generate one new mock data
void TEST_MOCK_AFE_create_data()
{
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //ESP_LOGI(TAG_AFE, "Giving back generator semaphore");
    if (pdTRUE != xSemaphoreGiveFromISR(semaphore_generator, &xHigherPriorityTaskWoken))
    {
        //ESP_LOGE(TAG_AFE, "Failed to give back semaphore");
        return;
    }
    //spi_device_transmit(spi_master[0], &transaction_mock);
    // a yield is needed is a higher priority task is awoken
    // which it is, because the semaphoreGive always triggers a high priority task to generate data
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    

    return;
}


// This function is register as the user ISR function when the GPTimer invokes an interrupt
// This MUSTN'T call any blocking operations, including logging outputs
void Timer_sync_alarm()
{
    // this code is commented out during testing without AFE hardware
    //AFE_sync_chain();
    //AFE_sync_chain();
    //ESP_LOGI(TAG_AFE, "Hello from timer alarm");
    //TEST_GPTimer_hello();
    TEST_MOCK_AFE_create_data();
    //TEST_data++;
}

// This function inits GPTimer at a frequency of 100us which triggers every 500us
esp_err_t AFE_Init_Sync_timer()
{
    ESP_LOGI(TAG_AFE, "Starting GPtimer init");
    gptimer_config_t gptimer_config;/* ={
        .clk_src= GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000,
        .intr_priority = 0 // this sets the interrupt priority to the lowest priorities
    }; */
    gptimer_config.clk_src = GPTIMER_CLK_SRC_APB;// giving the timer the max possible clock of 80MHz
    gptimer_config.direction = GPTIMER_COUNT_UP;
    gptimer_config.resolution_hz = 1000*10;
    gptimer_config.intr_priority = 0;
    gptimer_alarm_config_t gptimer_alarm; /* = {
        .alarm_count = 10000/2000,
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    }; */
    gptimer_alarm.alarm_count = 10000/TEST_GEN_ODR; // sets the value when the alarm triggers, a second will have 10000 ticks, and the needed mock ODR is 2kSPs (2000)
    gptimer_alarm.flags.auto_reload_on_alarm = true; // when alarm is triggered, count is reloaded
    gptimer_alarm.reload_count = 0; // count autoreloads to 0

    ESP_ERROR_CHECK(gptimer_new_timer(&gptimer_config, &gptimer));
    gptimer_event_callbacks_t callback; /* = {
        .on_alarm = Timer_sync_alarm
    }; */
    callback.on_alarm = (void*)Timer_sync_alarm;

    ESP_LOGI(TAG_AFE, "starting timer");
    // Generating semaphore which will be used for the task to generate mock data
    semaphore_generator = xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &callback, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &gptimer_alarm));
    ESP_ERROR_CHECK(gptimer_start(gptimer));


    // generating task which will send new data via SPI
    xTaskCreatePinnedToCore(TEST_Task_generate_data_w_SPI, "Generate_data_w_spi", 3000, NULL, PRIORITY_TASK_STAGE_DATA +5, &Handle_TEST_Task_gen_data, 1);
    ESP_LOGI(TAG_AFE, "GPTimer and task configured");

    return ESP_OK;
}

// This function configures the pins used during SPI control mode
// the definitions for the pins are located in AFE_config.h
esp_err_t AFE_Config_GPIO_control()
{
    gpio_config_t pins_output;
    pins_output.intr_type = GPIO_INTR_DISABLE;
    pins_output.mode = GPIO_MODE_OUTPUT;
    pins_output.pin_bit_mask = (1ULL<<FORMAT0_pin) | (1ULL<<FORMAT1_pin) | (1ULL<<RESET_pin);
    pins_output.pull_down_en = 0;
    pins_output.pull_up_en = 0;
    return gpio_config(&pins_output);
}

// This function initializes a pwm controller for LED blinking and light intensity control.
// The controler is refurbished as a CLK source for the AFE
esp_err_t AFE_config_clk_source()
{
    ledc_timer_config_t AFE_clk_source = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,  // this parameter needs to adapt in order to allow high neough frequency
        .timer_num = LEDC_TIMER_0,
        .freq_hz =  AFE_MCLK,  // set to 32MHz
        .clk_cfg =  LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&AFE_clk_source));

    ledc_channel_config_t AFE_clk_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MCLK_pin,
        .duty = 1,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&AFE_clk_channel));

    return ESP_OK;
}

//This is the function where the ADC daisy-chain configuration should be done
// Configures the proposed way the AFE chain should be used
// TODO: Do i need to free spi buses between devices? maybe...
esp_err_t AFE_config()
{
    esp_err_t result;

    // each iteration of the loop configures one ADC in the daisy-chain
    for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
    {
        spi_transaction_t initial_response;
        
        ESP_LOGI(TAG_AFE, "Starting config of device %d", device);
        //setting channel on standby (NONE)
        //First response after reset is alays error code
        //if ((uint16_t)ADC_ERROR_CODE != AFE_command_get_response(spi_master[device], &initial_response)) ESP_LOGE(TAG_AFE, "ADC_Config: First response wasn't error code, maybe improper ADC restart");

        //Sending first command for channel stanby mode
        result = AFE_Send_Command(spi_master[device], RETRY, (uint8_t) MASK_ADC_WRITE|ADDRESS_ADC_CHANNEL_STANDBY, (uint8_t) MASK_ADC_CH_EN_ALL);
        if(result == ESP_FAIL) break;
        
        //Sending command for channel mode A
        result = AFE_Send_Command(spi_master[device], RETRY, (uint8_t) MASK_ADC_WRITE|ADDRESS_ADC_CHANNEL_MODE_A,(uint8_t) MASK_ADC_CMAR_SINC5|MASK_ADC_CMAR_DEC_32);
        if(result == ESP_FAIL) break;

        //Sending command for channel mode select
        result = AFE_Send_Command(spi_master[device], RETRY, (uint8_t) MASK_ADC_WRITE|ADDRESS_ADC_CHANNEL_MODE_SEL,(uint8_t) MASK_ADC_CMSR_ALL_A);
        if(result == ESP_FAIL) break;

        //Sending command for power mode
        result = AFE_Send_Command(spi_master[device], RETRY, (uint8_t)MASK_ADC_WRITE|ADDRESS_ADC_POWER_MODE,(uint8_t) MASK_ADC_PWR_MODE_LOW|MASK_ADC_LVDS_DIS|MASK_ADC_MCLK_DIV_32);
        if(result == ESP_FAIL) break;

        //Sending command for general config
        result = AFE_Send_Command(spi_master[device], RETRY, (uint8_t) MASK_ADC_WRITE|ADDRESS_ADC_CONFIG, (uint8_t)0x08);
        if(result == ESP_FAIL) break;

        //Sending command for interface config
        result = AFE_Send_Command(spi_master[device], RETRY, (uint8_t) MASK_ADC_WRITE|ADDRESS_ADC_INTERFACE_CONFIG,(uint8_t) 0x00|MASK_ADC_IC_DCLK_DIV_2);
        if(result == ESP_FAIL)
        {
            break;
        } else if(result == ESP_OK)
        {
            ESP_LOGI(TAG_AFE, "Device %d configured successfully", device);
        }

        AFE_reset(true);
        //RESET command needs to be issued
        // TODO: Do i need to free spi buses between devices? maybe...
    }

    if (ESP_OK == result) ESP_LOGI(TAG_AFE, "All devices were configured successfully");
    return result;
}

// This is a unused callback function
void slave_post_trans_cb(spi_slave_transaction_t *trans)
{
    // This is used to set custopm handshakes 
    //gpio_set_level()
}
// This is a unused callback function
void slave_post_setup_cb(spi_slave_transaction_t *trans)
{
    // This is used to set custopm handshakes 
    //gpio_set_level()
}

// This Task needs to initialize two spi buses. one must be a master with multiple /CS pins to configure multiple ADCs 
//The second bus is a slave which recieves data from the ADC daisy-chain
// This task must also handle AD configuration
// TODO: remember to initialize Xtal pins for source for ADC chip
void Task_AFE_init()
{
    for(;;)
    {
    //initializing GPIO PINS
    AFE_Config_GPIO_control();

    AFE_set_dout_format();
    AFE_set_SPI_controll_mode(true);

    //Initializing RTC for AFE CLK
    AFE_config_clk_source();

    

    //initializing  buses
    ESP_LOGI(TAG_AFE, "Starting SPI init");
    //Initializing slave bus ADC daisy-chain data
    bus_spi_slave.miso_io_num = SPI_SLAVE_MISO;
    bus_spi_slave.mosi_io_num = SPI_SLAVE_MOSI;
    bus_spi_slave.sclk_io_num = SPI_SLAVE_CLK;
    bus_spi_slave.isr_cpu_id = ESP_INTR_CPU_AFFINITY_1; // The spi slave triggers interrupts only for core 1, which handles AFE controll
    bus_spi_slave.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;

    ESP_LOGI(TAG_AFE, "Configured SPI slave bus");
    ESP_LOGI(TAG_AFE, "SPI slave can handle a length of %d bits", bus_spi_slave.max_transfer_sz * 8);
    //Initializing slave device for ADC daisy-chain data
    device_spi_slave.mode = 0;
    device_spi_slave.spics_io_num = SPI_SLAVE_CS0;
    device_spi_slave.queue_size = SIZE__SPI_SLAVE_QUEUE;
    device_spi_slave.flags = 0;
    device_spi_slave.post_setup_cb = slave_post_setup_cb;
    device_spi_slave.post_trans_cb = slave_post_trans_cb;

    esp_err_t ret;
    // Initializing SPI3_HOST as slave
    ret = spi_slave_initialize(SPI3_HOST, &bus_spi_slave, &device_spi_slave, SPI_DMA_DISABLED);
    if( ret != ESP_OK)
    {   
        ESP_LOGE(TAG_AFE, "Failed to initialize SPI slave on SPI3 /n");
        ESP_LOGE(TAG_AFE, "Error code: %d", ret);
    }
    //spi_bus_initialize(SPI3_HOST, &bus_spi_slave, SPI_DMA_DISABLED);
    //spi_bus_add_device(SPI3_HOST, &device_spi_slave, &spi_slave);
    ESP_LOGI(TAG_AFE, "Initialized SPI device");
    //assert( !(spi_slave == NULL));

    ESP_LOGI(TAG_AFE, "Configuring master bus");
    //initializins master bus for ADC config
    bus_spi_master.miso_io_num = SPI_MASTER_MISO;
    bus_spi_master.mosi_io_num = SPI_MASTER_MOSI;
    bus_spi_master.sclk_io_num = SPI_MASTER_CLK;
    bus_spi_master.max_transfer_sz = AFE_COMMAND_LEN;
    bus_spi_master.quadhd_io_num = -1;
    bus_spi_master.quadwp_io_num = -1;

    //Initializing SPI2_HOST bus as master 
    ret = spi_bus_initialize(SPI2_HOST, &bus_spi_master, SPI_DMA_DISABLED);
    assert(ret == ESP_OK);

    for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
    {
        ESP_LOGI(TAG_AFE, "Configuring master device %d", device);
        // The ADC operates in mode 0
        //Initializing master device for ADC daisy-chain config
        (device_spi_master[device]).clock_speed_hz = SPI_DATA_CLK;
        (device_spi_master[device]).mode = 0;
        (device_spi_master[device]).spics_io_num = CS_pins_master[device];
        (device_spi_master[device]).queue_size = 10;
        (device_spi_master[device]).duty_cycle_pos = 128;
        (device_spi_master[device]).cs_ena_posttrans = 3;
        (device_spi_master[device]).command_bits = 0;
        (device_spi_master[device]).address_bits = 0;
        (device_spi_master[device]).dummy_bits = 0;

        ret = spi_bus_add_device(SPI2_HOST, &device_spi_master[device], &spi_master[device]);
        assert(ret == ESP_OK);
        ESP_LOGI(TAG_AFE, "Master device %d configured", device);
    }
    //spi_bus_add_device(SPI2_HOST, &device_spi_master, &spi_master);
    //assert( !(spi_master[0] == NULL));

    // This delay is required in order to allow all of the settings for the spi slave to settle into the register
    //  I had to dicover this empyrically, because ofcourse the documentation doesn't say anything
    vTaskDelay(100/portTICK_PERIOD_MS);

    ret = AFE_config();
    if (ret != ESP_OK) ESP_LOGE(TAG_AFE, "AFE configuration failed");

    if (ESP_OK != AFE_Init_Sync_timer()) ESP_LOGE(TAG_AFE, "Failed to set sync timer for AFE");
    ESP_LOGI(TAG_AFE, "AFE initialization complete");
    vTaskDelete(NULL);
    }
    


}

// This task handles transaction creation andtransaction results
// This is seems like a very bad solution to creating a buch of transactions allocating it's data
// At this point this looks like the only way not to burst the memory budget and balance out execution time
// If you haven't noticed, this code is tied to the tick rate of the OS
// the tick rate of the OS is 100Hz and i need to pump out 2 kSps and there is a paralel task runing
// So i need to create 20ms worth of transactions which is 200, but I'm adding an extra 100 just decrease the odds
// of just blowing through the max number of transactions
void Task_AFE_get_data()
{
    // 
    
    spi_slave_transaction_t transactions[300];
    uint16_t transaction = 0;
    spi_slave_transaction_t* transaction_result = NULL;
    AFE_data_t *recieved_data;
    bool is_data_valid = true;

    vTaskDelay(1000/portTICK_PERIOD_MS);
    // This portion of the code handles the creation of the slave transactions
    // This is done in advance in order to save time 
    ESP_LOGI(TAG_AFE, "Creating transactions of length %d bits", AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET*8);
    for(uint16_t i = 0; i < 300; i++)
    {
        transactions[i].tx_buffer = NULL;
        transactions[i].length = (unsigned int) AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET*8;
        transactions[i].trans_len = (unsigned int) AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET*8;
        transactions[i].rx_buffer = malloc(AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET);
    }
    
    
    // Allocating memory for a lot of transactions
    // TODO: add malloc call for each new recieved data to copy to queue
    for(;;)
    {
        ESP_LOGI(TAG_AFE, "Starting set of transaction queues");
        for (uint8_t i = 0; i < 200; i++)
        {
            
            
            // We will check if we can queue data
            if(ESP_OK != spi_slave_queue_trans(SPI3_HOST, (spi_slave_transaction_t*) &transactions[transaction], 0))
            {
                ESP_LOGE(TAG_AFE, "Failed to queue spi slave transaction");
                break;
            }
            transaction++;
            transaction = transaction % 300;
        }
        
        ESP_LOGI(TAG_AFE, "Starting set of transaction results");
        // this portion handles recieveing results and passing valid data to datat staging via queue
        for (uint8_t i = 0; i < 200; i++)
        {
            is_data_valid = true;
            // this error occurs when queue is empty so we can immediately skip to preparing more transactions
            if(ESP_ERR_TIMEOUT ==  spi_slave_get_trans_result(SPI3_HOST, &transaction_result, 0))
            {
                ESP_LOGE(TAG_AFE, "Failed to get spi slave transaction result");
                break; 
            }
            recieved_data = (AFE_data_t*)(transaction_result->rx_buffer);
            
            // Checking  the header only for the first channel of an ADC, to verify if each ADC operates correctly
            for(uint8_t header = 0; header < AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET; header += AFE_NUM_OF_ADC_CH)
            {
                if ((0xE0 & recieved_data->data[header]) != 0){
                    ESP_LOGE(TAG_AFE, "Data packet has an error flag raised, has unsettled filter or repeated data, dropping whole image data");
                    is_data_valid = false;
                    break;
                }
            }
            // the malloc call needs to happen somewhere here because it would be nice if i could malloc after i decide if I even want to keep the data
            ESP_LOGI(TAG_AFE, "Sending data to queue");
            AFE_data_t *data = malloc(sizeof(AFE_data_t));
            memcpy(data, recieved_data, sizeof(AFE_data_t));
            //transactions[transaction].rx_buffer = malloc(AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET);
            ESP_LOGI(TAG_AFE, "Sending data to queue");
            if(is_data_valid == true)
            {
                if (pdFALSE == xQueueSend(queue_AFE_data, &data, 0))
                {
                    ESP_LOGE(TAG_AFE, "Error in sending data to trasaction queue. items in queue: %d \n Data is being missed", uxQueueMessagesWaiting(queue_AFE_data));
                    
                }
            }
             

        }
        vTaskDelay(2);
    }


}

// This task recieves unprocessed AFE data and removes the channel headers from the data to get a more compresed data format
void Task_AFE_stage_data()
{
    AFE_data_t *unparsed_data;
    image_data_raw_t image_filtered;

    vTaskDelay(1000/portTICK_PERIOD_MS);
    for(;;)
    {
        ESP_LOGI(TAG_AFE, "Getting unparsed data");
        xQueueReceive(queue_AFE_data, &unparsed_data ,portMAX_DELAY);
        uint16_t image_point = 0;
        image_filtered.len = AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*(AFE_SIZE_DATA_PACKET-1);
        // The format od the unparsed data is 1 byte header and 2 bytes data
        for(uint16_t byte = 1; byte < AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET; byte+=3)
        {
            image_filtered.data[image_point] = unparsed_data->data[byte] << 8 | unparsed_data->data[byte+1];
            image_point++;
        }
        free(unparsed_data);
        image_data_raw_t *image_stage = malloc(sizeof(image_data_raw_t));
        ESP_LOGI(TAG_AFE, "Staged new data on memory locaiton %p", image_stage);
        memcpy(image_stage, &image_filtered, sizeof(image_data_raw_t));

        xQueueSend(queue_image, &image_stage, portMAX_DELAY);
    }

}

void Task_init_AFE_tasks()
{
    // Initializing queue for AFE data
    for(;;){
        // Setting up test resources
        memset(&data_mock, 0x03, AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH*AFE_SIZE_DATA_PACKET); 
        // Setting up test resources

        uint8_t created_tasks = 0;
        // Verifying if the image queue resource was generated by the first core, if not waiting and retrying
        if (queue_image == NULL)
        {
            ESP_LOGE(TAG_AFE, "Image queue wasn't initialized yet, waiting a bit for 20ms for it to initialize then retrying");
            vTaskDelay(20/portTICK_PERIOD_MS);
            continue;
        }
        // initializing queue for transfer of trasactions between get_data and staging tasks
        queue_AFE_data = xQueueCreate(300, sizeof(AFE_data_t));
        if (queue_AFE_data == NULL)
        {
            ESP_LOGE(TAG_AFE, "Failed to create AFE_data_t queue, retrying");
            continue;
        }
        

        if ( pdPASS == xTaskCreatePinnedToCore(Task_AFE_init, "Task_AFE_init", 4000, NULL, configMAX_PRIORITIES-2, &Handle_Task_AFE_init, 1)) created_tasks++;
        if ( pdPASS == xTaskCreatePinnedToCore(Task_AFE_get_data, "Task_AFE_get_data", 10000, NULL, PRIORITY_TASK_GET_DATA, &Handle_TASK_Get_data, 1)) created_tasks++;
        if ( pdPASS == xTaskCreatePinnedToCore(Task_AFE_stage_data, "Task_AFE_stage_data", 4000, NULL, PRIORITY_TASK_STAGE_DATA, &Handle_Task_Stage_data, 1)) created_tasks++;
        if (created_tasks != 3)
        {
            ESP_LOGE(TAG_AFE, "All tasks were NOT created successfully. Number of created tasks %d", created_tasks);
            ESP_LOGE(TAG_AFE, "Deleting created tasks");
            if (Handle_Task_AFE_init != NULL) vTaskDelete(Handle_Task_AFE_init);
            if (Handle_Task_Stage_data != NULL) vTaskDelete(Handle_Task_Stage_data);
            if (Handle_TASK_Get_data != NULL) vTaskDelete(Handle_TASK_Get_data);;
            if (queue_AFE_data != NULL) vQueueDelete(queue_AFE_data);
            ESP_LOGE(TAG_AFE, "Deleted all tasks and resources and retrying");
            continue; // placing continue here to retry creating all of the tasks
        }

        vTaskDelete(NULL);
    }

}


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
            ESP_LOGI(TAG_AFE, "Sending data via spi on master device");
            spi_device_transmit(spi_master[0], &transaction_mock);
            vTaskDelay(500/portTICK_PERIOD_MS);
            data[1]++;
        }else
        {
            ESP_LOGI(TAG_AFE, "Sending command via command API");
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
    uint16_t recieved_data = 0;
    data_spi3.length = sizeof(AFE_data_t)*8;
    data_spi3.rx_buffer = &data;
    data_spi3.tx_buffer = NULL;
    spi_slave_transaction_t * spi_result = NULL;
    //A delay needs to be added for the slave configuration
    vTaskDelay(50/portTICK_PERIOD_MS);
    for(;;)
    {
        /* ESP_LOGI(TAG_AFE, "Recieving data via spi on slave device");
        spi_slave_transmit(SPI3_HOST, &data_spi3, portMAX_DELAY);        
        ESP_LOGI(TAG_AFE, "Recieved data is 0x%x", recieved_data); 
        vTaskDelay(1); */
        ESP_LOGI(TAG_AFE, "Recieving data via spi on slave device");
        spi_slave_queue_trans(SPI3_HOST, &data_spi3, portMAX_DELAY);

        
        spi_slave_get_trans_result(SPI3_HOST, &spi_result, portMAX_DELAY);
        ESP_LOGI(TAG_AFE, "Recieved new packet");
        for (uint8_t i = 0; i < sizeof(AFE_data_t)/2; i++)
        {
            ESP_LOGI(TAG_AFE, "Recieved data is 0x%x", *((uint16_t *)(spi_result->rx_buffer) +1) ); 
        }
        
        vTaskDelay(100);
    }
}
void TEST_spi_loopback()
{
    
    
    xTaskCreatePinnedToCore(Task_TEST_loopback_receiver, "Receiver", 3000, NULL, 3, &Handle_Task_TEST_loopback_receiver, 0);
    //xTaskCreatePinnedToCore(Task_TEST_loopback_sender, "Sender", 3000, NULL, 3, &Handle_Task_TEST_loopback_sender, 1);
    for(;;)
    {
        vTaskDelete(NULL);    
        //ESP_LOGI(TAG_AFE, "Waiting a bit before sending");
        //vTaskDelay(500/portTICK_PERIOD_MS);
        //vTaskDelay(1);
        //data++;
    }
    

}

void TEST_SPI()
{
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
        ESP_LOGI(TAG_AFE, "Changing controll mode %d", mode);
        if (ESP_OK != AFE_set_SPI_controll_mode(mode)) ESP_LOGE(TAG_AFE, "Failed to toggle SPI mode pin");
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
    ESP_LOGI(TAG_AFE, "Configuring Clock source");
    AFE_config_clk_source();
    ESP_LOGI(TAG_AFE, "Clock source configured");
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