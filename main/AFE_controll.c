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
#include "AFE_controll.h"
#include "AFE_config.h"


//ADC REGISTER ADDRESSES
#define ADDRESS_ADC_CHANNEL_STANDBY  0x0 //each channel has its' own bit
#define ADDRESS_ADC_CHANNEL_MODE_A 0x01 // determines filter type and decimation rate
#define ADDRESS_ADC_CHANNEL_MODE_B  0x02 // determines filter type and decimation rate
#define ADDRESS_ADC_CHANNEL_MODE_SEL  0x03 // Selects channel mode for each channel
#define ADDRESS_ADC_POWER_MODE 0x04 // Contains sleep_mode, power_mode, LVDS_enable, MCLK_DIV
#define ADDRESS_ADC_CONFIG  0x05 // contains CLK_QUAL_DIS, RETIME_EN, VCM_PD, VCM_VSEL
#define ADDRESS_ADC_DATA_CONTROL 0x06 // contains SPI_SYNC, SINGLE_SHOT_EN, SPI_RESET
#define ADDRESS_ADC_INTERFACE_CONFIG 0x07 // contains CRC_select, DCLK_DIV
#define ADDRESS_ADC_CHIP_STATUS 0x09 // CHIP_ERROR, NO_CLOCK_ERROR, RAM_BIST_PASS, RAM_BIST_RUNNING 
#define ADDRESS_ADC_GPIO_CONTROL 0x0E // Contains UGPIO_enable, GPIOE4_FILTER, GPIOE3_MODE3,GPIOE2_MODE2, GPIOE1_MODE1, GPIO0_MODE0
#define ADDRESS_ADC_GPIO_WRITE 0x0F  // Write states for the five GPIO pins
#define ADDRESS_ADC_GPIO_READ  0x10 // Reads state from the 5 GPIO pins 
//ADC REGISTER ADDRESSES

//ADC REGISTER MASKS, Datasheet of the AD7761 needs to be checked to verify which bit control which feature
#define MASK_ADC_READ 0x80   // Sets the R/W bit to read
#define MASK_ADC_WRITE 0x00  // Sets the R/W bit to write
#define MASK_ADC_FILTER_SINC5   0x8
#define MASK_ADC_FILTER_WIDEBAND 0x0
//ADC REGISTER MASKS

#define ADC_ERROR_CODE 0x0E00



TaskHandle_t Handle_Task_AFE_init = NULL;
TaskHandle_t Handle_TEST_spi_loop = NULL;
TaskHandle_t Handle_Task_TEST_loopback_receiver = NULL;
TaskHandle_t Handle_Task_TEST_loopback_sender = NULL;
TaskHandle_t Handle_Task_TEST_GPIO = NULL;

static const char *TAG_AFE = "AFE";

static spi_slave_interface_config_t device_spi_slave;
static spi_device_interface_config_t device_spi_master[AFE_NUM_OF_ADC];
static spi_bus_config_t bus_spi_slave, bus_spi_master;
static spi_device_handle_t spi_slave, spi_master[AFE_NUM_OF_ADC];
static uint8_t CS_pins_master[4] = {SPI_MASTER_CS0, SPI_MASTER_CS1, SPI_MASTER_CS2, SPI_MASTER_CS3}; // These GPIO pins need to be manually picked from datasheet

// for now this is locked out of using other DOUT modes, later a parameter will be added for configurability
esp_err_t AFE_set_dout_format()
{
    gpio_set_level(FORMAT0_pin, 0);
    gpio_set_level(FORMAT1_pin, 0);
}

esp_err_t AFE_set_SPI_controll_mode(bool SPI_mode)
{
    return gpio_set_level(CONTROL_MODE_pin, SPI_mode);
}

esp_err_t AFE_reset(bool use_spi)
{
    esp_err_t result;
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
        for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
        {
            result = AFE_Send_Command(spi_master[device], ADDRESS_ADC_DATA_CONTROL, 0x03);
            if(result != ESP_OK)
            {
                ESP_LOGE(TAG_AFE, "Failed to send reset command");
                return result;
            }

            result = AFE_Send_Command(spi_master[device], ADDRESS_ADC_DATA_CONTROL, 0x02);
            if(result != ESP_OK)
            {
                ESP_LOGE(TAG_AFE, "Failed to send reset command");
                return result;
            }
        }
    }
    return ESP_OK;
}

// This function is intended to be called after every config command sent to ADC to read the response in case an error code appears
uint8_t AFE_command_get_response(spi_device_handle_t spi_device, spi_transaction_t * transaction_get_response)
{
    uint8_t response = 0;
    transaction_get_response->length = 16;
    transaction_get_response->rx_buffer = &response;
    transaction_get_response->tx_buffer = NULL;
    spi_device_transmit(spi_device, transaction_get_response);
    return response;
}

esp_err_t AFE_Send_Command(spi_device_handle_t spi_device,  uint8_t address, uint8_t reg_value)
{
    spi_transaction_t transaction_command, trasnaction_response;
    //configuring command transaction
    transaction_command.length = 16;
    transaction_command.rx_buffer = NULL;
    uint8_t command[2] = {address, reg_value};
    transaction_command.tx_buffer = &command;
    //configuring response transaciton
    trasnaction_response.tx_buffer = NULL;
    trasnaction_response.length = 16;
    
    esp_err_t result;
    uint8_t cmd_attempts_ADC = 0;

    do 
    {
        //Attempts sending the same command 3 times then quits AFE config 
        if (cmd_attempts_ADC >=3)
        {
            ESP_LOGE(TAG_AFE, "All command attempts failed, exiting config routine");
            return ESP_FAIL;
        }
        cmd_attempts_ADC++;
        result = spi_device_transmit(spi_device, &transaction_command);
    }while((uint8_t)ADC_ERROR_CODE == AFE_command_get_response(spi_device, &trasnaction_response));

    if(ESP_OK != result) ESP_LOGE(TAG_AFE, "SPI config command failed");

    return result;

}

//This is the function where the ADC daisy-chain configuration should be done
esp_err_t AFE_config()
{
    esp_err_t result;

    // each iteration of the loop configures one ADC in the daisy-chain
    
    
    for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
    {
        spi_transaction_t initial_response;
        
        //setting channel on standby (NONE)
        //First response after reset is alays error code
        if (ADC_ERROR_CODE != AFE_command_get_response(spi_master[device], &initial_response)) ESP_LOGE(TAG_AFE, "ADC_Config: First response wasn't error code, maybe improper ADC restart");

        //Sending first command for channel stanby mode
        result = AFE_Send_Command(spi_master[device], MASK_ADC_WRITE|ADDRESS_ADC_CHANNEL_STANDBY, 0x00);
        if(result == ESP_FAIL) break;
        
        //Sending command for channel mode A
        result = AFE_Send_Command(spi_master[device], MASK_ADC_WRITE|ADDRESS_ADC_CHANNEL_MODE_A, 0x00);
        if(result == ESP_FAIL) break;

        //Sending command for channel mode select
        result = AFE_Send_Command(spi_master[device], MASK_ADC_WRITE|ADDRESS_ADC_CHANNEL_MODE_SEL, 0x00);
        if(result == ESP_FAIL) break;

        //Sending command for power mode
        result = AFE_Send_Command(spi_master[device], MASK_ADC_WRITE|ADDRESS_ADC_POWER_MODE, 0x00);
        if(result == ESP_FAIL) break;

        //Sending command for general config
        result = AFE_Send_Command(spi_master[device], MASK_ADC_WRITE|ADDRESS_ADC_CONFIG, 0x00);
        if(result == ESP_FAIL) break;

        //Sending command for interface config
        result = AFE_Send_Command(spi_master[device], MASK_ADC_WRITE|ADDRESS_ADC_INTERFACE_CONFIG, 0x00);
        if(result == ESP_FAIL)
        {
            break;
        } else if(result == ESP_OK)
        {
            ESP_LOGI(TAG_AFE, "Device %d configured successfully", device);
        }

        //RESET command needs to be issued
    }

    if (ESP_OK == result) ESP_LOGI(TAG_AFE, "All devices were configures successfully");
    return result;
}

void slave_post_trans_cb(spi_slave_transaction_t *trans)
{
    // This is used to set custopm handshakes 
    //gpio_set_level()
}

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
    gpio_config_t pins_output;
    pins_output.intr_type = GPIO_INTR_DISABLE;
    pins_output.mode = GPIO_MODE_OUTPUT;
    pins_output.pin_bit_mask = (1ULL<<FORMAT0_pin) | (1ULL<<FORMAT1_pin) | (1ULL<<RESET_pin);
    pins_output.pull_down_en = 0;
    pins_output.pull_up_en = 0;
    gpio_config(&pins_output);

    //initializing  buses
    ESP_LOGI(TAG_AFE, "Starting SPI init");
    //Initializing slave bus ADC daisy-chain data
    bus_spi_slave.miso_io_num = SPI_SLAVE_MISO;
    bus_spi_slave.mosi_io_num = SPI_SLAVE_MOSI;
    bus_spi_slave.sclk_io_num = SPI_SLAVE_CLK;
    //bus_spi_slave.max_transfer_sz = (AFE_NUM_OF_ADC*ADC_CHANNEL_NUM);

    ESP_LOGI(TAG_AFE, "Configured SPI slave bus");
    //Initializing slave device for ADC daisy-chain data
    device_spi_slave.mode = 0;
    device_spi_slave.spics_io_num = SPI_SLAVE_CS0;
    device_spi_slave.queue_size = 10;
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

    //initializins master bus for ADC config
    bus_spi_master.miso_io_num = SPI_MASTER_MISO;
    bus_spi_master.mosi_io_num = SPI_MASTER_MOSI;
    bus_spi_master.sclk_io_num = SPI_MASTER_CLK;
    //bus_spi_master.max_transfer_sz = AFE_COMMAND_LEN;
    bus_spi_master.quadhd_io_num = -1;
    bus_spi_master.quadwp_io_num = -1;

    //Initializing SPI2_HOST bus as master 
    ret = spi_bus_initialize(SPI2_HOST, &bus_spi_master, SPI_DMA_DISABLED);
    assert(ret == ESP_OK);

    for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
    {
        // The ADC operates in mode 0
        //Initializing master device for ADC daisy-chain config
        (device_spi_master[device]).clock_speed_hz = 5000000;
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
    }
    //spi_bus_add_device(SPI2_HOST, &device_spi_master, &spi_master);
    //assert( !(spi_master[0] == NULL));


    //ret = AFE_config();
    if (ret != ESP_OK) ESP_LOGE(TAG_AFE, "AFE configuration failed");

    vTaskDelete(NULL);
    }
    


}

void Task_init_AFE_tasks()
{

}


// Function intended for seeing if the spi pins really do output the correct data and if the signals are correctly routed
// spi3 is slave, spi2 is master
void Task_TEST_loopback_sender()
{   
    spi_transaction_t data_spi2;
    memset(&data_spi2, 0, sizeof(spi_transaction_t));
    uint8_t data = 0xA;
    data_spi2.length = 8;
    data_spi2.tx_buffer = &data;
    data_spi2.rx_buffer = NULL;
    for(;;)
    {
        ESP_LOGI(TAG_AFE, "Sending data via spi on master device");
        spi_device_transmit(spi_master[0], &data_spi2);
        vTaskDelay(500/portTICK_PERIOD_MS);
        data++;
    }
}
void Task_TEST_loopback_receiver()
{
    spi_slave_transaction_t data_spi3;
    memset(&data_spi3, 0, sizeof(spi_slave_transaction_t));
    uint8_t recieved_data = 0;
    data_spi3.length = 8;
    data_spi3.rx_buffer = &recieved_data;
    data_spi3.tx_buffer = NULL;
    for(;;)
    {
        ESP_LOGI(TAG_AFE, "Recieving data via spi on slave device");
        spi_slave_transmit(SPI3_HOST, &data_spi3, portMAX_DELAY);        
        ESP_LOGI(TAG_AFE, "Recieved data is %d", recieved_data); 
        vTaskDelay(1);
    }
}
void TEST_spi_loopback()
{
    
    
    xTaskCreatePinnedToCore(Task_TEST_loopback_receiver, "Receiver", 3000, NULL, 3, &Handle_Task_TEST_loopback_receiver, 0);
    xTaskCreatePinnedToCore(Task_TEST_loopback_sender, "Sender", 3000, NULL, 3, &Handle_Task_TEST_loopback_sender, 1);
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