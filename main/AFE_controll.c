//this .c file is dedicated for tasks and code which will controll, access and store data from an external ADC daisy-chain
// Todo: create tasks for collecting all data from the daisy-chain
//      create task for generating conssitent sampling time pulse
//      create init function for initializing the whole daisy chain
// all init functions will be called in the main app, and all tasks will be assigned to core 1
// First the bus must be initialized then the devices

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
#include "AFE_controll.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

//TODO: add defines for FORMATx pins and set them to 00 for using only one DOUT line

//USER DEFINES
#define AFE_NUM_OF_ADC 1
#define AFE_COMMAND_LEN 16
#define ADC_CHANNEL_NUM 8

//USER DEFINES

TaskHandle_t Handle_Task_AFE_init = NULL;
TaskHandle_t Handle_TEST_spi_loop = NULL;
TaskHandle_t Handle_Task_TEST_loopback_receiver = NULL;
TaskHandle_t Handle_Task_TEST_loopback_sender = NULL;

static const char *TAG_AFE = "AFE";

static spi_slave_interface_config_t device_spi_slave;
static spi_device_interface_config_t device_spi_master[AFE_NUM_OF_ADC];
static spi_bus_config_t bus_spi_slave, bus_spi_master;
static spi_device_handle_t spi_slave, spi_master[AFE_NUM_OF_ADC];
static uint8_t CS_pins_master[4] = {GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_14, GPIO_NUM_8}; // These GPIO pins need to be manually picked from datasheet

//ADC Commands, Datasheet of the AD7761 needs to be checked to verify which bit control which feature
/* static const uint8_t ADDRESS_ADC_CHANNEL_STANDBY = 0; //each channel has its' own bit
static const uint8_t ADDRESS_ADC_CHANNEL_MODE_A = 0x01; // determines filter type and decimation rate
static const uint8_t ADDRESS_ADC_CHANNEL_MODE_B = 0x02; // determines filter type and decimation rate
static const uint8_t ADDRESS_ADC_CHANNEL_MODE_SEL = 0x03; // Selects channel mode for each channel
static const uint8_t ADDRESS_ADC_POWER_MODE = 0x04; // Contains sleep_mode, power_mode, LVDS_enable, MCLK_DIV
static const uint8_t ADDRESS_ADC_CONFIG = 0x05; // contains CLK_QUAL_DIS, RETIME_EN, VCM_PD, VCM_VSEL
static const uint8_t ADDRESS_ADC_DATA_CONTROL = 0x06; // contains SPI_SYNC, SINGLE_SHOT_EN, SPI_RESET
static const uint8_t ADDRESS_ADC_INTERFACE_CONFIG = 0x07; // contains CRC_select, DCLK_DIV
static const uint8_t ADDRESS_ADC_CHIP_STATUS = 0x09; // CHIP_ERROR, NO_CLOCK_ERROR, RAM_BIST_PASS, RAM_BIST_RUNNING 
static const uint8_t ADDRESS_ADC_GPIO_CONTROL = 0x0E; // Contains UGPIO_enable, GPIOE4_FILTER, GPIOE3_MODE3,GPIOE2_MODE2, GPIOE1_MODE1, GPIO0_MODE0
static const uint8_t ADDRESS_ADC_GPIO_WRITE = 0x0F;  // Write states for the five GPIO pins
static const uint8_t ADDRESS_ADC_GPIO_READ = 0x10; // Reads state from the 5 GPIO pins */
//ADC Commands

//This is the function where the ADC daisy-chain configuration should be done
void AFE_config()
{
    // each iteration of the loop configures one ADC in the daisy-chain
    //each register is configured with two send commands, first  for register address, second for register value
    // then a read command needs to be issued to check for error code
    for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
    {
        
    }

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
        //initializing  buses
    ESP_LOGI(TAG_AFE, "Starting SPI init");
    //Initializing slave bus ADC daisy-chain data
    bus_spi_slave.miso_io_num = GPIO_NUM_4;
    bus_spi_slave.mosi_io_num = GPIO_NUM_5;
    bus_spi_slave.sclk_io_num = GPIO_NUM_6;
    //bus_spi_slave.max_transfer_sz = (AFE_NUM_OF_ADC*ADC_CHANNEL_NUM);

    ESP_LOGI(TAG_AFE, "Configured SPI slave bus");
    //Initializing slave device for ADC daisy-chain data
    device_spi_slave.mode = 0;
    device_spi_slave.spics_io_num = GPIO_NUM_7;
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
    bus_spi_master.miso_io_num = GPIO_NUM_11;
    bus_spi_master.mosi_io_num = GPIO_NUM_13;
    bus_spi_master.sclk_io_num = GPIO_NUM_12;
    //bus_spi_master.max_transfer_sz = AFE_COMMAND_LEN;
    bus_spi_master.quadhd_io_num = -1;
    bus_spi_master.quadwp_io_num = -1;

    //Initializing SPI2_HOST bus as master 
    ret = spi_bus_initialize(SPI2_HOST, &bus_spi_master, SPI_DMA_DISABLED);
    assert(ret == ESP_OK);

    for(uint8_t device = 0; device < AFE_NUM_OF_ADC; device++)
    {
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

    //AFE_config();

    vTaskDelete(NULL);
    }
    


}

void Task_init_AFE_tasks()
{

}

void AFE_controll_send_command()
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