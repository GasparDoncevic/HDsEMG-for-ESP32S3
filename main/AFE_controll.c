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

static spi_device_interface_config_t device_spi_slave;
static spi_device_interface_config_t device_spi_master[AFE_NUM_OF_ADC];
static spi_bus_config_t bus_spi_slave, bus_spi_master;
static spi_device_handle_t spi_slave, spi_master;


//This is the function where the ADC daisy-chain configuration should be done
void AFE_config()
{
    // each iteration of the loop configures one ADC in the daisy-chain
    for(uint8_t device = 0; i < AFE_NUM_OF_ADC; device++)
    {
        spi_device_transmit()
    }

}

// This Task needs to initialize two spi buses. one must be a master with multiple /CS pins to configure multiple ADCs 
//The second bus is a slave which recieves data from the ADC daisy-chain
// This task must also handle AD configuration
// TODO separate code for slave spi bus and master spi bus (master is for ADC config and uses multiple CS pins, so one loop is apropriate)
void Task_AFE_init()
{
    for(;;)
    {
        //initializing  buses
    //initializins master bus for ADC config
    bus_spi_master.miso_io_num = GPIO_NUM_11;
    bus_spi_master.mosi_io_num = GPIO_NUM_13;
    bus_spi_master.sclk_io_num = GPIO_NUM_12;
    bus_spi_master.max_transfer_sz = AFE_COMMAND_LEN;
    //Initializing slave bus ADC daisy-chain data
    bus_spi_slave.miso_io_num = ;
    bus_spi_slave.mosi_io_num = ;
    bus_spi_slave.sclk_io_num = ;
    bus_spi_slave.max_transfer_sz = (AFE_NUM_OF_ADC*ADC_CHANNEL_NUM);

    //Initializing slave device for ADC daisy-chain data
    device_spi_slave.clock_speed_hz = 1000000; // clockspeed must match the configured DCLK od ADCs
    device_spi_slave.mode = 0;
    device_spi_slave.spics_io_num;
    device_spi_slave.queue_size = 10;

    //Initializing master device for ADC daisy-chain config
    device_spi_master->clock_speed_hz = 1000000;
    device_spi_master->mode = 0;
    device_spi_master->spics_io_num = GPIO_NUM_10;
    device_spi_master->queue_size = 10;

    //Initializing SPI2_HOST as master 
    spi_bus_initialize(SPI2_HOST, &bus_spi_master, 1);
    spi_bus_add_device(SPI2_HOST, &device_spi_master, &spi_master);

    // Initializing SPI3_HOST as slave
    spi_bus_initialize(SPI3_HOST, &bus_spi_slave, 1);
    spi_bus_add_device(SPI3_HOST, &device_spi_slave, &spi_slave);

    assert( !(spi_slave == NULL));
    assert( !(spi_master == NULL));

    AFE_config();

    vTaskDelete(NULL);
    }
    


}

void Task_init_AFE_tasks()
{

}

void AFE_controll_send_command()
{

}