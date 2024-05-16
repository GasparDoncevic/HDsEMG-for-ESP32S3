//this .c file is dedicated for tasks and code which will controll, access and store data from an external ADC daisy-chain
// Todo: create tasks for collecting all data from the daisy-chain
//      create task for generating conssitent sampling time pulse
//      create init function for initializing the whole daisy chain
// all init functions will be called in the main app, and all tasks will be assigned to core 1

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


void Task_AFE_init()
{

}

void Task_init_AFE_tasks()
{

}

void AFE_controll_send_command()
{

}