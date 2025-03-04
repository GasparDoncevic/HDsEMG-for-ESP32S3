#ifndef AFE_CONTROLL_H
#define AFE_CONTROLL_H
//USER DEFINES
#define AFE_NUM_OF_ADC 1
#define AFE_NUM_OF_ADC_CH 4
#define AFE_SIZE_DATA_PACKET 3
#define AFE_COMMAND_LEN 16
#define ADC_CHANNEL_NUM 8
#define AFE_MAX_NUM_ADC 6

#define PRIORITY_TASK_GET_DATA 5
#define PRIORITY_TASK_STAGE_DATA 6
#define PRIORITY_TASK_SEND_CMD 7

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"

//USER DEFINES
typedef struct
{
    uint8_t len;
    uint16_t data[AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH];
} PACKED_ATTR image_data_raw_t;


typedef enum
{
    RETRY,
    NO_RETRY,
} retry;




//FUNCTIONS
esp_err_t AFE_config();
void Task_AFE_init();
void Task_init_AFE_tasks();
esp_err_t AFE_Send_Command(spi_device_handle_t spi_device, retry will_retry, uint8_t address, uint8_t reg_value);
uint16_t AFE_command_get_response(spi_device_handle_t spi_device);
esp_err_t AFE_set_dout_format();
esp_err_t AFE_set_SPI_controll_mode(bool SPI_mode);
esp_err_t AFE_reset(bool use_spi);
esp_err_t AFE_config_clk_source();
esp_err_t AFE_Init_Sync_timer();
esp_err_t AFE_sync_chain();
void reverseN_bytes(uint8_t *data, uint32_t len);

//void AFE_controll_send_command();
// void TEST_SPI();
// void TEST_GPIO();
// void TEST_CLKSRC();
// void TEST_GPtimer();

#endif