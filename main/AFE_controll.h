#ifndef AFE_CONTROLL_H
#define AFE_CONTROLL_H
//USER DEFINES
#define AFE_NUM_OF_ADC 1
#define AFE_NUM_OF_ADC_CH 1
#define AFE_SIZE_DATA_PACKET 3
#define AFE_COMMAND_LEN 16
#define ADC_CHANNEL_NUM 8
#define AFE_MAX_NUM_ADC 6

#define PRIORITY_TASK_GET_DATA 5
#define PRIORITY_TASK_STAGE_DATA 6
#define PRIORITY_TASK_SEND_CMD 7


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
//void AFE_controll_send_command();
// void TEST_SPI();
// void TEST_GPIO();
// void TEST_CLKSRC();
// void TEST_GPtimer();

#endif