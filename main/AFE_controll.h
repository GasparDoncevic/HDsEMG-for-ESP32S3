//this .h file is dedicated for all of the data structures that will hold image data 
#ifndef AFE_CONTROLL_H
#define AFE_CONTROLL_H
//USER DEFINES
#define AFE_NUM_OF_ADC 1
#define AFE_NUM_OF_ADC_CH 8
#define AFE_SIZE_DATA_PACKET 3
#define AFE_COMMAND_LEN 16
#define ADC_CHANNEL_NUM 8

//USER DEFINES
typedef struct
{
    uint8_t len;
    uint16_t data[AFE_NUM_OF_ADC*AFE_NUM_OF_ADC_CH];
} image_data_raw_t;


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
void TEST_SPI();
void TEST_GPIO();
void TEST_CLKSRC();
void TEST_GPtimer();

#endif