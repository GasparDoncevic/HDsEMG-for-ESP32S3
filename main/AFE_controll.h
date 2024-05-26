//this .h file is dedicated for all of the data structures that will hold image data 


//USER DEFINES
#define AFE_NUM_OF_ADC 1
#define AFE_COMMAND_LEN 16
#define ADC_CHANNEL_NUM 8

//USER DEFINES




//FUNCTIONS
esp_err_t AFE_config();
void Task_AFE_init();
void Task_init_AFE_tasks();
//void AFE_controll_send_command();
void TEST_SPI();
void TEST_GPIO();
void TEST_CLKSRC();

