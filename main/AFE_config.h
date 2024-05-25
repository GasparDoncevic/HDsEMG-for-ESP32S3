//This file is inteded to contain all of the define macros for controlling the AFE
// This is done in order to maintain the abstraction level for the AFE_control module
// and make it adaptable to any new dvelopment board or microcontroler which wishes to use it
#ifndef driver/gpio.h
#include "driver/gpio.h"
#endif

// SPI SLAVE PINS
#define SPI_SLAVE_CS0   GPIO_NUM_7
#define SPI_SLAVE_MISO  GPIO_NUM_4
#define SPI_SLAVE_MOSI  GPIO_NUM_5
#define SPI_SLAVE_CLK   GPIO_NUM_6
// SPI MASTER PINS
#define SPI_MASTER_CS0  GPIO_NUM_10
#define SPI_MASTER_CS1  GPIO_NUM_9
#define SPI_MASTER_CS2  GPIO_NUM_14
#define SPI_MASTER_CS3  GPIO_NUM_8
#define SPI_MASTER_MISO GPIO_NUM_11
#define SPI_MASTER_MOSI GPIO_NUM_13
#define SPI_MASTER_CLK  GPIO_NUM_12
// OTHER AFE FUNCTION PINS
#define FORMAT0_pin     GPIO_NUM_17
#define FORMAT1_pin     GPIO_NUM_14
#define RESET_pin       GPIO_NUM_18
#define CONTROL_MODE_pin    GPIO_NUM_38
#define XTAL_1_pin      GPIO_NUM_20
#define XTAL_2_pin      GPIO_NUM_21
