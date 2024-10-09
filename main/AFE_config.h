//This file is inteded to contain all of the define macros for controlling the AFE
// This is done in order to maintain the abstraction level for the AFE_control module
// and make it adaptable to any new dvelopment board or microcontroler which wishes to use it
#ifndef AFE_CONFIG_H
#define AFE_DONFIG_H

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
#define MCLK_pin      GPIO_NUM_20
//#define XTAL_2_pin      GPIO_NUM_21


//TODO: Configuirations for the clock source for the AFE
#define SPI_DATA_CLK 16000000 // 16MHz
#define AFE_MCLK 32000000 // 32 MHz

//ADC register addresses, and pin masks

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

//ADC read/write masks
#define MASK_ADC_READ 0x80   // Sets the R/W bit to read
#define MASK_ADC_WRITE 0x00  // Sets the R/W bit to write


//Channel standby register, determines which channels will be inactive
#define MASK_ADC_CSR_CH7_DIS    0x80
#define MASK_ADC_CSR_CH6_DIS    0x40
#define MASK_ADC_CSR_CH5_DIS    0x20
#define MASK_ADC_CSR_CH4_DIS    0x10
#define MASK_ADC_CSR_CH3_DIS    0x08
#define MASK_ADC_CSR_CH2_DIS    0x04
#define MASK_ADC_CSR_CH1_DIS    0x02
#define MASK_ADC_CSR_CH0_DIS    0x01
#define MASK_ADC_CH_EN_ALL      0x00

// Channel mode A register
#define MASK_ADC_CMAR_SINC5     0x8
#define MASK_ADC_CMAR_WIDEBAND  0x0
#define MASK_ADC_CMAR_DEC_32    0x0
#define MASK_ADC_CMAR_DEC_64    0x1
#define MASK_ADC_CMAR_DEC_128   0x2
#define MASK_ADC_CMAR_DEC_256   0x3
#define MASK_ADC_CMAR_DEC_512   0x4
#define MASK_ADC_CMAR_DEC_1024  0x5

// Channel mode B register
#define MASK_ADC_CMBR_SINC5     0x8
#define MASK_ADC_CMBR_WIDEBAND  0x0
#define MASK_ADC_CMBR_DEC_32    0x0
#define MASK_ADC_CMBR_DEC_64    0x1
#define MASK_ADC_CMBR_DEC_128   0x2
#define MASK_ADC_CMBR_DEC_256   0x3
#define MASK_ADC_CMBR_DEC_512   0x4
#define MASK_ADC_CMBR_DEC_1024  0x5

// Channel mode select register, setting values to 1 sets to mode B and 0 sets to mode A
#define MASK_ADC_CMSR_B_CH0   0x01
#define MASK_ADC_CMSR_B_CH1   0x02
#define MASK_ADC_CMSR_B_CH2   0x04
#define MASK_ADC_CMSR_B_CH3   0x08
#define MASK_ADC_CMSR_B_CH4   0x10
#define MASK_ADC_CMSR_B_CH5   0x20
#define MASK_ADC_CMSR_B_CH6   0x40
#define MASK_ADC_CMSR_B_CH7   0x80
#define MASK_ADC_CMSR_ALL_B   0xFF
#define MASK_ADC_CMSR_ALL_A   0x00

// Power mode select register
#define MASK_ADC_SLEEP_MODE     0x80
#define MASK_ADC_NORMAL_MODE    0x00
#define MASK_ADC_PWR_MODE_LOW   0x00
#define MASK_ADC_PWR_MODE_MED   0x20
#define MASK_ADC_PWR_MODE_FAST  0x30
#define MASK_ADC_LVDS_EN        0x04
#define MASK_ADC_LVDS_DIS       0x00
#define MASK_ADC_MCLK_DIV_32    0x00
#define MASK_ADC_MCLK_DIV_8     0x02
#define MASK_ADC_MCLK_DIV_4     0x03

// General device configuration register
#define MASK_ADC_GDCR_CLK_QUAL_EN   0x00
#define MASK_ADC_GDCR_CLK_QUAL_DIS  0x40
#define MASK_ADC_GDCR_RETIME_DIS    0x00
#define MASK_ADC_GDCR_RETIME_EN    0x20
#define MASK_ADC_GDCR_VCM_BUF_EN    0x00
#define MASK_ADC_GDCR_VCM_BUF_DIS   0x10
#define MASK_ADC_GDCR_VCM_SEL_HALF  0x00
#define MASK_ADC_GDCR_VCM_SEL_1V65  0x00
#define MASK_ADC_GDCR_VCM_SEL_2V5   0x00
#define MASK_ADC_GDCR_VCM_SEL_2V14  0x00

// Data Control register
#define MASK_ADC_DC_SPI_SYNC_LOW    0x00
#define MASK_ADC_DC_SPI_SYNC_HIGH   0x80
#define MASK_ADC_DC_SINGLE_EN       0x10
#define MASK_ADC_DC_SPI_RESET_SEQ2  0x02
#define MASK_ADC_DC_SPI_RESET_SEQ1  0x03

// Interface configuration register
#define MASK_ADC_IC_CRC_SEL_DIS     0x00
#define MASK_ADC_IC_CRC_SEL_4       0x04
#define MASK_ADC_IC_CRC_SEL_16      0x08
#define MASK_ADC_IC_DCLK_DIV_8      0x00
#define MASK_ADC_IC_DCLK_DIV_4      0x01
#define MASK_ADC_IC_DCLK_DIV_2      0x02
#define MASK_ADC_IC_DCLK_DIV_1      0x03

#define ADC_ERROR_CODE 0x0E00

#define TEST_GEN_ODR 1

#endif