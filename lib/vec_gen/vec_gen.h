#ifndef __VEC_GEN_H__
#define __VEC_GEN_H__

#include "stm32f4xx.h"
#include "gpio.h"
#include "spi.h"

///////////////////////////////////////////////////////////////////////////////
// Port Definitions
//   Note: suffix of "b" stands for "bar", i.e. an active-low signal
///////////////////////////////////////////////////////////////////////////////
#define VEC_CLK           GPIO_PB6  // white wire  (port B)
#define GOb               GPIO_PA0  // green wire  (port A)
#define COUNT_LDb         GPIO_PA4  // blue wire   (port A)
#define COLOR_LD          GPIO_PA8  // yellow wire (port A)
#define X_SHIFT_REG_CLK   GPIO_PA5  // orange wire (port A)
#define X_SHIFT_REG_LD    GPIO_PA6  // red wire    (port A)
#define X_SHIFT_REG_DATA  GPIO_PA7  // brown wire  (port A)
#define Y_SHIFT_REG_CLK   GPIO_PC10 // orange wire (port C)
#define Y_SHIFT_REG_LD    GPIO_PC11 // red wire    (port C)
#define Y_SHIFT_REG_DATA  GPIO_PC12 // brown wire  (port C)

///////////////////////////////////////////////////////////////////////////////
// Peripheral Assignments
///////////////////////////////////////////////////////////////////////////////
#define X_SPI SPI1
#define Y_SPI SPI3

void loadCounter(uint16_t x_value, uint16_t y_value);

void loadColor(uint16_t x_value, uint16_t y_value, unsigned int red, unsigned int green, unsigned int blue);

#endif