#ifndef __GPIO_H__
#define __GPIO_H__

#include "stm32f4xx.h"

// Logic Levels
#define GPIO_LOW    0
#define GPIO_HIGH   1

// Arbitrary GPIO functions for pinMode()
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#define GPIO_ALT    2
#define GPIO_ANALOG 3

// Pin definitions for every GPIO pin
#define GPIO_PA0    0
#define GPIO_PA1    1
#define GPIO_PA2    2
#define GPIO_PA3    3
#define GPIO_PA4    4
#define GPIO_PA5    5
#define GPIO_PA6    6
#define GPIO_PA7    7
#define GPIO_PA8    8
#define GPIO_PA9    9
#define GPIO_PA10   10
#define GPIO_PA11   11
#define GPIO_PA12   12
#define GPIO_PA13   13
#define GPIO_PA14   14
#define GPIO_PA15   15

#define GPIO_PB0    0
#define GPIO_PB1    1
#define GPIO_PB2    2
#define GPIO_PB3    3
#define GPIO_PB4    4
#define GPIO_PB5    5
#define GPIO_PB6    6
#define GPIO_PB7    7
#define GPIO_PB8    8
#define GPIO_PB9    9
#define GPIO_PB10   10
#define GPIO_PB11   11
#define GPIO_PB12   12
#define GPIO_PB13   13
#define GPIO_PB14   14
#define GPIO_PB15   15

#define GPIO_PC0    0
#define GPIO_PC1    1
#define GPIO_PC2    2
#define GPIO_PC3    3
#define GPIO_PC4    4
#define GPIO_PC5    5
#define GPIO_PC6    6
#define GPIO_PC7    7
#define GPIO_PC8    8
#define GPIO_PC9    9
#define GPIO_PC10   10
#define GPIO_PC11   11
#define GPIO_PC12   12
#define GPIO_PC13   13
#define GPIO_PC14   14
#define GPIO_PC15   15

void pinMode(GPIO_TypeDef * GPIOx, int pin, int function);
void alternateFunctionMode(GPIO_TypeDef * GPIOx, int pin, int alt_func);
int digitalRead(GPIO_TypeDef * GPIOx, int pin);
void digitalWrite(GPIO_TypeDef * GPIOx, int pin, int val);
void togglePin(GPIO_TypeDef * GPIOx, int pin);

#endif