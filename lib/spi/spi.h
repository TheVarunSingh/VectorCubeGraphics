#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx.h"

void configureSPI(SPI_TypeDef * SPIx);
void sendReceiveSPI(SPI_TypeDef * SPIx, uint16_t data);
void doubleSendSPI(SPI_TypeDef* SPIx, SPI_TypeDef* SPIy, uint16_t dataX, uint16_t dataY);


#endif