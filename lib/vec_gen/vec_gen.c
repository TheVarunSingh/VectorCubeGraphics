/**
    @file vec_gen.h
    @author Ben Bracker
    @version 1.0
*/
#include "vec_gen.h"

void loadCounter(uint16_t x_value, uint16_t y_value) {
    // opaquify shift register outputs
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);
    // send new value over SPI
    sendReceiveSPI(X_SPI, x_value);
    sendReceiveSPI(Y_SPI, y_value);

    // strobe shift register output latch
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);
    // strobe counter load
    digitalWrite(GPIOA, COUNT_LDb, GPIO_HIGH);
    digitalWrite(GPIOA, COUNT_LDb, GPIO_LOW);
    digitalWrite(GPIOA, COUNT_LDb, GPIO_HIGH);
}

void loadColor(uint16_t x_value, uint16_t y_value, unsigned int red, unsigned int green, unsigned int blue) {
    // opaquify shift register outputs
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);
    // send new value over SPI
    sendReceiveSPI(X_SPI, (green<<14) | (blue<<12) | x_value);
    sendReceiveSPI(Y_SPI, (red<<13) | ((green>>2)<<12) | y_value);

    // strobe shift register output latch
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);

    // strobe counter load and color load
    digitalWrite(GPIOA, COUNT_LDb, GPIO_HIGH);
    digitalWrite(GPIOA, COLOR_LD, GPIO_HIGH);
    digitalWrite(GPIOA, COUNT_LDb, GPIO_LOW);
    digitalWrite(GPIOA, COLOR_LD, GPIO_LOW);
    digitalWrite(GPIOA, COUNT_LDb, GPIO_HIGH);
}