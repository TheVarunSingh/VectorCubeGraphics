#include "gpio.h"

void pinMode(GPIO_TypeDef * GPIOx, int pin, int function) {
    switch(function) {
        case GPIO_INPUT:
            GPIOx->MODER &= ~(0b11 << 2*pin);
            break;
        case GPIO_OUTPUT:
            GPIOx->MODER |= (0b1 << 2*pin);
            GPIOx->MODER &= ~(0b1 << (2*pin+1));
            break;
        case GPIO_ALT:
            GPIOx->MODER &= ~(0b1 << 2*pin);
            GPIOx->MODER |= (0b1 << (2*pin+1));
            break;
        case GPIO_ANALOG:
            GPIOx->MODER |= (0b11 << 2*pin);
            break;
    }
    GPIOx->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11; // speedy
}

void alternateFunctionMode(GPIO_TypeDef * GPIOx, int pin, int alt_func) {
    if (pin < 8) {
        GPIOx->AFR[0] &= ~(0b1111 << 4*pin);
        GPIOx->AFR[0] |= (alt_func << 4*pin);
    } else {
        GPIOx->AFR[1] &= ~(0b1111 << 4*(pin-8));
        GPIOx->AFR[1] |= (alt_func << 4*(pin-8));
    }
    pinMode(GPIOx, pin, GPIO_ALT);
}

int digitalRead(GPIO_TypeDef * GPIOx, int pin) {
    return ((GPIOx->IDR) >> pin) & 1;
}

void digitalWrite(GPIO_TypeDef * GPIOx, int pin, int val) {
    if (val) (GPIOx->BSRR) |= (1 << pin);        // set
    else     (GPIOx->BSRR) |= (1 << pin << 16);  // reset
}

void togglePin(GPIO_TypeDef * GPIOx, int pin) {
    digitalWrite(GPIOx, pin, !digitalRead(GPIOx, pin));
}