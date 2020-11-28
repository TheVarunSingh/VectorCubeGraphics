// https://www.tutorialspoint.com/computer_graphics/3d_transformation.htm

#include "stm32f4xx.h"
#include "gpio.h"
#include "cubetransformations.h"

#define LED_PIN GPIO_PA5

const unsigned int CUBE_VECTOR_DATA_SIZE = 38; // words
float cubeVectorData1[19][2];
float cubeVectorData2[19][2];

volatile int newCommandAvailable = 0;
volatile uint16_t lastCommand;
volatile int currentData = 1;

void USART2_IRQHandler() {
    uint16_t message = USART2->DR;
    lastCommand = message;
    newCommandAvailable = 1;
}

void configureUSART2() {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    USART2->CR1 |= (USART_CR1_UE); // Enable USART
    USART2->CR1 &= ~(USART_CR1_M); // M=0 corresponds to 8 data bits
    USART2->CR2 &= ~(USART_CR2_STOP); // 0b00 corresponds to 1 stop bit
    USART2->CR1 &= ~(USART_CR1_OVER8); // Set to 16 times sampling freq

    USART2->BRR |= (8 << USART_BRR_DIV_Mantissa_Pos);
    USART2->BRR |= (11 << USART_BRR_DIV_Fraction_Pos); // 11/16

    USART2->CR1 |= (USART_CR1_RXNEIE); // Enable the receive interrupt
    USART2->CR1 |= (USART_CR1_RE); // Enable the receiver

    // USART2 GPIOA pins
    alternateFunctionMode(GPIOA, GPIO_PA2, 7);
    alternateFunctionMode(GPIOA, GPIO_PA3, 7);

    __enable_irq(); // Enable interrupts globally
    NVIC_EnableIRQ(USART2_IRQn);
}

void configureSPI() {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    pinMode(GPIOB, GPIO_PB1, GPIO_OUTPUT); // NSS
    alternateFunctionMode(GPIOB, GPIO_PB13, 5); // CLK
    alternateFunctionMode(GPIOB, GPIO_PB14, 5); // MISO
    alternateFunctionMode(GPIOB, GPIO_PB15, 5); // MOSI
    digitalWrite(GPIOB, GPIO_PB1, 1); // set NSS high

    SPI2->CR1 |= (0 << SPI_CR1_SPE_Pos); // disable the SPI interface

    SPI2->CR1 |= ((0b111 << SPI_CR1_BR_Pos) |
                  (0     << SPI_CR1_CPOL_Pos) |
                  (0     << SPI_CR1_CPHA_Pos) |
                  (1     << SPI_CR1_DFF_Pos) |
                  (0     << SPI_CR1_LSBFIRST_Pos) |
                            SPI_CR1_SSM |
                            SPI_CR1_SSI |
                            SPI_CR1_MSTR);

    SPI2->CR1 |= (SPI_CR1_SPE); // enable the SPI interface
}

void configureDMA() {

}

int main(void) {
    configureUSART2();
    pinMode(GPIOA, LED_PIN, GPIO_OUTPUT);

    while (1) {
        if (newCommandAvailable) {
            newCommandAvailable = 0;

            switch (lastCommand) {
                case ((uint16_t)'w'):
                    rotateYCube(1);
                    break;
                case ((uint16_t)'s'):
                    rotateYCube(-1);
                    break;
                case ((uint16_t)'a'):
                    rotateZCube(1);
                    break;
                case ((uint16_t)'d'):
                    rotateZCube(-1);
                    break;
                case ((uint16_t)'i'):
                    digitalWrite(GPIOA, LED_PIN, 1);
                    break;
                case ((uint16_t)'k'):
                    digitalWrite(GPIOA, LED_PIN, 0);
                    break;
                case ((uint16_t)'j'):
                    togglePin(GPIOA, LED_PIN);
                    break;
            }

            if (currentData == 1) {
                calculateCubeVectorData(cubeVectorData2);
                currentData = 2;
            } else {
                calculateCubeVectorData(cubeVectorData1);
                currentData = 1;
            }
        }
    }

    return 0;
}