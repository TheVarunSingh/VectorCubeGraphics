// https://www.tutorialspoint.com/computer_graphics/3d_transformation.htm

#include "stm32f4xx.h"
#include "clock.h"
#include "gpio.h"
#include "cubetransformations.h"
#include "timers.h"
#include "spi.h"
#include "vec_gen.h"

#define LED_PIN GPIO_PA5

#define DELAY_TIM       TIM3
#define VEC_MASTER_CLK  TIM4
#define VEC_TIMER       TIM5

const unsigned int CUBE_VECTOR_DATA_SIZE = 38; // words
float cubeVectorData1[19][2];
float cubeVectorData2[19][2];

volatile int newCommandAvailable = 0;
volatile uint16_t lastCommand;
volatile int currentData = 1;

void USART2_IRQHandler() {
    if (USART2->SR | USART_SR_RXNE) {
        uint16_t message = USART2->DR;
        lastCommand = message;
        newCommandAvailable = 1;
    }
}

void configureDelayTimer() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    configureTimer(TIM3);
    NVIC_EnableIRQ(TIM3_IRQn);
}

void configureUSART2() {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    USART2->CR1 |= (USART_CR1_UE); // Enable USART
    USART2->CR1 &= ~(USART_CR1_M); // M=0 corresponds to 8 data bits
    USART2->CR2 &= ~(USART_CR2_STOP); // 0b00 corresponds to 1 stop bit
    USART2->CR1 &= ~(USART_CR1_OVER8); // Set to 16 times sampling freq

    USART2->BRR |= (45 << USART_BRR_DIV_Mantissa_Pos);
    USART2->BRR |= (0b1001 << USART_BRR_DIV_Fraction_Pos); // 9/16

    USART2->CR1 |= (USART_CR1_RXNEIE); // Enable the receive interrupt
    USART2->CR1 |= (USART_CR1_RE); // Enable the receiver

    // USART2 GPIOA pins
    alternateFunctionMode(GPIOA, GPIO_PA2, 7);
    alternateFunctionMode(GPIOA, GPIO_PA3, 7);

    NVIC_EnableIRQ(USART2_IRQn);
}

void configureGPIOs() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    alternateFunctionMode(GPIOB, GPIO_PB6, 2); // VEC_CLK (white wire) alt func VEC_CLK_CH1

    alternateFunctionMode(GPIOA, GPIO_PA0, 2); // GOb (green wire) alt func VEC_TIMER_CH1

    pinMode(GPIOA, GPIO_PA4, GPIO_OUTPUT); // COUNT_LD (blue wire)
    pinMode(GPIOA, GPIO_PA8, GPIO_OUTPUT); // COLOR_LD (yellow wire)

    alternateFunctionMode(GPIOA, GPIO_PA5, 5);  // X_SHIFT_REG_CLK  (orange wire) alt func SPI1_SCK
    pinMode(GPIOA, GPIO_PA6, GPIO_OUTPUT);      // X_SHIFT_REG_LD   (red wire)
    alternateFunctionMode(GPIOA, GPIO_PA7, 5);  // X_SHIFT_REG_DATA (brown wire)  alt func SPI1_MOSI

    alternateFunctionMode(GPIOC, GPIO_PC10, 6); // Y_SHIFT_REG_CLK  (orange wire) alt func SPI3_SCK
    pinMode(GPIOC, GPIO_PC11, GPIO_OUTPUT);     // Y_SHIFT_REG_LD   (red wire)
    alternateFunctionMode(GPIOC, GPIO_PC12, 6); // Y_SHIFT_REG_DATA (brown wire)  alt func SPI3_MOSI

    pinMode(GPIOA, LED_PIN, GPIO_OUTPUT); // LED on the Nucleo
}

void configureBRM() {
    /* Configure BRM */

    // Enable clock to timers
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    // Configure VEC_CLK Timer for outputting 1.5MHz clock
    configureCaptureCompare(VEC_MASTER_CLK);
    configurePWM(VEC_MASTER_CLK, 1);
    generatePWMfreq(VEC_MASTER_CLK, 10000000U, 2U);

    // Configure VEC_TIMER Timer for controlling GO signal based on VEC_CLK
    configureCaptureCompare(VEC_TIMER);
    configureDuration(VEC_TIMER, 0, 1, 0b010);
    VEC_TIMER->DIER |= TIM_DIER_UIE; // enable interrupt req upon updating
}

void configureDMA() {
    // TODO
}

#define NEG (1<<10)
void drawDiamond() {
    // sets SR.UIF so we can make it through the loop on the first run
    generateDuration(VEC_TIMER, 1, 2);
    delay_micros(DELAY_TIM,1);

    // load starting position
    loadCounter(512,512);
    delay_micros(DELAY_TIM,1);

    // double triangle shape thing data
    uint16_t array[6][2] = {{512,0},{0,512},{NEG|512,NEG|512},{NEG|512,0},{0,NEG|512},{512,512}};
    uint8_t i = 1; // I have no actually no idea which vector it wants to draw first, but ah well this works.
    while (1) {
        // start getting next vector
        i+=1;
        if (i==6) i=0;
        // send new values over SPI
        // (notice how we can accomplish this while VEC_TIMER is busy drawing a vector)
        doubleSendSPI(X_SPI, Y_SPI, array[i][0], array[i][1]);
        // wait until we are done drawing
        while(!(VEC_TIMER->SR & TIM_SR_UIF));

        // strobe shift register output latch
        //   (aw cheese whiz this is one thing we certainly can't expect a DMA contraption to handle)
        digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_HIGH);
        digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_HIGH);
        digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
        digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);

        // generate GO signal
        VEC_TIMER->SR &= ~(TIM_SR_UIF); // software has to clear this flag
        genDuration(VEC_TIMER, 1028, 5);
    }
}

int main(void) {
    configureFlash();
    configure84MHzClock();

    configureUSART2();
    configureDelayTimer();
    configureBRM();

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    configureSPI(SPI1);
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    configureSPI(SPI3);

    configureGPIOs();

    __enable_irq(); // Enable interrupts globally

    generateDuration(VEC_TIMER, 1, 2);
    loadColor(0, 0, 0b000, 0b011, 0b10);

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