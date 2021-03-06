// https://www.tutorialspoint.com/computer_graphics/3d_transformation.htm
// THIS IS A BORKED GIT MERGE; I don't how to make git not merge
// please do not use this version
#include "stm32f4xx.h"
#include "clock.h"
#include "gpio.h"
#include "cubetransformations.h"
#include "timers.h"
#include "spi.h"
#include "vec_gen.h"

#define LED_GPIO GPIOB
#define LED_PIN GPIO_PB10

#define DELAY_TIM       TIM3
#define VEC_MASTER_CLK  TIM4
#define VEC_TIMER       TIM5
#define X_DMA           DMA1
#define Y_DMA           DMA1
#define X_DMA_STREAM    DMA1_Stream1
#define Y_DMA_STREAM    DMA1_Stream2

const unsigned int CUBE_VECTOR_DATA_SIZE = 38; // words
#define ARRAY_SIZE 19
uint16_t cubeVectorData1[ARRAY_SIZE][2];
uint16_t cubeVectorData2[ARRAY_SIZE][2];

volatile int newCommandAvailable = 0;
volatile uint16_t lastCommand;
volatile int currentData = 1;

#define NEG (1 << 10)
//uint16_t array[ARRAY_SIZE][2] = {{512,0},{0,512},{NEG|512,NEG|512},{NEG|512,0},{0,NEG|512},{512,512}};
//uint16_t x_array[ARRAY_SIZE] = {512,0,NEG|512,NEG|512,0,512};
//uint16_t y_array[ARRAY_SIZE] = {0,512,NEG|512,0,NEG|512,512};
uint8_t i = 0; // I have no actually no idea which vector it wants to draw first, but ah well this works.

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

    pinMode(LED_GPIO, LED_PIN, GPIO_OUTPUT); // external debugging LED
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
    NVIC_EnableIRQ(TIM5_IRQn);
}

void configureDMA(DMA_Stream_TypeDef* dma_stream, uint16_t* source, uint16_t* destination) {
    // SxCR register:
    // - memory-to-memory
    // - don't increment memory ptr, don't increment periph ptr.
    // - 16-bit data size for both source and destination.
    // - High priority (2/3).

    // Disable stream
    dma_stream->CR &= ~DMA_SxCR_EN;
    // Wait until it is truly off
    while (dma_stream->CR & DMA_SxCR_EN_Msk);

    // Fully Reset stream configuration
    dma_stream->CR &= ~(DMA_SxCR_CHSEL |
                        DMA_SxCR_MBURST |
                        DMA_SxCR_PBURST |
                        DMA_SxCR_CT | // current target (double buffer mode)
                        DMA_SxCR_DBM | // double buffer mode
                        DMA_SxCR_PL | // priority level
                        DMA_SxCR_PINCOS | // peripheral increment offset size
                        DMA_SxCR_MSIZE |
                        DMA_SxCR_PSIZE |
                        DMA_SxCR_MINC | // memeory increment
                        DMA_SxCR_PINC | // peripheral increment
                        DMA_SxCR_CIRC | // circular mode
                        DMA_SxCR_DIR | // direction of transfer
                        DMA_SxCR_PFCTRL | // flow controller selection
                        DMA_SxCR_TCIE | // transfer complete interrupt enable
                        DMA_SxCR_HTIE | // half transfer interrupt enable
                        DMA_SxCR_TEIE | // transfer error interrupt enable
                        DMA_SxCR_DMEIE | // direct mode error interrupt enable
                        DMA_SxCR_EN);
    // Source
    dma_stream->M0AR = (uint32_t) source;
    // Destination
    dma_stream->PAR = (uint32_t) destination;
    // Data transfer length: we want to go 1 by 1.
    dma_stream->NDTR = (uint32_t) 1;
    // Configuration register
    dma_stream->CR |= ((0b01 << DMA_SxCR_MSIZE_Pos) | // 16 bit source
                       (0b01 << DMA_SxCR_PSIZE_Pos) | // 16 bit dest
                       (0b10 << DMA_SxCR_PL_Pos) | // high priority
                       (0b10 << DMA_SxCR_DIR_Pos)| // direction: memory-to-memory
                       DMA_SxCR_TCIE); // transfer complete interrupt enable
}

void runDMA(DMA_Stream_TypeDef* dma_stream, uint16_t* source) {
    // Disable stream
    dma_stream->CR &= ~DMA_SxCR_EN;
    // Wait until it is truly off
    while (dma_stream->CR & DMA_SxCR_EN_Msk);
    // Source
    dma_stream->M0AR = (uint32_t) source;
    // Data transfer length: we want to go 1 by 1.
    dma_stream->NDTR = (uint32_t) 1;
    // Enable
    dma_stream->CR |= DMA_SxCR_EN;
}

void drawDiamond() {
    // drives GOb high when it is done so that we aren't drawing anything
    VEC_TIMER->DIER &= ~(TIM_DIER_UIE); // we don't want to jump into an interrupt until we are at the starting position
    generateDuration(VEC_TIMER, 1, 2);

    // load starting position
    loadCounter(512,512);
    delay_micros(DELAY_TIM,1000);

    // now generate an update to start the interrupt cycle
    VEC_TIMER->DIER |= TIM_DIER_UIE;
    generateDuration(VEC_TIMER, 1, 2);
}

void TIM5_IRQHandler() {
    VEC_TIMER->SR &= ~(TIM_SR_UIF); // software has to clear this flag

    // strobe shift register output latch
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);

    // generate GO signal
    generateDuration(VEC_TIMER, 1028, 5);

    //uint16_t** array = (currentData==1) ? cubeVectorData1 : cubeVectorData2;
    // and while we're drawing the current vector, let's start loading in the next one
    //doubleSendSPI(X_SPI, Y_SPI, cubeVectorData1[i][0], cubeVectorData1[i][1]);
    runDMA(X_DMA_STREAM, &cubeVectorData1[i][0]);
    runDMA(Y_DMA_STREAM, &cubeVectorData1[i][1]);
    ++i;
    i %= ARRAY_SIZE;
}

void WWDG_IRQHandler(){}

int main(void) {
    configure84MHzClock();

    configureUSART2();
    configureDelayTimer();
    configureBRM();

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    configureSPI(SPI1);
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    configureSPI(SPI3);

    configureGPIOs();

    // Turn on clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    configureDMA(X_DMA_STREAM, cubeVectorData1, &X_SPI->DR);
    configureDMA(Y_DMA_STREAM, cubeVectorData1, &Y_SPI->DR);

    __enable_irq(); // Enable interrupts globally

    loadColor(0, 0, 0b000, 0b011, 0b10);
    drawDiamond();

    // initial calculation
    rotateZCube(45);
    calculateCubeVectorData(cubeVectorData1);

    // default to on to show signs of life
    digitalWrite(GPIOA, LED_PIN, 1);

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
                    digitalWrite(LED_GPIO, LED_PIN, 1);
                    break;
                case ((uint16_t)'k'):
                    digitalWrite(LED_GPIO, LED_PIN, 0);
                    break;
                case ((uint16_t)'j'):
                    togglePin(LED_GPIO, LED_PIN);
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
