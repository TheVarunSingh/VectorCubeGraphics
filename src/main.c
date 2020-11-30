// https://www.tutorialspoint.com/computer_graphics/3d_transformation.htm
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

const unsigned int CUBE_VECTOR_DATA_SIZE = 76; // words
#define ARRAY_SIZE 38
uint16_t cubeVectorData1[ARRAY_SIZE][2];
uint16_t cubeVectorData2[ARRAY_SIZE][2];

volatile int newCommandAvailable = 0;
volatile uint16_t lastCommand;
volatile int currentData = 1;

//uint16_t array[ARRAY_SIZE][2] = {{512,0},{0,512},{NEG|512,NEG|512},{NEG|512,0},{0,NEG|512},{512,512}};
//uint16_t x_array[ARRAY_SIZE] = {512,0,NEG|512,NEG|512,0,512};
//uint16_t y_array[ARRAY_SIZE] = {0,512,NEG|512,0,NEG|512,512};

uint16_t testDataSize = 6;
uint16_t testDataSource[6] = {101, 102, 103, 104, 105, 106};
uint16_t testDataDest[6] = {4,5,6,7,8,9};

// Vector Data Bits
#define NEG (1<<10)    // applies negative direction; for use with x or y data
#define BNK (1<<8)     // blanks current vector (i.e. makes it transparent); for use with z data
#define LD_COL (1<<9)  // loads new color data; for use with z data
#define LD_POS (1<<10) // loads new absolute position

/* Font */
uint16_t space_x[1] = {120};
uint16_t space_y[1] = {0};
uint16_t space_z[1] = {0};

uint16_t zero_x[5] = {0, 80, 0, NEG|80, 120};
uint16_t zero_y[5] = {120, 0, NEG|120, 0, 0};
uint16_t zero_z[5] = {0,0,0,0,BNK};

uint16_t one_x[3] = {40,0,80};
uint16_t one_y[3] = {120,NEG|120,0};
uint16_t one_z[3] = {BNK,0,BNK};

uint16_t two_x[7] = {0,80,0,NEG|80,0,80,40};
uint16_t two_y[7] = {120,0,NEG|60,0,NEG|60,0,0};
uint16_t two_z[7] = {BNK,0,0,0,0,0,BNK};

uint16_t three_x[7] = {0,80,0,NEG|80,0,80,40};
uint16_t three_y[7] = {120,0,NEG|120,0,60,0,NEG|60};
uint16_t three_z[7] = {BNK,0,0,0,BNK,0,BNK};

uint16_t four_x[6] = {0,0,80,0,0,40};
uint16_t four_y[6] = {120,NEG|60,0,60,NEG|120,0};
uint16_t four_z[6] = {BNK,0,0,BNK,0,BNK};

uint16_t five_x[6] = {80,0,NEG|80,0,80,40};
uint16_t five_y[6] = {0,60,0,60,0,NEG|120};
uint16_t five_z[6] = {0,0,0,0,0,BNK};

uint16_t six_x[6] = {0,80,0,NEG|80,0,120};
uint16_t six_y[6] = {60,0,NEG|60,0,120,NEG|120};
uint16_t six_z[6] = {BNK,0,0,0,0,BNK};

uint16_t seven_x[4] = {0,80,0,40};
uint16_t seven_y[4] = {120,0,NEG|120,0};
uint16_t seven_z[4] = {BNK,0,0,BNK};

uint16_t eight_x[7] = {0,80,0,NEG|80,0,80,40};
uint16_t eight_y[7] = {120,0,NEG|120,0,60,0,NEG|60};
uint16_t eight_z[7] = {0,0,0,0,BNK,0,BNK};

uint16_t nine_x[6] = {80,NEG|80,0,80,0,40};
uint16_t nine_y[6] = {60,0,60,0,NEG|120,0};
uint16_t nine_z[6] = {BNK,0,0,0,0,BNK};

uint16_t a_x[7] = {0,40,40,0,NEG|80,80,40};
uint16_t a_y[7] = {80,40,NEG|40,NEG|80,40,0,NEG|40};
uint16_t a_z[7] = {0,0,0,0,BNK,0,BNK};

uint16_t b_x[12] = {0,60,20,0,NEG|20,NEG|60,60,20,0,NEG|20,NEG|60,120};
uint16_t b_y[12] = {120,0,NEG|20,NEG|20,NEG|20,0,0,NEG|20,NEG|20,NEG|20,0,0};
uint16_t b_z[12] = {0,0,0,0,0,0,BNK,0,0,0,0,BNK};

uint16_t c_x[5] = {0,80,NEG|80,80,40};
uint16_t c_y[5] = {120,0,NEG|120,0,0};
uint16_t c_z[5] = {0,0,BNK,0,BNK};

uint16_t d_x[7] = {0,40,40,0,NEG|40,NEG|40,120};
uint16_t d_y[7] = {120,0,NEG|40,NEG|40,NEG|40,0,0};
uint16_t d_z[7] = {0,0,0,0,0,0,BNK};

uint16_t e_x[7] = {80,NEG|80,0,80,NEG|20,NEG|60,120};
uint16_t e_y[7] = {0,0,120,0,NEG|60,0,NEG|60};
uint16_t e_z[7] = {0,BNK,0,0,BNK,0,BNK};

uint16_t f_x[5] = {0,80,NEG|20,NEG|60,120};
uint16_t f_y[5] = {120,0,NEG|60,0,NEG|60};
uint16_t f_z[5] = {0,0,BNK,0,BNK};

uint16_t g_x[8] = {0,80,0,NEG|40,40,0,NEG|80,120};
uint16_t g_y[8] = {120,0,NEG|40,NEG|40,0,NEG|40,0,0};
uint16_t g_z[8] = {0,0,0,BNK,0,0,0,BNK};

uint16_t h_x[6] = {0,0,80,0,0,40};
uint16_t h_y[6] = {120,NEG|60,0,60,NEG|120,0};
uint16_t h_z[6] = {0,BNK,0,BNK,0,BNK};

uint16_t i_x[6] = {80,NEG|80,80,NEG|40,0,80};
uint16_t i_y[6] = {0,120,0,0,NEG|120,0};
uint16_t i_z[6] = {0,BNK,0,BNK,0,BNK};

uint16_t j_x[5] = {0,40,40,0,40};
uint16_t j_y[5] = {40,NEG|40,0,120,NEG|120};
uint16_t j_z[5] = {BNK,0,0,0,BNK};

uint16_t k_x[5] = {0,60,NEG|60,60,60};
uint16_t k_y[5] = {120,0,NEG|60,NEG|60,0};
uint16_t k_z[5] = {0,BNK,0,0,BNK};

uint16_t l_x[4] = {0,0,80,40};
uint16_t l_y[4] = {120,NEG|120,0,0};
uint16_t l_z[4] = {BNK,0,0,BNK};

uint16_t m_x[5] = {0,40,40,0,40};
uint16_t m_y[5] = {120,NEG|40,40,NEG|120,0};
uint16_t m_z[5] = {0,0,0,0,BNK};

uint16_t n_x[4] = {0,80,0,40};
uint16_t n_y[4] = {120,NEG|120,120,NEG|120};
uint16_t n_z[4] = {0,0,0,BNK};

uint16_t o_x[5] = {0, 80, 0, NEG|80, 120};
uint16_t o_y[5] = {120, 0, NEG|120, 0, 0};
uint16_t o_z[5] = {0,0,0,0,BNK};

uint16_t p_x[5] = {0,80,0,NEG|80,120};
uint16_t p_y[5] = {120,0,NEG|60,0,NEG|60};
uint16_t p_z[5] = {0,0,0,0,BNK};

uint16_t q_x[8] = {0,80,0,NEG|40,NEG|40,40,40,40};
uint16_t q_y[8] = {120,0,NEG|80,NEG|40,0,40,NEG|40,0};
uint16_t q_z[8] = {0,0,0,0,0,BNK,0,BNK};

uint16_t r_x[7] = {0,80,0,NEG|80,20,60,40};
uint16_t r_y[7] = {120,0,NEG|60,0,0,NEG|60,0};
uint16_t r_z[7] = {0,0,0,0,BNK,0,BNK};

uint16_t s_x[6] = {80,0,NEG|80,0,80,40};
uint16_t s_y[6] = {0,60,0,60,0,NEG|120};
uint16_t s_z[6] = {0,0,0,0,0,BNK};

uint16_t t_x[5] = {0,80,NEG|40,0,80};
uint16_t t_y[5] = {120,0,0,NEG|120,0};
uint16_t t_z[5] = {BNK,0,BNK,0,BNK};

uint16_t u_x[5] = {0,0,80,0,40};
uint16_t u_y[5] = {120,NEG|120,0,120,NEG|120};
uint16_t u_z[5] = {BNK,0,0,0,BNK};

uint16_t v_x[4] = {0,40,40,40};
uint16_t v_y[4] = {120,NEG|120,120,NEG|120};
uint16_t v_z[4] = {BNK,0,0,BNK};

uint16_t w_x[6] = {0,0,4,4,0,4};
uint16_t w_y[6] = {120,NEG|120,40,NEG|40,120,NEG|120};
uint16_t w_z[6] = {BNK,0,0,0,0,BNK};

uint16_t x_x[4] = {80,NEG|80,80,40};
uint16_t x_y[4] = {120,0,NEG|120,0};
uint16_t x_z[4] = {0,BNK,0,BNK};

uint16_t y_x[6] = {40,0,NEG|40,80,NEG|40,80};
uint16_t y_y[6] = {0,80,40,0,NEG|40,NEG|80};
uint16_t y_z[6] = {BNK,0,0,BNK,0,BNK};

uint16_t z_x[5] = {0,80,NEG|80,80,40};
uint16_t z_y[5] = {120,0,NEG|120,0,0};
uint16_t z_z[5] = {BNK,0,0,0,BNK};

/* Global Vector Vars */
// Actual Buffers
uint16_t x_buffer0[200]; // stores x data to be drawn
uint16_t y_buffer0[200]; // stores y data to be drawn
uint16_t z_buffer0[200]; // stores intensity and control signal information
uint16_t x_buffer1[200]; // stores x data to be drawn
uint16_t y_buffer1[200]; // stores y data to be drawn
uint16_t z_buffer1[200]; // stores intensity and control signal information
// Aliases to Actual Buffers
uint16_t* x_d_buffer=x_buffer0; // d for currently being drawn
uint16_t* y_d_buffer=y_buffer0; // d for currently being drawn
uint16_t* z_d_buffer=z_buffer0; // d for currently being drawn
uint16_t* x_w_buffer=x_buffer1; // w for currently being written
uint16_t* y_w_buffer=y_buffer1; // w for currently being written
uint16_t* z_w_buffer=z_buffer1; // w for currently being written
// Indices
unsigned int d_index=0; // index of vector currently being drawn (operates like an iterator)
unsigned int w_index=0; // index of vector currently being written (operates like a stack pointer)
unsigned int d_last=0;  // when d_index == d_last, we know we have reached the end of the data to be drawn
// Color State
uint8_t x_color=0;   // current or last color (this is used to reinstate color for a non-blanked vector after a blanked vector has been drawn)
uint8_t y_color=0;   // current or last color (this is used to reinstate color for a non-blanked vector after a blanked vector has been drawn)
unsigned int blanked=0; // stores whether the previous vector was blanked
// Previous Instruction
uint16_t curr_x=0;
uint16_t curr_y=0;
uint16_t curr_z=0;

void addLoad(uint16_t x_pos, uint16_t y_pos, unsigned int red, unsigned int green, unsigned int blue) {
    // this might actually belong in vec_gen library
    // Note that red and green use 3 bits and blue uses 2 bits.
    // write hardware-relevant dat0
    x_w_buffer[w_index] = ((green<<14) | (blue<<12) | x_pos);
    y_w_buffer[w_index] = ((red<<13) | ((green>>2)<<12) | y_pos);
    // write control data
    z_w_buffer[w_index] = LD_COL | LD_POS;
    // increment allocated space
    w_index++;
}

void swapBuffers() {
    if (x_d_buffer==x_buffer0) {
        x_d_buffer=x_buffer1; 
        y_d_buffer=y_buffer1; 
        z_d_buffer=z_buffer1; 
        x_w_buffer=x_buffer0;
        y_w_buffer=y_buffer0; 
        z_w_buffer=z_buffer0;
    } else {
        x_d_buffer=x_buffer0; 
        y_d_buffer=y_buffer0; 
        z_d_buffer=z_buffer0; 
        x_w_buffer=x_buffer1;
        y_w_buffer=y_buffer1; 
        z_w_buffer=z_buffer1;
    }
    d_last = w_index;
    w_index = 0;
    d_index = 0;
}

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

void configureDMA() {
    // Using DMA 2 Stream 0 for memory-to-memory transfer
    // Note that in memory-to-memory, the peripheral stuff is the source and the memory stuff is the dest

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Disable stream
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN_Msk); // wait until stream is off

    // Reset configuration values
    // - Select channel 0 (channel does not matter for memory-to-memory)
    // - Single transfer mode
    // - Disable double buffer mode
    // - Priority level low (0/4) - will be set to high later
    // - Memory and peripheral data size are 8 bits - will be set to 16 bits later
    // - Disable memory and peripheral increment mode - will be enabled later
    // - Disable circular mode
    // - Peripheral-to-memory mode - will be set to memory-to-memory later
    // - Set DMA as the flow controller
    // - Disable DMA interrupts - some will be enabled later
    // - Disable the stream (should already be disabled)
    DMA2_Stream0->CR = 0;

    DMA2_Stream0->CR |= (0b10 << DMA_SxCR_PL_Pos);    // high priority (3/4)
    DMA2_Stream0->CR |= (0b01 << DMA_SxCR_MSIZE_Pos); // 16-bit memory data size
    DMA2_Stream0->CR |= (0b01 << DMA_SxCR_PSIZE_Pos); // 16-bit peripheral data size (is this even needed?)
    DMA2_Stream0->CR |= (0b10 << DMA_SxCR_DIR_Pos);   // memory-to-memory mode
    DMA2_Stream0->CR |= DMA_SxCR_MINC;                // enable memory increment mode
    DMA2_Stream0->CR |= DMA_SxCR_PINC;                // enable peripheral increment mode
}

void runDMA(uint16_t * source, uint16_t * destination, uint32_t numberOfDatas) {
    // Disable stream
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN_Msk); // wait until stream is off

    // Configure source and destination
    DMA2_Stream0->PAR  = (uint32_t) source;
    DMA2_Stream0->M0AR = (uint32_t) destination;

    // Data transfer length
    DMA2_Stream0->NDTR = (uint32_t) numberOfDatas;

    // Enable stream
    DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void notDMA(uint16_t* source, uint16_t* destination, uint32_t numberOfDatas) {
    for (unsigned int i=0;i<numberOfDatas;i++) {
        destination[i]=source[i];
    }
}

void beginDrawing() {
    // drives GOb high when it is done so that we aren't drawing anything
    VEC_TIMER->DIER &= ~(TIM_DIER_UIE); // we don't want to jump into an interrupt until we are at the starting position
    generateDuration(VEC_TIMER, 1, 2);
    // load in initial vector to get things started
    // it would be wise for this first vector to load a starting position and color
    doubleSendSPI(X_SPI, Y_SPI, x_d_buffer[d_index], y_d_buffer[d_index]);
    // now generate an update to start the interrupt cycle
    VEC_TIMER->DIER |= TIM_DIER_UIE;
    // since we aren't drawing a vector yet, we need to manually ensure the data have time to be loaded in,
    // and set CCR1 so that GOb never activates
    generateDuration(VEC_TIMER, 50, 50);
}

void fetchNextVector() {
    // Fetch next vector
    curr_x = x_d_buffer[d_index];
    curr_y = y_d_buffer[d_index];
    curr_z = z_d_buffer[d_index];
    d_index++;
    if (d_index >= d_last) {
        // TODO: Swap buffers or raise a flag telling main loop to do that
        d_index = 0;
    }
}

// This is the main function for handling drawing stuff
void TIM5_IRQHandler() {
    // Clear interrupt flag.
    VEC_TIMER->SR &= ~(TIM_SR_UIF);
    // Output the previous vector's data by strobing shift reg latch.
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);
    // Apply control signals for previous vector.
    //   Strobe color latch if we need to.
    //   For now, let's always apply color.
    //if (curr_z & LD_COL) {
    digitalWrite(GPIOA, COLOR_LD, GPIO_HIGH);
    digitalWrite(GPIOA, COLOR_LD, GPIO_LOW);
    //}
    //   Strobe counter parallel load if we need to (moves beam to absolute position)
    if (curr_z & LD_POS) {
        // If so, strobe the counter parallel load.
        digitalWrite(GPIOA, COUNT_LD, GPIO_LOW);
        digitalWrite(GPIOA, COUNT_LD, GPIO_HIGH);
        fetchNextVector();
        // For now, let's always add color unless we're blanking the vector
        if (!(curr_z&BNK)) {
            curr_x |= x_color<<12;
            curr_y |= y_color<<12;
        }
        // Send out next vector
        doubleSendSPI(X_SPI, Y_SPI, curr_x, curr_y);
        // Wait until it's done sending
        while(!(Y_SPI->SR & SPI_SR_TXE));
        // And wait some more for the beam to finish moving
        // Lest we turn on the electron beam in the middle of jumping
        delay_micros(DELAY_TIM, 100);
        // Let's go through this again
        TIM5_IRQHandler();
        // Sure an SPI interrupt might save some time, but we can get away with this simplicity 
        // because absolute position loads are pretty infrequent.
    } else {
        // Run BRM's to draw a line
        generateDuration(VEC_TIMER, 1028, 5);
        // yes we are counting on the rest of this function to run before this duration times up
        fetchNextVector();
    }
    // Now let's manually and sequentially process sending the next vector.
    // Yeah sure there might be some more optimizations if you do this a-sequentially,
    // (so that you can process stuff while running an SPI transfer),
    // but I found that very hard to debug because you're juggling the next vector and the current vector all at once.
    // The STM's pretty fast so I think we can trade in some multitasking for reliability.

    // If says it has color, let's note down that color
    if (curr_z&LD_COL) {
        x_color = curr_x>>12;
        y_color = curr_y>>12;
    }
    // For now, let's always add color unless we're blanking the vector
    if (!(curr_z&BNK)) {
        curr_x |= x_color<<12;
        curr_y |= y_color<<12;
    }
    // Now we can start start sending out the data for the next vector.
    doubleSendSPI(X_SPI, Y_SPI, curr_x, curr_y);
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

    configureDMA();

    __enable_irq(); // Enable interrupts globally

    runDMA(testDataSource, testDataDest, testDataSize);

    // generate test image
    addLoad(260, 602, 0b000, 0b100, 0b00);
    runDMA(h_x,&x_w_buffer[w_index],6);
    runDMA(h_y,&y_w_buffer[w_index],6);
    runDMA(h_z,&z_w_buffer[w_index],6);
    w_index+=6;
    runDMA(e_x,&x_w_buffer[w_index],7);
    runDMA(e_y,&y_w_buffer[w_index],7);
    runDMA(e_z,&z_w_buffer[w_index],7);
    w_index+=7;
    runDMA(l_x,&x_w_buffer[w_index],4);
    runDMA(l_y,&y_w_buffer[w_index],4);
    runDMA(l_z,&z_w_buffer[w_index],4);
    w_index+=4;
    runDMA(l_x,&x_w_buffer[w_index],4);
    runDMA(l_y,&y_w_buffer[w_index],4);
    runDMA(l_z,&z_w_buffer[w_index],4);
    w_index+=4;
    runDMA(o_x,&x_w_buffer[w_index],5);
    runDMA(o_y,&y_w_buffer[w_index],5);
    runDMA(o_z,&z_w_buffer[w_index],5);
    w_index+=5;
    addLoad(185, 342, 0b011, 0b000, 0b00);
    runDMA(g_x,&x_w_buffer[w_index],8);
    runDMA(g_y,&y_w_buffer[w_index],8);
    runDMA(g_z,&z_w_buffer[w_index],8);
    w_index+=8;
    runDMA(a_x,&x_w_buffer[w_index],7);
    runDMA(a_y,&y_w_buffer[w_index],7);
    runDMA(a_z,&z_w_buffer[w_index],7);
    w_index+=7;
    runDMA(m_x,&x_w_buffer[w_index],5);
    runDMA(m_y,&y_w_buffer[w_index],5);
    runDMA(m_z,&z_w_buffer[w_index],5);
    w_index+=5;
    runDMA(e_x,&x_w_buffer[w_index],7);
    runDMA(e_y,&y_w_buffer[w_index],7);
    runDMA(e_z,&z_w_buffer[w_index],7);
    w_index+=7;
    runDMA(r_x,&x_w_buffer[w_index],7);
    runDMA(r_y,&y_w_buffer[w_index],7);
    runDMA(r_z,&z_w_buffer[w_index],7);
    w_index+=7;
    runDMA(s_x,&x_w_buffer[w_index],6);
    runDMA(s_y,&y_w_buffer[w_index],6);
    runDMA(s_z,&z_w_buffer[w_index],6);
    w_index+=6;
    
    swapBuffers();

    beginDrawing();

    // initial calculation
    rotateYCube(LEFT_CUBE,  45);
    rotateZCube(RIGHT_CUBE, 45);
    calculateCubeVectorData(cubeVectorData1);

    // default to on to show signs of life
    digitalWrite(GPIOA, LED_PIN, 1);

    while (1) {
        if (newCommandAvailable) {
            newCommandAvailable = 0;

            switch (lastCommand) {
                case ((uint16_t)'w'):
                    rotateYCube(LEFT_CUBE,  1);
                    break;
                case ((uint16_t)'s'):
                    rotateYCube(LEFT_CUBE,  -1);
                    break;
                case ((uint16_t)'a'):
                    rotateZCube(LEFT_CUBE,  1);
                    break;
                case ((uint16_t)'d'):
                    rotateZCube(LEFT_CUBE,  -1);
                    break;
                case ((uint16_t)'q'):
                    scaleCube(LEFT_CUBE,  1.01, 1.01, 1.01);
                    break;
                case ((uint16_t)'e'):
                    scaleCube(LEFT_CUBE,  0.99, 0.99, 0.99);
                    break;

                case ((uint16_t)'i'):
                    rotateYCube(RIGHT_CUBE, 1);
                    break;
                case ((uint16_t)'k'):
                    rotateYCube(RIGHT_CUBE, -1);
                    break;
                case ((uint16_t)'j'):
                    rotateZCube(RIGHT_CUBE, 1);
                    break;
                case ((uint16_t)'l'):
                    rotateZCube(RIGHT_CUBE, -1);
                    break;
                case ((uint16_t)'u'):
                    scaleCube(RIGHT_CUBE, 1.01, 1.01, 1.01);
                    break;
                case ((uint16_t)'o'):
                    scaleCube(RIGHT_CUBE, 0.99, 0.99, 0.99);
                    break;

                case ((uint16_t)'v'):
                    togglePin(LED_GPIO, LED_PIN);
                    break;
                case ((uint16_t)'n'):
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
