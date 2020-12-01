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

// Vector Data Bits
#define NEG (1<<10)    // applies negative direction; for use with x or y data
#define BLANK (1<<0)   // blanks current vector (i.e. makes it transparent); for use with z data
#define LD_COL (1<<1)  // loads new color data; for use with z data
#define LD_POS (1<<2) // loads new absolute position; for use with z data

/* Font */
uint16_t space_x[1] = {120};
uint16_t space_y[1] = {0};
uint16_t space_z[1] = {0};

uint16_t zero_x[5] = {0, 80, 0, NEG|80, 120};
uint16_t zero_y[5] = {120, 0, NEG|120, 0, 0};
uint16_t zero_z[5] = {0,0,0,0,BLANK};

uint16_t one_x[3] = {40,0,80};
uint16_t one_y[3] = {120,NEG|120,0};
uint16_t one_z[3] = {BLANK,0,BLANK};

uint16_t two_x[7] = {0,80,0,NEG|80,0,80,40};
uint16_t two_y[7] = {120,0,NEG|60,0,NEG|60,0,0};
uint16_t two_z[7] = {BLANK,0,0,0,0,0,BLANK};

uint16_t three_x[7] = {0,80,0,NEG|80,0,80,40};
uint16_t three_y[7] = {120,0,NEG|120,0,60,0,NEG|60};
uint16_t three_z[7] = {BLANK,0,0,0,BLANK,0,BLANK};

uint16_t four_x[6] = {0,0,80,0,0,40};
uint16_t four_y[6] = {120,NEG|60,0,60,NEG|120,0};
uint16_t four_z[6] = {BLANK,0,0,BLANK,0,BLANK};

uint16_t five_x[6] = {80,0,NEG|80,0,80,40};
uint16_t five_y[6] = {0,60,0,60,0,NEG|120};
uint16_t five_z[6] = {0,0,0,0,0,BLANK};

uint16_t six_x[6] = {0,80,0,NEG|80,0,120};
uint16_t six_y[6] = {60,0,NEG|60,0,120,NEG|120};
uint16_t six_z[6] = {BLANK,0,0,0,0,BLANK};

uint16_t seven_x[4] = {0,80,0,40};
uint16_t seven_y[4] = {120,0,NEG|120,0};
uint16_t seven_z[4] = {BLANK,0,0,BLANK};

uint16_t eight_x[7] = {0,80,0,NEG|80,0,80,40};
uint16_t eight_y[7] = {120,0,NEG|120,0,60,0,NEG|60};
uint16_t eight_z[7] = {0,0,0,0,BLANK,0,BLANK};

uint16_t nine_x[6] = {80,NEG|80,0,80,0,40};
uint16_t nine_y[6] = {60,0,60,0,NEG|120,0};
uint16_t nine_z[6] = {BLANK,0,0,0,0,BLANK};

uint16_t a_x[7] = {0,40,40,0,NEG|80,80,40};
uint16_t a_y[7] = {80,40,NEG|40,NEG|80,40,0,NEG|40};
uint16_t a_z[7] = {0,0,0,0,BLANK,0,BLANK};

uint16_t b_x[12] = {0,60,20,0,NEG|20,NEG|60,60,20,0,NEG|20,NEG|60,120};
uint16_t b_y[12] = {120,0,NEG|20,NEG|20,NEG|20,0,0,NEG|20,NEG|20,NEG|20,0,0};
uint16_t b_z[12] = {0,0,0,0,0,0,BLANK,0,0,0,0,BLANK};

uint16_t c_x[5] = {0,80,NEG|80,80,40};
uint16_t c_y[5] = {120,0,NEG|120,0,0};
uint16_t c_z[5] = {0,0,BLANK,0,BLANK};

uint16_t d_x[7] = {0,40,40,0,NEG|40,NEG|40,120};
uint16_t d_y[7] = {120,0,NEG|40,NEG|40,NEG|40,0,0};
uint16_t d_z[7] = {0,0,0,0,0,0,BLANK};

uint16_t e_x[7] = {80,NEG|80,0,80,NEG|20,NEG|60,120};
uint16_t e_y[7] = {0,0,120,0,NEG|60,0,NEG|60};
uint16_t e_z[7] = {0,BLANK,0,0,BLANK,0,BLANK};

uint16_t f_x[5] = {0,80,NEG|20,NEG|60,120};
uint16_t f_y[5] = {120,0,NEG|60,0,NEG|60};
uint16_t f_z[5] = {0,0,BLANK,0,BLANK};

uint16_t g_x[8] = {0,80,0,NEG|40,40,0,NEG|80,120};
uint16_t g_y[8] = {120,0,NEG|40,NEG|40,0,NEG|40,0,0};
uint16_t g_z[8] = {0,0,0,BLANK,0,0,0,BLANK};

uint16_t h_x[6] = {0,0,80,0,0,40};
uint16_t h_y[6] = {120,NEG|60,0,60,NEG|120,0};
uint16_t h_z[6] = {0,BLANK,0,BLANK,0,BLANK};

uint16_t i_x[6] = {80,NEG|80,80,NEG|40,0,80};
uint16_t i_y[6] = {0,120,0,0,NEG|120,0};
uint16_t i_z[6] = {0,BLANK,0,BLANK,0,BLANK};

uint16_t j_x[5] = {0,40,40,0,40};
uint16_t j_y[5] = {40,NEG|40,0,120,NEG|120};
uint16_t j_z[5] = {BLANK,0,0,0,BLANK};

uint16_t k_x[5] = {0,60,NEG|60,60,60};
uint16_t k_y[5] = {120,0,NEG|60,NEG|60,0};
uint16_t k_z[5] = {0,BLANK,0,0,BLANK};

uint16_t l_x[4] = {0,0,80,40};
uint16_t l_y[4] = {120,NEG|120,0,0};
uint16_t l_z[4] = {BLANK,0,0,BLANK};

uint16_t m_x[5] = {0,40,40,0,40};
uint16_t m_y[5] = {120,NEG|40,40,NEG|120,0};
uint16_t m_z[5] = {0,0,0,0,BLANK};

uint16_t n_x[4] = {0,80,0,40};
uint16_t n_y[4] = {120,NEG|120,120,NEG|120};
uint16_t n_z[4] = {0,0,0,BLANK};

uint16_t o_x[5] = {0, 80, 0, NEG|80, 120};
uint16_t o_y[5] = {120, 0, NEG|120, 0, 0};
uint16_t o_z[5] = {0,0,0,0,BLANK};

uint16_t p_x[5] = {0,80,0,NEG|80,120};
uint16_t p_y[5] = {120,0,NEG|60,0,NEG|60};
uint16_t p_z[5] = {0,0,0,0,BLANK};

uint16_t q_x[8] = {0,80,0,NEG|40,NEG|40,40,40,40};
uint16_t q_y[8] = {120,0,NEG|80,NEG|40,0,40,NEG|40,0};
uint16_t q_z[8] = {0,0,0,0,0,BLANK,0,BLANK};

uint16_t r_x[7] = {0,80,0,NEG|80,20,60,40};
uint16_t r_y[7] = {120,0,NEG|60,0,0,NEG|60,0};
uint16_t r_z[7] = {0,0,0,0,BLANK,0,BLANK};

uint16_t s_x[6] = {80,0,NEG|80,0,80,40};
uint16_t s_y[6] = {0,60,0,60,0,NEG|120};
uint16_t s_z[6] = {0,0,0,0,0,BLANK};

uint16_t t_x[5] = {0,80,NEG|40,0,80};
uint16_t t_y[5] = {120,0,0,NEG|120,0};
uint16_t t_z[5] = {BLANK,0,BLANK,0,BLANK};

uint16_t u_x[5] = {0,0,80,0,40};
uint16_t u_y[5] = {120,NEG|120,0,120,NEG|120};
uint16_t u_z[5] = {BLANK,0,0,0,BLANK};

uint16_t v_x[4] = {0,40,40,40};
uint16_t v_y[4] = {120,NEG|120,120,NEG|120};
uint16_t v_z[4] = {BLANK,0,0,BLANK};

uint16_t w_x[6] = {0,0,4,4,0,4};
uint16_t w_y[6] = {120,NEG|120,40,NEG|40,120,NEG|120};
uint16_t w_z[6] = {BLANK,0,0,0,0,BLANK};

uint16_t x_x[4] = {80,NEG|80,80,40};
uint16_t x_y[4] = {120,0,NEG|120,0};
uint16_t x_z[4] = {0,BLANK,0,BLANK};

uint16_t y_x[6] = {40,0,NEG|40,80,NEG|40,80};
uint16_t y_y[6] = {0,80,40,0,NEG|40,NEG|80};
uint16_t y_z[6] = {BLANK,0,0,BLANK,0,BLANK};

uint16_t z_x[5] = {0,80,NEG|80,80,40};
uint16_t z_y[5] = {120,0,NEG|120,0,0};
uint16_t z_z[5] = {BLANK,0,0,0,BLANK};

// Cube Vars
const unsigned int CUBE_VECTOR_DATA_SIZE = 76; // words
#define ARRAY_SIZE 38
uint16_t cubeVectorData[2][ARRAY_SIZE];
uint16_t cubeZData[38] = {BLANK,BLANK,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,0,0,
                          BLANK,0,BLANK,0,BLANK,0,BLANK,BLANK};

// Keyboard Input Vars
volatile int newCommandAvailable = 0;
volatile uint16_t lastCommand;

/* Vector Vars */
// Buffer Struct
#define BUFFER_SIZE 200
typedef struct {
    uint16_t x[BUFFER_SIZE]; // stores x data to be sent to hardware
    uint16_t y[BUFFER_SIZE]; // stores y data to be sent to hardware
    uint16_t z[BUFFER_SIZE]; // stores software controls
    unsigned int top;        // points to where the first free chunk of memory is
    unsigned int anim_index; // points to the first vector of the first animated object is (used in write mode)
    unsigned int read_index; // points to where the first unread vector is (used in read mode)
    //
    // Here's a picture to visualize (not to scale).
    //
    //                X           Y         Z
    //           0 --------    --------    ----
    //             (vector data that does often
    //              not change frame to frame)
    //  anim_index --------    --------    ----
    //             (vector data that does often 
    //              change frame to frame)
    //         top --------    --------    ----
    //             (unusued memory)
    // BUFFER_SIZE --------    --------    ----
    //
} vector_buffer;
// Actual Buffers
vector_buffer buff_0_value; // value as in not a pointer
vector_buffer buff_1_value;
// Aliases to Actual Buffers
vector_buffer* buff_r = &buff_0_value; // r for currently being read from (and drawn to screen)
vector_buffer* buff_w = &buff_1_value; // w for currently being written
// Color State
uint8_t x_color=0; // current color
uint8_t y_color=0; 
unsigned int blanked=0; // stores whether the previous vector was blanked
// Current Instruction Being Drawn to Screen
uint16_t curr_x=0;
uint16_t curr_y=0;
uint16_t curr_z=0;
// Buffer Switch Request
unsigned int buffer_swap_req=0; // draw-er issues the request, and when comput-er is done calculating the next frame, the request is granted

void swapBuffers() {
    if (buff_r==&buff_0_value) {
        buff_r = &buff_1_value;
        buff_w = &buff_0_value;
    } else {
        buff_r = &buff_0_value;
        buff_w = &buff_1_value;
    }
    buff_r->read_index=0;
    buff_w->top=buff_w->anim_index; // we consider anything that needs to be animated as unwritten to begin with
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

    pinMode(GPIOA, GPIO_PA4, GPIO_OUTPUT); // COUNT_LDb (blue wire)
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

void runDMA(uint16_t * source, uint16_t * destination, unsigned int numberOfDatas) {
    // Disable stream
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN_Msk); // wait until stream is off

    // Clear Status Registers
    DMA2->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0);

    // Configure source and destination
    DMA2_Stream0->PAR  = (uint32_t) source;
    DMA2_Stream0->M0AR = (uint32_t) destination;

    // Data transfer length
    DMA2_Stream0->NDTR = (uint32_t) numberOfDatas;

    // Enable stream
    DMA2_Stream0->CR |= DMA_SxCR_EN;
}

void addVectorsToBuffer(uint16_t* x_data, uint16_t* y_data, uint16_t* z_data, unsigned int length) {
    // Adds an array of <length> vectors to write buffer
    //
    // use DMA to do busywork of copying
    runDMA(x_data, &buff_w->x[buff_w->top], length);
    runDMA(y_data, &buff_w->y[buff_w->top], length);
    runDMA(z_data, &buff_w->z[buff_w->top], length);
    // increment allocated space
    buff_w->top+=length;
}

void addLoadToBuffer(uint16_t x_pos, uint16_t y_pos, unsigned int red, unsigned int green, unsigned int blue) {
    // Adds a load instruction to write buffer.
    // A load instruction directly sets beam to an absolute position onscreen.
    // And while we're at it, it can also set the current color.
    // red:   3 bits
    // green: 3 bits
    // blue:  2 bits
    //
    // write hardware control data (stuff actually sent to shift registers)
    buff_w->x[buff_w->top] = ((green<<14) | (blue<<12) | x_pos);
    buff_w->y[buff_w->top] = ((red<<13) | ((green>>2)<<12) | y_pos);
    // write software control data
    buff_w->z[buff_w->top] = LD_COL | LD_POS;
    // increment allocated space
    buff_w->top++;
}

void beginDrawing() {
    // load in initial vector to get things started
    // it would be wise for this first vector to load a starting position and color
    X_SPI->DR = buff_r->x[buff_r->read_index];
    Y_SPI->DR = buff_r->y[buff_r->read_index];

    // now generate an update to start the interrupt cycle
    VEC_TIMER->DIER |= TIM_DIER_UIE;
    // since we aren't drawing a vector yet, we need to manually ensure the data have time to be loaded in,
    // and set CCR1 so that GOb never activates
    generateDuration(VEC_TIMER, 300, 300);
}

void fetchNextVector() {
    // Fetch next vector
    curr_x = buff_r->x[buff_r->read_index];
    curr_y = buff_r->y[buff_r->read_index];
    curr_z = buff_r->z[buff_r->read_index];
    buff_r->read_index++;
    if (buff_r->read_index >= buff_r->top) {
        buff_r->read_index = 0;
        buffer_swap_req=1; // draw-er makes the request
    }
}

// This is the main function for handling drawing stuff
void TIM5_IRQHandler() {
    // Clear interrupt flag.
    VEC_TIMER->SR &= ~(TIM_SR_UIF);
    // If we're waiting on the comput-er to finish the next frame, hold tight and wait
    if (buffer_swap_req) return;
    // Output the previous vector's data by strobing shift reg latch.
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_HIGH);
    digitalWrite(GPIOA, X_SHIFT_REG_LD, GPIO_LOW);
    digitalWrite(GPIOC, Y_SHIFT_REG_LD, GPIO_LOW);
    // Apply control signals for previous vector.
    //   Strobe color latch if we need to.
    //   By default, let's always apply color.
    digitalWrite(GPIOA, COLOR_LD, GPIO_HIGH);
    digitalWrite(GPIOA, COLOR_LD, GPIO_LOW);
    //   Strobe counter parallel load if we need to (moves beam to absolute position)
    if (curr_z & LD_POS) {
        // If so, strobe the counter parallel load.
        digitalWrite(GPIOA, COUNT_LDb, GPIO_LOW);
        digitalWrite(GPIOA, COUNT_LDb, GPIO_HIGH);
        fetchNextVector();
        // By default, let's always add color unless we're blanking the vector
        if (!(curr_z&BLANK)) {
            curr_x |= x_color<<12;
            curr_y |= y_color<<12;
        }
        // Send out next vector
        X_SPI->DR = curr_x;
        Y_SPI->DR = curr_y;
        // Wait until it's done sending
        while(!(Y_SPI->SR & SPI_SR_TXE));
        // And wait some more for the beam to finish moving
        // Lest we turn on the electron beam in the middle of jumping
        delay_micros(DELAY_TIM, 70);
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
    // By default, let's always add color unless we're blanking the vector
    if (!(curr_z&BLANK)) {
        curr_x |= x_color<<12;
        curr_y |= y_color<<12;
    }
    // Now we can start start sending out the data for the next vector.
    X_SPI->DR = curr_x;
    Y_SPI->DR = curr_y;
}

void WWDG_IRQHandler(){}

void staticImages() {
    addLoadToBuffer(260, 800, 0b000, 0b010, 0b10);
    addVectorsToBuffer(h_x,h_y,h_z,6);
    addVectorsToBuffer(e_x,e_y,e_z,7);
    addVectorsToBuffer(l_x,l_y,l_z,4);
    addVectorsToBuffer(l_x,l_y,l_z,4);
    addVectorsToBuffer(o_x,o_y,o_z,5);
    addLoadToBuffer(185, 150, 0b010, 0b000, 0b10);
    addVectorsToBuffer(g_x,g_y,g_z,8);
    addVectorsToBuffer(a_x,a_y,a_z,7);
    addVectorsToBuffer(m_x,m_y,m_z,5);
    addVectorsToBuffer(e_x,e_y,e_z,7);
    addVectorsToBuffer(r_x,r_y,r_z,7);
    addVectorsToBuffer(s_x,s_y,s_z,6);
    buff_w->anim_index=buff_w->top;
}
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

    staticImages();
    swapBuffers();
    // and regenerate images for second buffer
    staticImages();

    // drives GOb high when it is done so that we aren't drawing anything
    VEC_TIMER->DIER &= ~(TIM_DIER_UIE); // we don't want to jump into an interrupt until we are at the starting position
    generateDuration(VEC_TIMER, 1, 2);
    beginDrawing();

    // initial calculation
    rotateYCube(LEFT_CUBE,  15);
    rotateZCube(RIGHT_CUBE, 75);
    calculateCubeVectorData(cubeVectorData);

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
            calculateCubeVectorData(cubeVectorData);
        }
        buff_w->top=buff_w->anim_index;
        addLoadToBuffer(512, 450, 0b010, 0b010, 0b00);
        addVectorsToBuffer(cubeVectorData[0],cubeVectorData[1],cubeZData,19);
        addLoadToBuffer(512, 450, 0b000, 0b000, 0b11);
        addVectorsToBuffer(&cubeVectorData[0][19],&cubeVectorData[1][19],cubeZData,19);
        // if draw-er has requested the next buffer,
        // the comput-er can now oblige that request
        if (buffer_swap_req) {
            buffer_swap_req=0;
            swapBuffers();
            beginDrawing();
        }
    }
    return 0;
}
