#include "timers.h"

void configureTimer(TIM_TypeDef * TIMx) {
    // Set prescaler division factor
    TIMx->PSC = (uint32_t)(84-1); // Assuming 84 MHz
    // Generate an update event to update prescaler value
    TIMx->EGR |= TIM_EGR_UG;
    // Enable counter
    TIMx->CR1 |= TIM_CR1_CEN;
}

void configureCaptureCompare(TIM_TypeDef * TIMx) {
    /* Disable Counter */
    TIMx->CR1 &= ~(TIM_CR1_CEN);                    // disable counter
    /* Configure Capture/Compare Channel 1 for PWM Output */
    TIMx->CCER &= ~(TIM_CCER_CC1E);                 // disable capture/compare channel 1 so we can fiddle with its settings
    TIMx->CCMR1 &= ~(TIM_CCMR1_CC1S);               // capture/compare channel to output mode
    TIMx->CCMR1 |= (0b111 << TIM_CCMR1_OC1M_Pos);   // set capture/compare channel 1 to PWM mode
    TIMx->CCER |= TIM_CCER_CC1P;                    // set polarity to active low
    TIMx->CCMR1 |= TIM_CCMR1_OC1PE;                 // set capture/compare channel 1 preload enable
    TIMx->CCER |= TIM_CCER_CC1E;                    // enable capture/compare channel 1
}

void configureDuration(TIM_TypeDef * TIMx, uint32_t prescale, uint8_t slave_mode, uint8_t master_src) {
    /* Configures a timer for outputting large precise delays.
       Given a TIMER_BASE_FREQ of 84 MHz, the resolution (each counter tick) is 1 / 84 MHz * prescale
       Likewise the longest countable duration is resolution * 2^(bits in ARR)
    */

    /* Disable Counter */
    TIMx->CR1 &= ~(TIM_CR1_CEN); // disable counter
    /* Select input clock */
    if (slave_mode) {
        // select which master's trigger to respond to (see table 54 of ref manual)
        TIMx->SMCR &= ~(TIM_SMCR_TS);
        TIMx->SMCR |= (master_src << TIM_SMCR_TS_Pos);
        // select external clock mode (aka output of the trigger selection mux)
        TIMx->SMCR |= (0b111 << TIM_SMCR_SMS_Pos);
    } else {
        // disable slave mode controller;
        // use instead internal clock CK_INT (aka "TIMx_CLK from RCC")
        TIMx->SMCR &= ~(TIM_SMCR_SMS);
    }

    /* Prescale */
    TIMx->PSC = prescale;

    /* Configure Counter as Upcounter */
    TIMx->CR1 &= ~(TIM_CR1_CMS); // use edge-aligned mode (i.e. plain vanilla up or down counting)
    TIMx->CR1 &= ~(TIM_CR1_DIR); // upcounter mode

    /* Allow Automatic Updating and Interrupting on Overflow Events */
    TIMx->CR1 |= TIM_CR1_ARPE;  // "auto-reload preload enabled"
                                // transfers preload register to shadow register every update event, meaning
                                // we can change ARR and PSC without having to manually request an update
    TIMx->ARR = 1;  // set max count to 1 as a default so that
                    // we generate update events quickly when we turn timer on
    TIMx->CR1 |= TIM_CR1_OPM;   // stop counter at update events; we want to time out just one pulse
    //TIMx->DIER |= TIM_DIER_UIE; // enable interrupts upon updating

    /* Implement Settings */
    TIMx->CR1 |= TIM_CR1_URS;   // let only counter under/overflows generate interrupts
                                // so that when we manually generate an update, it doesn't make an interrupt
    TIMx->EGR |= TIM_EGR_UG;    // manually generate an update to initialize all shadow registers
}

void generateDuration(TIM_TypeDef * TIMx, uint32_t duration, uint32_t compare_val) {
    /* Generates an update after the specified duration.
     * The actual time duration is resolution (determined by configureDuration) * duration
     * Note that duration should not exceed 2^31-1 (TIM2,5) or 2^15-1 (TIM3,4) */
    TIMx->CR1 |= TIM_CR1_UDIS;  // Disable update events because apparently it would be bad
                                // if the timer by chance tried to update the shadow registers
                                // while we're writing new values in the preload registers.
    TIMx->ARR = duration;
    TIMx->CCR1 = compare_val;

    TIMx->CR1 &= ~(TIM_CR1_UDIS);   // enable update events;
                                    // (i.e. send update signal every time counter under/over-flows)
                                    // In duration mode, this will generate an interrupt.

    TIMx->EGR |= TIM_EGR_UG;  // manually generate an update to initialize all shadow registers
    TIMx->CR1 |= TIM_CR1_CEN; // enable counter
}

void configurePWM(TIM_TypeDef * TIMx, uint8_t master_mode) {
    /* Configures a timer for outputting musical tones (or more generally, PWM stuff)*/

    /* Disable Counter */
    TIMx->CR1 &= ~(TIM_CR1_CEN); // disable counter

    /* Select internal clock at maximum frequency */
    TIMx->SMCR &= ~(TIM_SMCR_SMS);  // disable slave mode controller;
                                    // use instead internal clock CK_INT (aka "TIMx_CLK from RCC")
    TIMx->PSC = 0; // do not prescale; let the counter receive full CK_INT frequency

    /* Configure Counter as Upcounter */
    TIMx->CR1 &= ~(TIM_CR1_CMS); // use edge-aligned mode (i.e. plain vanilla up or down counting)
    TIMx->CR1 &= ~(TIM_CR1_DIR); // upcounter mode

    /* Allow Automatic Updating on Overflow Events */
    TIMx->CR1 |= TIM_CR1_ARPE;  // "auto-reload preload enabled"
                                // transfers preload register to shadow register every update event, meaning
                                // we can change ARR and PSC without having to manually request an update
    TIMx->ARR = 1;  // set max count to 1 as a default so that
                    // we generate update events quickly when we turn timer on
    TIMx->CR1 &= ~(TIM_CR1_OPM); // do not stop counter at update events

    /* If Master, output trigger signal upon update events */
    if (master_mode) {
        // set update event at as trigger output (TRGO)
        TIMx->CR2 &= ~(TIM_CR2_MMS);
        TIMx->CR2 |= (0b010 << TIM_CR2_MMS_Pos);
    }

    /* Start Running */
    TIMx->EGR |= TIM_EGR_UG;  // manually generate an update to initialize all shadow registers
    TIMx->CR1 |= TIM_CR1_CEN; // enable counter
}

void generatePWMfreq(TIM_TypeDef * TIMx, uint32_t freq, uint32_t duty_inv) {
    /* Generate a signal of frequency <freq> Hz and 1/<duty_inv> duty cycle
     * This is designed for applications where frequency matters most (e.g. tone generation).
     *
     * <timer> TIM2, TIM3, TIM4, or TIM5
     * <freq> in Hz, can range from TIMER_BASE_FREQ / (ARR size) to TIMER_BASE_FREQ
     *               though it should be noted that division rounding error approaches
     *               a maximum of ~50% as freq approaches TIMER_BASE_FREQ
     * <duty_inv> is 1/duty, and it is also subjected to divison rounding error
     *               as duty approaches TIMER_BASE_FREQ / freq
     *
     * TIM2 and TIM5 have 32 bit ARR, which at 84MHz base freq, will not generate freqs below 0.196Hz
     * TIM3 and TIM4 have 16 bit ARR, which at 84MHz base freq, will not generate freqs below 1282Hz
     */

    TIMx->CR1 |= TIM_CR1_UDIS;  // Disable update events because apparently it would be bad
                                // if the timer by chance tried to update the shadow registers
                                // while we're writing new values in the preload registers.
    if (freq != 0) {
        uint32_t num_ticks = 84000000UL/freq;
                                 // Both frequencies are in Hz, so the ratio
                                 // represents number of periods of base freq per period of tone freq.
                                 // Integer division shouldn't plague us with too much rounding error
                                 // if TIMER_BASE_FREQ is sufficiently large relative to max freq,

        TIMx->ARR  = num_ticks;          // Set value we count up to.
        TIMx->CCR1 = num_ticks/duty_inv; // Set the count threshold at which capture/compare channel 1 output is high/low.
                                         // Dividing by duty_inv is what implements the duty cycle
    } else {
        TIMx->ARR = 1;  // set to 1 to that it's constantly updating
        TIMx->CCR1 = 0; // set output to resting state
    }

    TIMx->CR1 &= ~(TIM_CR1_UDIS); // enable update events;
                                  // (i.e. send update signal every time counter under/over-flows)
}

inline void start_micros(TIM_TypeDef * TIMx, uint32_t us) {
    TIMx->ARR = us;               // Set timer max count
    TIMx->EGR |= TIM_EGR_UG;      // Force update
    TIMx->SR &= ~(TIM_SR_UIF);    // Clear UIF
    TIMx->CNT = 0;                // Reset count
}

inline void wait_micros(TIM_TypeDef * TIMx) {
    while(!(TIMx->SR & TIM_SR_UIF)); // Wait for UIF to go high
}

void delay_millis(TIM_TypeDef * TIMx, uint32_t ms) {
  start_micros(TIMx, ms*1000);
  wait_micros(TIMx);
}

void delay_micros(TIM_TypeDef * TIMx, uint32_t us) {
  start_micros(TIMx, us);
  wait_micros(TIMx);
}