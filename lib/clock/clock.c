#include "clock.h"

void configureFlash() {
    // Set to 2 waitstates
    FLASH->ACR &= ~(FLASH_ACR_LATENCY);
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;

    // Turn on the ART
    FLASH->ACR |= FLASH_ACR_PRFTEN;
}

void configure84MHzPLL() {
    /*
     Set clock to 84 MHz
     Output freq = (src_clk) * (N/M) / P
          84 MHz = (8 MHz) * (336/8) / 4
     M:8, N:336, P:4
     Use HSE as src_clk; on the Nucleo, it is connected to 8 MHz ST-Link clock
    */

    RCC->CR &= ~(RCC_CR_PLLON); // Turn off PLL
    RCC->CR &= ~(RCC_CR_PLLI2SON); // Turn off the I2S PLL too (it shares M and src_clk)
    while (RCC->CR & RCC_CR_PLLRDY); // Wait till PLL is unlocked (e.g., off)

    // Select HSE as src_clk
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;

    // Set M
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
    RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);

    // Set N
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
    RCC->PLLCFGR |= (336 << RCC_PLLCFGR_PLLN_Pos);

    // Set P (yes, 0b01 is interpreted to mean a factor of 4)
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
    RCC->PLLCFGR |= (0b01 << RCC_PLLCFGR_PLLP_Pos);

    RCC->CR |= RCC_CR_PLLON; // Turn on PLL
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait till PLL is locked
}

void configure84MHzClock() {
    /* Sets system clock to 84 MHz from the PLL which is fed 8 MHz from HSE */

    configureFlash(); // configure flash to support the higher clock speed

    // Turn on and bypass for HSE from ST-LINK
    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    // Configure and turn on PLL
    // Note that this will have the side effects of turning off the I2S PLL
    // and possibly changing its input configuration.
    configure84MHzPLL();

    // Select PLL as clock source
    RCC->CFGR &= ~(RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));

    // Set AHB (system clock) prescalar to 0 so we get full speed!
    RCC->CFGR &= ~(RCC_CFGR_HPRE);
    // Set APB2 (high-speed bus) prescaler to no division
    // (this will let our clocks receive the full SYSCLK freq)
    RCC->CFGR &= ~(RCC_CFGR_PPRE2);
    // Set APB1 (low-speed bus) to divide by 2 (because APB1 should not exceed 42 MHz)
    RCC->CFGR &= ~(RCC_CFGR_PPRE1);
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    // Note that clocks on APB1 will still get full 84 MHz if APB1 at 42 MHz
}