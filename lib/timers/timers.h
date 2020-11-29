#ifndef __TIMERS_H__
#define __TIMERS_H__

#include "stm32f4xx.h"

void configureTimer(TIM_TypeDef * TIMx);
void configureCaptureCompare(TIM_TypeDef * TIMx);

void configureDuration(TIM_TypeDef * TIMx, uint32_t prescale, uint8_t slave_mode, uint8_t master_src);
void generateDuration(TIM_TypeDef * TIMx, uint32_t duration, uint32_t compare_val);

void configurePWM(TIM_TypeDef * TIMx, uint8_t master_mode);
void generatePWMfreq(TIM_TypeDef * TIMx, uint32_t freq, uint32_t duty_inv);

void start_micros(TIM_TypeDef * TIMx, uint32_t us);
void wait_micros(TIM_TypeDef * TIMx);
void delay_millis(TIM_TypeDef * TIMx, uint32_t ms);
void delay_micros(TIM_TypeDef * TIMx, uint32_t us);

#endif