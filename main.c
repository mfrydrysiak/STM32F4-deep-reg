#include "stm32f4xx.h"

#define PLL__M (8 << 0)
#define PLL__N (360 << 6)
#define PLL__P (((2 >> 1) -1) << 16)

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void myClockConfig()
{
	/* Turn on the HSE clock */
	RCC->CR |= RCC_CR_HSEON;
	
	/* Wait for HSE to be set */
	while((RCC->CR & RCC_CR_HSERDY) == 0);

	/* Set PLL M, N and P values */
	RCC->PLLCFGR = ( PLL__M | PLL__N | PLL__P );	
	
	/* Set HSE as the source for the PLL */
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
	
	/* Turn on the PLL */
	RCC->CR |= RCC_CR_PLLON;											
	
	/* Wait for PLL to be set */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);				
	
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
	
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;							// APB1 = 180MHz / 4 = 45 MHz
																								// (45 Mhz * 2) / 2 (because if APBx==1 then x1, otherwise x2!)
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void pushBtnLedON()
{
	if (GPIOA->IDR & GPIO_IDR_IDR_0)
		GPIOG->ODR |= GPIO_ODR_ODR_13;
	else 
		GPIOG->ODR &= ~GPIO_ODR_ODR_13;
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void EXTI0_IRQHandler()
{
	pushBtnLedON();
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
int main()
{
	myClockConfig(); 	// AHB  = 180 MHz
										// APB1 = 45 MHz
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;	// GPIOG
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;		// TIM4
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	// SYSCFG
	
	/* PA0: input, pull-down mode */
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;	
	
	/* PB6: alternate function */
	GPIOB->MODER |= GPIO_MODER_MODER6_1;	
	/* PB6: AF2 */
	GPIOB->AFR[0] |= 0x02000000;
	
	/* PG13: general purpose output mode */
	GPIOG->MODER |= GPIO_MODER_MODER13_0;

	/*----------------------------------*/
	/* External interrupt configuration */
	/* Select the source input for the EXTI0 external interrupt (PA0) */
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	/* Unmask an interrupt request from line (PA)0 */
	EXTI->IMR |= EXTI_IMR_MR0;
	
	/* Rising trigger enabled (for Event and Interrupt) for input line */
	EXTI->RTSR |= EXTI_RTSR_TR0;
	
	/* Enable NVIC interrupt request */
	NVIC_EnableIRQ(EXTI0_IRQn);
	/*----------------------------------*/
	
	/*------------------------*/
	/* TIM4 configuration     */
	/* Set the TIM4 prescaler */
	/* 45MHz / 4500 = 10kHz 	*/
	TIM4->PSC = 45 - 1;
	
	TIM4->ARR = 10000 - 1; 			// determines the frequency		
	TIM4->CCR1 = 500 ;			
	TIM4->CCMR1 |= (TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_2);
	TIM4->CCMR1 |= TIM_CCMR1_IC1PSC_1;
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CR1 |= TIM_CR1_CEN;
	/*--------------------*/

	while(1)
	{

	}
}

