#include "stm32f4xx.h"

#define PLL__M 						(8 << 0)
#define PLL__N 						(360 << 6)
#define PLL__P 						(((2 >> 1) -1) << 16)
#define SYS_TICK_1_MS			(180000)

volatile unsigned int time_ms = 0;
volatile unsigned int duty_cycle  = 0;
volatile uint8_t spi_dr;
volatile int flag = 0;

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
																								
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;							// APB2 = 90Mhz
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void mySysTickConfig()
{
	SysTick->LOAD  = SYS_TICK_1_MS;                   								// set reload register
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); // set Priority for Systick Interrupt
  SysTick->VAL   = 0UL;                                             // load the SysTick Counter Value
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;  
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void RCC_Configuration()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;	// GPIOC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;	// GPIOF
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;	// GPIOG
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;		// TIM4
	RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;		// SPI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	// SYSCFG
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void GPIO_Configuration()
{
	/* PA0: input, pull-down mode */
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;	
	
	/* PB6: alternate function */
	GPIOB->MODER |= GPIO_MODER_MODER6_1;	
	/* PB6: AF2 */
	GPIOB->AFR[0] |= 0x02000000;
	
	/* PG13: general purpose output mode */
	GPIOG->MODER |= GPIO_MODER_MODER13_0;
	
	/* PF7, PF8, PF9: SPI5 SCK, MISO, MOSI (configure the alternate functions and high speed) */
	GPIOF->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
	GPIOF->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR8_1 | GPIO_OSPEEDER_OSPEEDR9_1;		
	/* Select AF5 (SPI) */
	GPIOF->AFR[0] = 0x50000000;	//PF7
	GPIOF->AFR[1] = 0x00000055;	//PF8, PF9
	/* Set CS (PC1) */
	GPIOC->MODER |= GPIO_MODER_MODER1_0;
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void EXTI_Configuration()
{
	/* Select the source input for the EXTI0 external interrupt (PA0) */
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	/* Unmask an interrupt request from line (PA)0 */
	EXTI->IMR |= EXTI_IMR_MR0;
	
	/* Rising trigger enabled (for Event and Interrupt) for input line */
	EXTI->RTSR |= EXTI_RTSR_TR0;
	
	/* Enable NVIC interrupt request */
	NVIC_EnableIRQ(EXTI0_IRQn);
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void TIM_Configuration()
{
	/* TIM4 configuration     */
	/* Set the TIM4 prescaler */
	/* 45MHz / 4500 = 10kHz 	*/
	TIM4->PSC = 45 - 1;
	TIM4->ARR = 10000 - 1; 			// determines the frequency		
	TIM4->CCR1 = 0 ;			
	TIM4->CCMR1 |= (TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_2);
	TIM4->CCMR1 |= TIM_CCMR1_IC1PSC_1;
	TIM4->CR1 |= TIM_CR1_ARPE;
	TIM4->EGR |= TIM_EGR_UG;
	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CR1 |= TIM_CR1_CEN;
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void SPI_Configuration()
{
	SPI5->CR1 |= SPI_CR1_BR_2;									// step 1
	SPI5->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;		// step 2
																							// step 3 - 8 bits (by default)
																		          // step 4 - MSB first (by default)
  SPI5->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;			// step 5
																							// step 6 - frame format: SPI Motorola (by default)
  SPI5->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;		// step 7 

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
void delay_ms(unsigned int time_delay)
{
	time_ms = time_delay;
	while(time_ms){
	}
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void SysTick_Handler()
{
	if (time_ms != 0)
		time_ms--;
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void EXTI0_IRQHandler()
{
	if (GPIOA->IDR & GPIO_IDR_IDR_0) {
		delay_ms(50);
		if (GPIOA->IDR & GPIO_IDR_IDR_0) {
			if ((TIM4->CCR1) <= 5000) {
				duty_cycle += 250;
				TIM4->CCR1 = duty_cycle;
			} else {
				duty_cycle = 0;
				TIM4->CCR1 = duty_cycle;
			}
		}
	}
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
uint8_t L3GD20_sendData_SPI(uint8_t address){

	/* Send data to L3GD20 (select the internal address) */
	if (address == 0x00)
		SPI5->DR = address;							// only for receiving purposes
	else
		SPI5->DR = (address | 0x80);		// select the internal address
	
	while(!(SPI5->SR & SPI_SR_RXNE)); // wait for the received data to be ready to read
	
	return SPI5->DR;									// this clears also the RXNE flag
}
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
int main()
{
	myClockConfig(); 		// AHB  = 180 MHz
											// APB1 = 45 MHz
	mySysTickConfig();	// SysTick = 1ms
	RCC_Configuration();
	//TIM_Configuration();
	GPIO_Configuration();
	//EXTI_Configuration();
	SPI_Configuration();
		
	GPIOC->ODR |= GPIO_ODR_ODR_1;  // CS high (PC1)
	
	while(1)
	{
		GPIOC->ODR &= ~GPIO_ODR_ODR_1; 			// CS low (PC1)
		L3GD20_sendData_SPI(0x0F);  				// send data request to the motion sensor by selecting its internal address
		spi_dr = L3GD20_sendData_SPI(0x00); // send this byte just to read the previous requested value
    GPIOC->ODR |= GPIO_ODR_ODR_1;  			// CS high (PC1)
	}
}

