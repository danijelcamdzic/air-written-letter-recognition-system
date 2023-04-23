/**
 * @file peripheral_functions.c
 *
 * This file contains the definitions for the functions
 * that involve working with peripherals
 *
 *  Version: 2.3
 *  Created on: June 25th, 2020
 *      Author: Danijel Camdzic
 */
#include "peripheral_functions.h"

/* Function Definitions */
/* Initializations function and I2C protocol functions*/
/**
 * @brief Clock_init
 *
 * Function that intializes the clock frequency and the source of clock for the STM32F030R8 microncontroller.
 *
 */
void Clock_Init(void)
{
	/* Use PLL as Clock Source and Mul by 12 */
	RCC->CR &= ~(RCC_CR_PLLON); /**< Turn off PLL */
	while (RCC->CR & RCC_CR_PLLRDY)
		;							/**< Wait until PLL is ready */
	RCC->CFGR |= RCC_CFGR_PLLMUL12; /**< Multiply PLL value by 12. Must not exceed 48 Mhz (Max frequency)! */
	RCC->CR |= RCC_CR_PLLON;		/**< Turn on PLL */
	while (!(RCC->CR & RCC_CR_PLLRDY))
		; /**< Wait until PLL is ready in order to avoid messing stuff up */

	FLASH->ACR |= FLASH_ACR_LATENCY;  /**< Add ONE wait state for the flash acces time because 24 Mhz <= SYSCLOCK <= 48MHz */
	RCC->CFGR &= ~(RCC_CFGR_PPRE);	  /**< Clear PCLK prescaler settings for safety */
	RCC->CFGR |= RCC_CFGR_PPRE_DIV16; /**< HCLK divided by 16 */
	RCC->CFGR &= ~(RCC_CFGR_HPRE);	  /**< Clear HCLK prescaler settings */
	while (!(RCC->CR & RCC_CR_PLLRDY))
		;						   /**< Wait until PLL is ready */
	RCC->CFGR &= ~(RCC_CFGR_SW);   /**< Clear System clcok switch settings */
	RCC->CFGR |= RCC_CFGR_SW_PLL;  /**< Select PLL as source */
	RCC->CFGR &= ~(RCC_CFGR_PPRE); /**< Clear preslacer settings set before for safety */
}

/**
 * @brief GPIO_Init
 *
 * Function that initializes the GPIO pins.
 *
 */
void GPIO_Init(void)
{
	/* Enable clock for GPIOs */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	GPIOA->MODER |= GPIO_MODER_MODER5_0; /**< Output mode (01) for bit 5 */
}

/**
 * @brief TIMER3_Init
 *
 * Function that initializes the TIMER3 peripheral with it's frequency and up count max.
 *
 */
void TIMER3_Init(void)
{
	/* Configuring Timer (TIM3) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->SR = 0;					   /**< Resetting the SR register of TIM3 */
	TIM3->PSC = (uint16_t)(24000 - 1); /**< SYSCLK/(PSC+1) = TIM3_frequency */
	TIM3->ARR = (uint16_t)1999;		   /**< Counts to 1999 and then goes to 0. */

	/* In case I need interrupts */
	// TIM3->DIER |= (1 << 0);
	// NVIC_SetPriority(TIM3_IRQn, 2); 				/**< Priority level 2 */
	// NVIC_EnableIRQ(TIM3_IRQn);
}

/**
 * @brief USART2_Init
 *
 * Function that initializes UART for the STM32F030R8 microcontroller to be used to transfer data to the PC in order
 * to visualize orientation.
 *
 */
void USART2_Init(void)
{
	/* Configuring UART (USART2) */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; /**< Enable clock for the USART2 */

	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL2 & 0x100) | (GPIO_AFRL_AFSEL3 & 0x1000);
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; /**< Alternate function for pins 2 and 3 */

	USART2->CR1 &= ~(USART_CR1_UE);				/**< Disable UART */
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; /**< Enable transmit and receive */

	USART2->BRR = (uint32_t)417; /**< 48MHz/115200 = 416.66 -> 417 */

	USART2->CR1 |= USART_CR1_UE; /**< Enable UART */
}

/**
 * @brief I2C1_Init
 *
 * Function that initializes I2C for the STM32F030R8 microcontroller to be used to communicate with the MPU-9250 sensor.
 *
 */
void I2C1_Init(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; /**< Enable clock for the GPIOB (Pins for I2C1) */

	GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL8 & 0x01) | (GPIO_AFRH_AFSEL9 & 0x10);
	GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;																				/**< Alternate function for the pins 8 and 9 */
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR8_1 | (GPIO_OSPEEDR_OSPEEDR8_1 >> 1) | GPIO_OSPEEDER_OSPEEDR9_1 | (GPIO_OSPEEDR_OSPEEDR9_1 >> 1); /**< High speed */
	GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;																					/**< Open drain type */
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;

	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; /**< Enable clock for I2C1 */

	I2C1->CR1 &= ~(I2C_CR1_PE);				  /**< Disable I2C1 peripheral */
	I2C1->TIMINGR = 0x2000090E & 0xF0FFFFFFU; /**< Adjust timing */
	I2C1->CR1 |= I2C_CR1_PE;				  /**< Enable I2C1 peripheral */
}

/**
 * @brief I2C_Read
 *
 * Function that performs the read cycle over I2C protocol to the sensor MPU-9250.
 *
 */
void I2C_Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata)
{
	/* Wait in case of busy */
	while (I2C1->ISR & I2C_ISR_BUSY)
		;

	/* Sending Device Address */
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_START |
				   I2C_CR2_STOP | I2C_CR2_SADD | I2C_CR2_RD_WRN);
	I2C1->CR2 |= (Slave_Address & I2C_CR2_SADD);
	I2C1->CR2 |= ((0x01 << 16U) & I2C_CR2_NBYTES);
	/* Sending start bit */
	I2C1->CR2 |= I2C_CR2_START;

	/* Waiting TXIS to be Set */
	while (!(I2C1->ISR & I2C_ISR_TXIS))
		;

	/* Sending Register Address */
	I2C1->TXDR = Register_Address;

	/* Waiting TC to be Set */
	while (!(I2C1->ISR & I2C_ISR_TC))
		;

	/* Sending Device Address and Restart */
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_START | I2C_CR2_STOP | I2C_CR2_RD_WRN);
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 |= I2C_CR2_RD_WRN;
	I2C1->CR2 |= I2C_CR2_START;

	/* Waiting RXNE to be Set */
	while (!(I2C1->ISR & I2C_ISR_RXNE))
		;

	/* Reading data from the receive register */
	*pdata = I2C1->RXDR;

	/* Waiting for STOPF to detect STOP bit */
	while (!(I2C1->ISR & I2C_ISR_STOPF))
		;

	/* Clear STOP bit */
	I2C1->ICR |= I2C_ICR_STOPCF;

	/* Clear configuration */
	I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);
}

/**
 * @brief I2C_Write
 *
 * Function that performs the write cycle over I2C protocol to the sensor MPU-9250.
 *
 */
void I2C_Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata)
{
	/* Wait in case of busy */
	while (I2C1->ISR & I2C_ISR_BUSY)
		;

	/* Sending Device Address */
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_START |
				   I2C_CR2_STOP | I2C_CR2_SADD | I2C_CR2_RD_WRN);
	I2C1->CR2 |= (Slave_Address & I2C_CR2_SADD);
	I2C1->CR2 |= ((0x01 << 16U) & I2C_CR2_NBYTES);
	I2C1->CR2 |= I2C_CR2_RELOAD;
	I2C1->CR2 |= I2C_CR2_START;

	/* Wait until TXIS is Set */
	while (!(I2C1->ISR & I2C_ISR_TXIS))
		;

	/* Send Memory Address */
	I2C1->TXDR = Register_Address;

	/* Wait until TCR is Set */
	while (!(I2C1->ISR & I2C_ISR_TCR))
		;

	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD);
	I2C1->CR2 |= I2C_CR2_AUTOEND;

	/* Wait until TXIS is Set */
	while (!(I2C1->ISR & I2C_ISR_TXIS))
		;
	I2C1->TXDR = *pdata;

	/* Wait until STOPF is Set and clear it */
	while (!(I2C1->ISR & I2C_ISR_STOPF))
		;
	I2C1->ICR |= I2C_ICR_STOPCF;

	/* Clear configuration */
	I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);
}

/**
 * @brief SendToUART
 *
 * Function that sends the string data over UART.
 *
 */
void SendToUART(uint8_t *pbuf)
{
	uint32_t size = strlen((char *)pbuf);
	for (uint32_t i = 0; i < size; i++)
	{
		USART2->TDR = pbuf[i]; /**< Send through the Transmit Data Register */
		while (!(USART2->ISR & USART_ISR_TC))
			; /**< Check transmission complete flag in order to continue */
	}
}

