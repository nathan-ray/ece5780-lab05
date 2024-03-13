/**
  ******************************************************************************
	* @name						: Nathaniel Ray Raharjo | u1360092
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	
	// ORANGE -> 9
	// GREEN  -> 8
	// BLUE		-> 7
	// RED		-> 6
	// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6, PC7, PC8 & PC9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Start PC9 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // Start PC8 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Start PC7 high
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Start PC6 high
	
	// RCC enable for GPIOB and GPIOC
	RCC->AHBENR |= (1 << 18); // B
	RCC->AHBENR |= (1 << 19); // C
	
	// RCC enable for I2C2
	RCC->APB1ENR |= (1 << 22);
	
	// GPIO PB11 to AF
	GPIOB->MODER |= (1 << 23);
	GPIOB->MODER &= ~(1 << 22);
	
	// GPIO PB11 to open drain
	GPIOB->OTYPER |= (1 << 11);
	
	// GPIO PB11 to I2C2_SDA AF1 (0001)
	GPIOB->AFR[1] &= ~(1 << 15);
	GPIOB->AFR[1] &= ~(1 << 14);
	GPIOB->AFR[1] &= ~(1 << 13);
	GPIOB->AFR[1] |= (1 << 12);
	
	// GPIO PB13 to AF
	GPIOB->MODER |= (1 << 27);
	GPIOB->MODER &= ~(1 << 26);
	
	// GPIO PB13 to open drain
	GPIOB->OTYPER |= (1 << 13);
	
	// GPIO PB13 to I2C2_SCL AF5 (0101)
	GPIOB->AFR[1] &= ~(1 << 23);
	GPIOB->AFR[1] |= (1 << 22);
	GPIOB->AFR[1] &= ~(1 << 21);
	GPIOB->AFR[1] |= (1 << 20);
	
	// GPIO PB14 to output mode
	GPIOB->MODER &= ~(1 << 29);
	GPIOB->MODER |= (1 << 28);
	
	// GPIO PB14 push pull
	GPIOB->OTYPER &= ~(1 << 14);
	
	// GPIO PB14 init high
	GPIOB->ODR |= (1 << 14);
	
	// GPIO PC0 to output mode
	GPIOC->MODER &= ~(1 << 1);
	GPIOC->MODER |= (1 << 0);
	
	// GPIO PC0 to push pull
	GPIOC->OTYPER &= ~(1 << 0);
	
	// GPIO PC0 init high
	GPIOC->ODR |= (1 << 0);
	
	// TIMINGR to 100 kHz
	// PRESC to 1
	I2C2->TIMINGR &= ~(1 << 31);
	I2C2->TIMINGR &= ~(1 << 30);
	I2C2->TIMINGR &= ~(1 << 29);
	I2C2->TIMINGR |= (1 << 28);
	// SCLDEL to 0x4 or 0'b0100
	I2C2->TIMINGR &= ~(1 << 23);
	I2C2->TIMINGR |= (1 << 22);
	I2C2->TIMINGR &= ~(1 << 21);
	I2C2->TIMINGR &= ~(1 << 20);
	// SDADEL to 0x2 or 0'b0010
	I2C2->TIMINGR &= ~(1 << 19);
	I2C2->TIMINGR &= ~(1 << 18);
	I2C2->TIMINGR |= (1 << 17);
	I2C2->TIMINGR &= ~(1 << 16);
	// SCLH to 0x0F or 0'b1111
	I2C2->TIMINGR &= ~(1 << 15);
	I2C2->TIMINGR &= ~(1 << 14);
	I2C2->TIMINGR &= ~(1 << 13);
	I2C2->TIMINGR &= ~(1 << 12);
	I2C2->TIMINGR |= (1 << 11);
	I2C2->TIMINGR |= (1 << 10);
	I2C2->TIMINGR |= (1 << 9);
	I2C2->TIMINGR |= (1 << 8);
	// SCLL to 0x13 or 0'00010011
	I2C2->TIMINGR &= ~(1 << 7);
	I2C2->TIMINGR &= ~(1 << 6);
	I2C2->TIMINGR &= ~(1 << 5);
	I2C2->TIMINGR |= (1 << 4);
	I2C2->TIMINGR &= ~(1 << 3);
	I2C2->TIMINGR &= ~(1 << 2);
	I2C2->TIMINGR |= (1 << 1);
	I2C2->TIMINGR |= (1 << 0);
	
	// PE
	I2C2->CR1 |= (1 << 0);
	
	/* I2C initialization
	1. Set the slave address in the SADD[7:1] bit field.
	2. Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
	3. Configure the RD_WRN to indicate a read/write operation.
	4. Do not set the AUTOEND bit, this lab requires software start/stop operation.
	5. Setting the START bit to begin the address frame
	*/
	// SET ADD10 to 0
	I2C2->CR2 &= ~(1 << 11);
	// SADD to 0x69 0'b1101001 7 bits
	/*I2C2->CR2 |= (1 << 9); DONT CARES
	I2C2->CR2 |= (1 << 8);*/
	I2C2->CR2 |= (1 << 7); 
	I2C2->CR2 |= (1 << 6);
	I2C2->CR2 &= ~(1 << 5);
	I2C2->CR2 |= (1 << 4);
	I2C2->CR2 &= ~(1 << 3);
	I2C2->CR2 &= ~(1 << 2);
	I2C2->CR2 |= (1 << 1);
	/* I2C2->CR2 |= (1 << 0); DONT CARE */
	// NBYTES to 0x1 or 0'b1
	I2C2->CR2 &= ~(1 << 23);
	I2C2->CR2 &= ~(1 << 22);
	I2C2->CR2 &= ~(1 << 21);
	I2C2->CR2 &= ~(1 << 20);
	I2C2->CR2 &= ~(1 << 19);
	I2C2->CR2 &= ~(1 << 18);
	I2C2->CR2 &= ~(1 << 17);
	I2C2->CR2 |= (1 << 16);
	// RD_WRN to write
	I2C2->CR2 &= ~(1 << 10);
	// START
	I2C2->CR2 |= (1 << 13);
	
	int doItOnce = 0;
	
  while (1)
  {
		if (doItOnce == 1) continue;
		
		// wait until TXIS(1) is set NACKF(4)
		while ( !(I2C2->ISR & 2) ) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			if (I2C2->ISR & 8) {	// error if this is true
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // turn on red led
				doItOnce = 1;
				continue;
			}
			HAL_Delay(50);
		}
		
		if ((I2C2->ISR & 2) && !(I2C2->ISR & 8)) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // turn on red led
			// set register of WHO_AM_I to TXDR which is 0x0F or 0'b00001111
			I2C2->TXDR &= ~(1 << 7);
			I2C2->TXDR &= ~(1 << 6);
			I2C2->TXDR &= ~(1 << 5);
			I2C2->TXDR &= ~(1 << 4);
			I2C2->TXDR |= (1 << 3);
			I2C2->TXDR |= (1 << 2);
			I2C2->TXDR |= (1 << 1);
			I2C2->TXDR |= (1 << 0);
		}
		
		// wait until TC is set
		while ( !(I2C2->ISR & 64) ) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);	// toggle blue LED
			HAL_Delay(50);
		}
		
		if (I2C2->ISR & 64) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // turn on blue led
			// reload CR2 but set to read
			// SET ADD10 to 0
			I2C2->CR2 &= ~(1 << 11);
			// SADD to 0x69 0'b1101001 7 bits
			/*I2C2->CR2 |= (1 << 9); DONT CARES
			I2C2->CR2 |= (1 << 8);*/
			I2C2->CR2 |= (1 << 7); 
			I2C2->CR2 |= (1 << 6);
			I2C2->CR2 &= ~(1 << 5);
			I2C2->CR2 |= (1 << 4);
			I2C2->CR2 &= ~(1 << 3);
			I2C2->CR2 &= ~(1 << 2);
			I2C2->CR2 |= (1 << 1);
			/* I2C2->CR2 |= (1 << 0); DONT CARE */
			// NBYTES to 0x1 or 0'b1
			I2C2->CR2 &= ~(1 << 23);
			I2C2->CR2 &= ~(1 << 22);
			I2C2->CR2 &= ~(1 << 21);
			I2C2->CR2 &= ~(1 << 20);
			I2C2->CR2 &= ~(1 << 19);
			I2C2->CR2 &= ~(1 << 18);
			I2C2->CR2 &= ~(1 << 17);
			I2C2->CR2 |= (1 << 16);
			// RD_WRN to read
			I2C2->CR2 |= (1 << 10);
			// START
			I2C2->CR2 |= (1 << 13);
		}
		
		// wait until RXNE is set
		while ( !(I2C2->ISR & 4) ) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);	// toggle green LED
			if (I2C2->ISR & 8) {	// error if this is true
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // turn on red led
				doItOnce = 1;
				continue;
			}
			HAL_Delay(50);
		}
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // turn on green led
		
		// wait until TC is set
		while ( !(I2C2->ISR & 64) ) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);	// toggle orange LED
			HAL_Delay(50);
		}
		
		// check if RXDR matches with 0xD3 or 211
		if (I2C2->RXDR == 211) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // turn on orange led
			I2C2->CR2 |= (1 << 14);	// STOP I2C2
			doItOnce = 1;
		}
			
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
