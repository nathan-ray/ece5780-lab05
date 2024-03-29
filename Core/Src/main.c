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
char read();
void stop();
void write(char val);
char writeToRegister(char addr, char value);
int16_t readXAxis();
int16_t readYAxis();

/*
 * Writes the val onto TSIX.
 *
 */
void write(char val) {
	//clear SADD and NBYTES
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// set to write
	I2C2->CR2 &= ~(1 << 10);
	I2C2->CR2 |= (0x69 << 1) | (1 << 16);
	I2C2->CR2 |= I2C_CR2_START;
	
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return;
		}
		
		if (I2C2->ISR & I2C_ISR_TXIS) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
	
	// set register of CTRL_REG1
	I2C2->TXDR = val;
	
	// wait until TC is set
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return;
		}
		
		if (I2C2->ISR & I2C_ISR_TC) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
}

/* 
 * Returns the byte from RXDR.
 */
char read() {
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
	// wait until RXNE is set
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_RXNE) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
	
	char val = I2C2->RXDR;
	
	// wait until TC is set
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_TC) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}

	}
	return val;
}

/*
 * Stops the I2C2.
 */
void stop() {
	I2C2->CR2 |= (1 << 14);	// STOP I2C2
}

/*
 * Reads the x-axis from the gyroscope.
 */
int16_t readXAxis() {
	int16_t xAxis = 0;
	write(0xA8);
	stop();
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (2 << 16) | I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
	
	// wait for first 8-bit data
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_RXNE) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
	
	xAxis = I2C2->RXDR;
	
	// wait for second 8-bit data
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_RXNE) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}

	xAxis |= (I2C2->RXDR << 8);
	
	// wait until TC is set
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_TC) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}

	}
	return xAxis;
}

/*
 * Reads the y-axis from the gyroscope.
 */
int16_t readYAxis() {
	int16_t yAxis = 0;
	write(0xAA);
	stop();
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 = (0x69 << 1) | (2 << 16) | I2C_CR2_RD_WRN;
	I2C2->CR2 |= I2C_CR2_START;
	
	// wait for first 8-bit data
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_RXNE) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
	
	yAxis = I2C2->RXDR;
	
	// wait for second 8-bit data
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_RXNE) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}

	yAxis |= (I2C2->RXDR << 8);
	
	// wait until TC is set
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_TC) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}

	}
	return yAxis;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	
	// GREEN  -> 9
	// ORANGE -> 8
	// BLUE		-> 7
	// RED		-> 6
	// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6, PC7, PC8 & PC9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // Start PC9 reset
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // Start PC8 reset
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Start PC7 reset
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Start PC6 reset
	
	// RCC enable for GPIOB and GPIOC
	RCC->AHBENR |= (1 << 18); // B
	RCC->AHBENR |= (1 << 19); // C
	
	// RCC enable for I2C2
	RCC->APB1ENR |= (1 << 22);
	
	// Set PB11 & PB13 to AF Mode, PB14 to ouput
  GPIOB->MODER  |= (1 << 23) | (1 << 27) | (1 << 28);
	// Set PB11 & PB13 to open-drain
  GPIOB->OTYPER |= (1 << 11) | (1 << 13);
	// Set pull-up on PB11 & PB13
  GPIOB->PUPDR  |= (1 << 22) | (1 << 26);
	// Set AF1 on PB11 as I2C2_SDA & AF5 on PB13 as I2C2_SCL
  GPIOB->AFR[1] = 0x00501000;
	// Set PB14 high
  GPIOB->BSRR = (1 << 14);
	// Set PC0 high
	GPIOC->BSRR = (1 << 0);    
	
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
	// SCLH to 0x0F or 0'b00001111
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
	I2C2->CR1 |= I2C_CR1_PE;
	 
	/* I2C initialization
	1. Set the slave address in the SADD[7:1] bit field.
	2. Set the number of data byte to be transmitted in the NBYTES[7:0] bit field.
	3. Configure the RD_WRN to indicate a read/write operation.
	4. Do not set the AUTOEND bit, this lab requires software start/stop operation.
	5. Setting the START bit to begin the address frame
	*/
  //I2C2->CR2 = (0x69 << 1) | (1 << 16);

  // Set the START bit to begin the address frame
  //I2C2->CR2 |= I2C_CR2_START;
	
	/*
	// wait until TXIS is set
	while (1) {
		if (I2C2->ISR & I2C_ISR_TXIS) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		}
	}
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // turn on red led when TXIS is set
	// set register of WHO_AM_I to TXDR which is 0x0F or 0'b00001111
	I2C2->TXDR |= 0x0F;			
	
	// wait until TC is set
	while (1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // turn on blue led when TXIS is set
	
	// reload CR2 but set to read
	I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;

	// wait until RXNE is set
	while (1) {
		if (I2C2->ISR & I2C_ISR_RXNE) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		}
	}
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // turn on green led
	
	// wait until TC is set
	while (1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		}
	}
	
	// check if RXDR matches with 0xD3
  if (I2C2->RXDR != 0xD3)
  {
		// TURN OFF ALL LEDS IF RXDR DON'T MATCH
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  }
	else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // turn on orange led
		I2C2->CR2 |= (1 << 14);	// STOP I2C2
	}
	*/
	
	// ##############################################################
	// WRITING 0x0B to CTRL_REG1
	// ##############################################################
	//clear SADD and NBYTES
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	// set to write
	I2C2->CR2 &= ~(1 << 10);
	I2C2->CR2 |= (0x69 << 1) | (2 << 16);
	I2C2->CR2 |= I2C_CR2_START;
	
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_TXIS) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
	
	I2C2->TXDR = 0x20;
	
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_TXIS) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
	
	I2C2->TXDR = 0x0B;
	
	// wait until TC is set
	while (1) {
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		
		if (I2C2->ISR & I2C_ISR_TC) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			break;
		}
	}
	
	// read CTRL_REG1 to make sure data is set correctly
	write(0x20);
	if (read() != 0x0b) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		return 0;
	}
	
	// ##############################################################
	// SETUP LEDS
	// ##############################################################
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	// ##############################################################
	// READING FROM GYROSCOPE
	// ##############################################################
	
	int16_t xAxis = 0;
	int16_t yAxis = 0;
	const int16_t threshold = 0x00FF;
	while (1) {
		xAxis = readXAxis();
		yAxis = readYAxis();
		
		if (xAxis > threshold) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		
		if (yAxis < 0 - threshold) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		
		if (xAxis < 0 - threshold) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		
		if (yAxis > threshold) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		}
		
		HAL_Delay(100);
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
