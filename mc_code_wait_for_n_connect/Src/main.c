/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */



#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>







///******************************************************************************
// * @file:    core_cm3.h
// * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Header File
// * @version: V1.20
// * @date:    22. May 2009
// *----------------------------------------------------------------------------
// *
// * Copyright (C) 2009 ARM Limited. All rights reserved.
// *
// * ARM Limited (ARM) is supplying this software for use with Cortex-Mx
// * processor based microcontrollers.  This file can be freely distributed
// * within development tools that are supporting such ARM based processors.
// *
// * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// *
// ******************************************************************************/
//
///* memory mapping struct for System Control Block */
//typedef struct
//{
//  __I  uint32_t CPUID;                        /*!< CPU ID Base Register                                     */
//  __IO uint32_t ICSR;                         /*!< Interrupt Control State Register                         */
//  __IO uint32_t VTOR;                         /*!< Vector Table Offset Register                             */
//  __IO uint32_t AIRCR;                        /*!< Application Interrupt / Reset Control Register           */
//  __IO uint32_t SCR;                          /*!< System Control Register                                  */
//  __IO uint32_t CCR;                          /*!< Configuration Control Register                           */
//  __IO uint8_t  SHP[12];                      /*!< System Handlers Priority Registers (4-7, 8-11, 12-15)    */
//  __IO uint32_t SHCSR;                        /*!< System Handler Control and State Register                */
//  __IO uint32_t CFSR;                         /*!< Configurable Fault Status Register                       */
//  __IO uint32_t HFSR;                         /*!< Hard Fault Status Register                                       */
//  __IO uint32_t DFSR;                         /*!< Debug Fault Status Register                                          */
//  __IO uint32_t MMFAR;                        /*!< Mem Manage Address Register                                  */
//  __IO uint32_t BFAR;                         /*!< Bus Fault Address Register                                   */
//  __IO uint32_t AFSR;                         /*!< Auxiliary Fault Status Register                              */
//  __I  uint32_t PFR[2];                       /*!< Processor Feature Register                               */
//  __I  uint32_t DFR;                          /*!< Debug Feature Register                                   */
//  __I  uint32_t ADR;                          /*!< Auxiliary Feature Register                               */
//  __I  uint32_t MMFR[4];                      /*!< Memory Model Feature Register                            */
//  __I  uint32_t ISAR[5];                      /*!< ISA Feature Register                                     */
//} SCB_Type;
//
//#define SCS_BASE            (0xE000E000)                              /*!< System Control Space Base Address    */
//#define SCB_BASE            (SCS_BASE +  0x0D00)                      /*!< System Control Block Base Address    */
//#define SCB                 ((SCB_Type *)           SCB_BASE)         /*!< SCB configuration struct             */
//
//#define NVIC_AIRCR_VECTKEY    (0x5FA << 16)   /*!< AIRCR Key for write access   */
//#define NVIC_SYSRESETREQ            2         /*!< System Reset Request         */
//
///* ##################################    Reset function  ############################################ */
///**
// * @brief  Initiate a system reset request.
// *
// * @param   none
// * @return  none
// *
// * Initialize a system reset request to reset the MCU
// */
//static __INLINE void NVIC_SystemReset(void)
//{
//  SCB->AIRCR  = (NVIC_AIRCR_VECTKEY | (SCB->AIRCR & (0x700)) | (1<<NVIC_SYSRESETREQ)); /* Keep priority group unchanged */
//  __DSB();                                                                                 /* Ensure completion of memory access */
//  while(1);                                                                                /* wait until reset */
//}








void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    }
}

void my_printf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}





bool nunchuck_connected(uint32_t xferOptions, uint32_t error_code)
{
	if (xferOptions == 0 || error_code != 0)
		return false;
	else
		return true;
}



#define NUNCHUCK_ADDRESS  0xA4
#define NUNCHUCK_REGADDR  0x40

int main(void)
{


//	SCB->AIRCR = 0x05fa0004;    // System RESET!!

//	NVIC_SystemReset();

	uint16_t year = 2015;
	uint8_t month = 12;
	uint8_t day   = 18;
	char* date = "date";

	// "Today's date: 2015-12-18"
	my_printf("Today's %s: %d-%d-%d\r\n", date, year, month, day);

	uint8_t msg1[] = "test print";
	HAL_UART_Transmit(&huart2, msg1, strlen((char*)msg1), 5) ;



	  /* USER CODE BEGIN 1 */
	//	sch_power_up ();
	//
	  /* USER CODE END 1 */

	  /* MCU Configuration--------------------------------------------------------*/

	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	  HAL_Init();

	  /* USER CODE BEGIN Init */
	//
	  /* USER CODE END Init */

	  /* Configure the system clock */
	  SystemClock_Config();

	  /* USER CODE BEGIN SysInit */
	//
	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_USART2_UART_Init();
//	  MX_ADC1_Init();
	  MX_I2C1_Init();
	  /* USER CODE BEGIN 2 */
	//
	//	sch_init ();
	//
	//
	//	lab1_init();
	//
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */


	  //set mode of nunchuck
	//  const uint8_t buf1[] = {0x40};
	//  const uint8_t buf2[] = {0x00};
	//
	//  HAL_I2C_Master_Transmit(I2C1, NUNCHUCK_ADDRESS, buf1, 1, 100);
	//  HAL_I2C_Master_Transmit(I2C1, NUNCHUCK_ADDRESS, buf2, 1, 100);




	  while(nunchuck_connected(hi2c1.XferOptions, hi2c1.ErrorCode) == false)
	  {
		  //set mode of nunchuck
		  uint8_t init_data[6];
		  init_data[0] = 0xF0;
		  init_data[1] = 0x55;
		//  twi_send_msg(&TWID, NUNCHUCK, &init_data[0], 2);
		  HAL_I2C_Master_Transmit(&hi2c1, NUNCHUCK_ADDRESS, init_data, 2, 100);
		  init_data[0] = 0xFB;
		  init_data[1] = 0x00;
		  HAL_I2C_Master_Transmit(&hi2c1, NUNCHUCK_ADDRESS, init_data, 2, 100);
		//  twi_send_msg(&TWID, NUNCHUCK, &init_data[0], 2);
	//	  init_data[0] = 0x00;
	//	  HAL_I2C_Master_Transmit(&hi2c1, NUNCHUCK_ADDRESS, init_data, 1, 100);



		  wait(10);

	//	  unsigned char cmd[] = {NUNCHUCK_REGADDR, 0x00};
	//	  uint8_t read_data[6] = {0,0,0,0,0,0};
	//	  HAL_I2C_Master_Transmit(&hi2c1, NUNCHUCK_REGADDR, cmd, 1, 100);
	//	  HAL_I2C_Master_Receive (&hi2c1, NUNCHUCK_REGADDR, read_data, 6, 100);
	//
	//	  wait(10);
	  }





	  //read from nunchuck
	  uint8_t data[6];
	  uint8_t buf[] = {0};

	//  HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	//  HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

	  bool connected;
	  volatile uint32_t c = 5;

	  while(1)
	  {
		  if (true)
			  connected = 1;
		  else
			  connected = 1;

		  c = hi2c1.XferOptions;
		  uint8_t msg1[] = "ABCCC";
		  HAL_UART_Transmit(&huart2, msg1, strlen((char*)msg1), 5) ;
		  HAL_I2C_Master_Transmit(&hi2c1, NUNCHUCK_ADDRESS, buf, 1, 100);
		  HAL_I2C_Master_Receive (&hi2c1, NUNCHUCK_ADDRESS, data, 6, 100);


	  }


	//  I2C_Write(I2C1, buf, 1, NUNCHUK_ADDRESS);
	//  I2C_Read(I2C1, data, 6, NUNCHUK_ADDRESS);


	  while (1)
	  {
		  uint8_t msg1[] = "ABCCC";
		  HAL_UART_Transmit(&huart2, msg1, strlen((char*)msg1), 5) ;
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
