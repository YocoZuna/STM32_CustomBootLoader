/* USER CODE BEGIN Header */
/**
  ******************************************************************************
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "TI_aes_128.h"
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
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
#define ACK        	0x79
#define NACK      	0x1F
#define CMD_ERASE 	0x43
#define CMD_GETID 	0x02
#define CMD_WRITE   0x2b
#define PROGRAMING_DONE 0x53
uint8_t AES_KEY[] = {0x40,0x23,0x3C,0x4D,0x36,0x39,0x57,0x74,0x79,0x43,0x78,0x53,0x51,0x39,0x12,0x19};
uint8_t CMDBuffor[20];

typedef struct {
	uint32_t JUMP_ADDR;
}
App_Info;
__attribute__((section(".programInfo"))) App_Info ApplicationInfo = {0x8004000};
App_Info* pApplicationInfo =  &ApplicationInfo;

void getID( const uint8_t* pData);
void eraseFlash( uint8_t* pData);
void programFlash( uint8_t* pData);




void bootApp(void)
{
	uint32_t jumpaddr = (pApplicationInfo->JUMP_ADDR);

	if (jumpaddr!=0xFFFFFFFF)
	{
		uint32_t msp = *(uint32_t*)(pApplicationInfo->JUMP_ADDR);
		//MX_GPIO_DeInit();
		SysTick->CTRL - 0x0;
		HAL_DeInit();
		RCC->CIR = 0x00000000;
		__set_MSP(msp);
		__DMB();
		SCB->VTOR = jumpaddr;
		__DSB();


		void(*rest_handler)(void) = (void*)(*(uint32_t*)(jumpaddr+4));
		rest_handler();

	}


}
void checkIfUpIsThere(void)
{
	uint32_t jumpaddr = (pApplicationInfo->JUMP_ADDR);

	if (jumpaddr!=0xFFFFFFFF)
		bootApp();
	else
	{
		while(1)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,RESET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,SET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,RESET);
			HAL_Delay(1500);
		}
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	 HAL_Init();
	 SystemClock_Config();
	  MX_GPIO_Init();
	  MX_USART2_UART_Init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_RESET)
  {


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */




  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  uint32_t delay = 1000;
  uint32_t tick= HAL_GetTick();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if((HAL_GetTick()-tick)>delay)
	  {
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		tick= HAL_GetTick();
	  }
	  HAL_UART_Receive(&huart2, CMDBuffor, 20, 20);

	  switch (CMDBuffor[0]){
	  case CMD_GETID:
		  getID(CMDBuffor);
		  break;
	  case CMD_ERASE:
		  eraseFlash(CMDBuffor);
		  break;
	  case CMD_WRITE:
		  programFlash(CMDBuffor);
		  break;
	  case PROGRAMING_DONE:
		  bootApp();
		  break;

	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	}
  }
  else{
	  checkIfUpIsThere();
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
	{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void getID( const uint8_t* pData){


	uint32_t crc = 0 ;
	uint32_t cmd = pData[0];
	memcpy(&crc,pData+1,sizeof(uint32_t));

	if (crc == HAL_CRC_Calculate(&hcrc,&cmd,1))
	{

		//Send ACK
		CMDBuffor[0] = ACK;
		HAL_UART_Transmit(&huart2,CMDBuffor,1, HAL_MAX_DELAY);
		uint16_t devID = (uint16_t)(DBGMCU->IDCODE&0xFFF);
		HAL_UART_Transmit(&huart2,(uint8_t*)&devID, 2, HAL_MAX_DELAY);

	}
	else{
		// send NACK
		CMDBuffor[0] = NACK;
		HAL_UART_Transmit(&huart2,CMDBuffor,1, HAL_MAX_DELAY);
	}

}


void eraseFlash( uint8_t* pData)
{
	/*
	 *
	* --------------------------------------------
	* | CMD_ID | number of sectors     |  CRC32  |
	* | 1 byte |     to erase          | 4 bytes |
	* |--------|-----------------------|---------|
	* |  0x43  |   N r OFF for all     |   CRC   |
	* --------------------------------------------
	 *
	 *
	 *
	 */

	uint32_t crc = 0 ;
	uint32_t cmd[2] = {pData[0],pData[1]};
	uint32_t badBlock = 0;

	memcpy(&crc,pData+2,sizeof(uint32_t));

	if (crc == HAL_CRC_Calculate(&hcrc,cmd,2))
	{
		FLASH_EraseInitTypeDef flash  ={0};
		flash.Banks = FLASH_BANK_1;
		flash.TypeErase = FLASH_TYPEERASE_SECTORS;
		flash.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		flash.Sector = FLASH_SECTOR_1;
		flash.NbSectors = cmd[1] == 0xFF ? FLASH_SECTOR_TOTAL-1 : cmd[1];
		HAL_FLASH_Unlock();
		HAL_FLASHEx_Erase(&flash, &badBlock);
		HAL_FLASH_Lock();

		if (badBlock == 0xFFFFFFFF)
		{
			//Send ACK
			CMDBuffor[0] = ACK;
			HAL_UART_Transmit(&huart2,CMDBuffor,1, HAL_MAX_DELAY);

		}
		else{
			// send NACK
			CMDBuffor[0] = NACK;
			HAL_UART_Transmit(&huart2,CMDBuffor,1, HAL_MAX_DELAY);
		}


	}
}
void programFlash( uint8_t* pData)
{
	/*
	 *
	* A write command has the following structure:
	*
	* ----------------------------------------
	* | CMD_ID | starting address  |  CRC32  |
	* | 1 byte |     4 byte        | 4 bytes |
	* |--------|-------------------|---------|
	* |  0x2b  |    0x08004000     |   CRC   |
	* ----------------------------------------
	*
	* The second message has the following structure
	*
	* ------------------------------
	* |    data bytes    |  CRC32  |
	* |      16 bytes    | 4 bytes |
	* |------------------|---------|
	* | BBBBBBBBBBBBBBBB |   CRC   |
	* ------------------------------
	 */
	static char firstEntry = 1;
	uint32_t crc = 0 ;
	uint32_t cmd[4] = {pData[0]};
	uint8_t data[20] = {0};
	memcpy(&crc,pData+5,sizeof(uint32_t));
	memcpy(&cmd[1],pData+1,sizeof(uint32_t));
	uint32_t sAddr = cmd[1];

	if (crc == HAL_CRC_Calculate(&hcrc,cmd,2))
	{

		//Send ACK
		CMDBuffor[0] = ACK;
		HAL_UART_Transmit(&huart2,CMDBuffor,1, HAL_MAX_DELAY);
		if(HAL_UART_Receive(&huart2, data, 20, 10)==HAL_TIMEOUT)
			return;
		memcpy(&crc,data+16,sizeof(uint32_t));
		memcpy(&cmd[0],data,4*(sizeof(uint32_t)));


		if (crc == HAL_CRC_Calculate(&hcrc,(uint32_t*)data,4))
		{
			HAL_FLASH_Unlock();
			if (firstEntry == 1)
			{

				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)pApplicationInfo,sAddr);
				firstEntry=0;

			}

			aes_enc_dec((uint8_t*)data,AES_KEY,1);

			for (uint8_t i=0;i<16;i++)
			{

				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, sAddr, data[i]);
				sAddr+=1;
			}
			HAL_FLASH_Lock();

		}
		//Send ACK
		CMDBuffor[0] = ACK;
		HAL_UART_Transmit(&huart2,CMDBuffor,1, HAL_MAX_DELAY);
	}
	else
	{
		// send NACK
		CMDBuffor[0] = NACK;
		HAL_UART_Transmit(&huart2,CMDBuffor,1, HAL_MAX_DELAY);
	}





}
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
	  //__io_putchar(*ptr++);
	  HAL_UART_Transmit(&huart2, (uint8_t*)ptr++, 1, 1000);
  }
  return len;
}
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
