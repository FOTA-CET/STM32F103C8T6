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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERCENT_ADDR 0x100
#define NUMBER_PAGE_CONFIG 9
#define NUMBER_PAGE_CURRENT 10
#define NUMBER_PAGE_OTA 69

#define SIZE_PAGE_CURRENT 59
#define SIZE_PAGE_OTA 59

#define FLASH_SIZE_HEX ((uint32_t)1024*SIZE_PAGE_CURRENT)

typedef enum {OTA, CURRENT}Region;
typedef enum {INIT, FW_SIZE, CAN_FLASH}State;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
uint32_t PROGRAM_START_CURRENT = ((uint32_t)0x08000000 + (NUMBER_PAGE_CURRENT*1024));
uint32_t PROGRAM_START_OTA = ((uint32_t)0x08000000 + (NUMBER_PAGE_OTA*1024));
uint32_t CURRENT_START_CONFIG = ((uint32_t)0x08000000 + (NUMBER_PAGE_CONFIG*1024));
uint32_t FLASH_ADDRESS;

uint32_t count=0,size_count=0, size_count_can = 1;
uint32_t size_program=0;
uint8_t flag=0;

uint16_t number_data_can = 0;
uint16_t counter = 0;
uint8_t currentPercent = -1;
uint8_t percentBuf[1] = {};
	
uint16_t application_write_idx = 0;
	
uint32_t timeout_init = 4500;
uint32_t timeout_flash = 100;
uint32_t initTime;
	
State bootloaderState = INIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Send(uint32_t id, uint8_t *data, uint8_t len);
int processPercentCAN(int firmwareSize, int count);
static void goto_application( void );
void clearFlash(Region chooseVersion);
void resetTick();
void updateCurrent();
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	FLASH_ADDRESS = PROGRAM_START_CURRENT;
	clearFlash(OTA);

	initTime = HAL_GetTick();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(bootloaderState == INIT)
		{
			if((HAL_GetTick() - initTime) >= timeout_init) goto_application();
		}
		else if(bootloaderState == FW_SIZE)
		{
			if((HAL_GetTick() - initTime) >= timeout_flash) goto_application();
		}
		else
		{
			goto_application();
		}
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	
	
		CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
		
		if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        // Notification Error
        Error_Handler();
    }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_Send(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = id;
    TxHeader.ExtId = 0x01;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

static HAL_StatusTypeDef write_data_to_flash_app( uint8_t *data, uint16_t data_len)
{
  HAL_StatusTypeDef ret;

  do
  {
    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    for(int i = 0; i < data_len/2; i++)
    {
      uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD,
                               (PROGRAM_START_OTA + application_write_idx ),
                               halfword_data
                             );
      if( ret == HAL_OK )
      {
        //update the data count
        application_write_idx += 2;
      }
      else
      {
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }
  }while( false );

  return ret;
}
		
int processPercentCAN(int firmwareSize, int count)
{
	uint8_t percent = (uint8_t)(((uint32_t)count * 100 + firmwareSize / 2) / firmwareSize);
	if(percent < 100) return percent;
	return -1;
}

void clearFlash(Region chooseVersion)
{
		HAL_FLASH_Unlock();
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;

		EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
		if(chooseVersion == CURRENT)
		{
			EraseInitStruct.PageAddress   = PROGRAM_START_CURRENT;
			EraseInitStruct.NbPages       = SIZE_PAGE_CURRENT;
		}
		else
		{
			EraseInitStruct.PageAddress   = PROGRAM_START_OTA;
			EraseInitStruct.NbPages       = SIZE_PAGE_OTA;
		}

		int ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
}

static void goto_application( void )
{
	uint32_t PROGRAM_START_ADDRESS = PROGRAM_START_CURRENT;
	
	void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*)(PROGRAM_START_ADDRESS + 4U)));

	if( app_reset_handler == (void*)0xFFFFFFFF )
	{
	  while(1);
	}
	__set_MSP(*(volatile uint32_t*) PROGRAM_START_ADDRESS);
	app_reset_handler();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){}
		if(RxHeader.StdId == 0x01)
		{
			bootloaderState = FW_SIZE;
			resetTick();
			initTime = HAL_GetTick();
			uint32_t ts = 1;
			for(int8_t i = RxHeader.DLC - 1; i >= 0; i--)
			{
				size_program += (uint32_t)(RxData[i]*ts);
				ts*=10;
			}
					flag = 1;
		}
		
		if(RxHeader.StdId == 0x02 && flag == 1)
		{
			resetTick();
			initTime = HAL_GetTick();
			if((uint32_t)size_count_can*8 >= size_program )
			{
				percentBuf[0] = 100;
				CAN_Send(PERCENT_ADDR, percentBuf, 1);
				write_data_to_flash_app( RxData, RxHeader.DLC);
				clearFlash(CURRENT);
				updateCurrent();
				bootloaderState = CAN_FLASH;
				return;
			}
		else
		{
			write_data_to_flash_app( RxData, RxHeader.DLC);
			for(uint8_t i = 0; i < 8; i++)
			{
				int8_t percent = processPercentCAN(size_program, counter++);
				if(percent != -1 && percent != currentPercent)
				{
					currentPercent = percent;
					memset(percentBuf,0, 1);
					percentBuf[0] = percent;
					CAN_Send(PERCENT_ADDR, percentBuf, 1);
					// break;
				}
			}
			for(uint8_t i = 0; i < 8;i++)
			{
				RxData[i] = 0xFF;
			}
			size_count_can++;
			number_data_can++;
		}
	}
}

void resetTick()
{
	static uint8_t initialized = 0;
	if (!initialized)
	{
		HAL_InitTick(TICK_INT_PRIORITY);
		initialized = 1;
	}
}

void updateCurrent()
{
	HAL_StatusTypeDef status;
	uint32_t i = 0;
	uint32_t buffer;

	HAL_FLASH_Unlock();

	for (i = 0; i < FLASH_SIZE_HEX; i += 4) {
			buffer = *(__IO uint32_t*)(PROGRAM_START_OTA + i);

			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PROGRAM_START_CURRENT + i, buffer);
			
			if (status != HAL_OK) {
					break;
			}
	}

	HAL_FLASH_Lock();
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
