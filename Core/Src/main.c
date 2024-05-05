/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include <string.h>
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DS18B20.h"
#include "max7219.h"
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DS18B20_Name DS1;
float Temp;
long last =0;
int i=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif	
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2 ,(uint8_t *)&ch,1,0xFFFF);
	return ch;
}

#define EEPROM_ADDRESS 0xA0
uint8_t day = 18;
uint8_t month = 3;
uint16_t year = 2024;
char data_to_read[20];

void Read_data(void);
const int MAX_LEN = 18;
uint8_t nRxData[MAX_LEN];
uint8_t nTxData[MAX_LEN];
uint8_t strCommand[4];
uint8_t strOpt[3];
uint8_t strData[8];
bool bDataAvailable = false;

uint8_t STX[] = {0x02U};
uint8_t ETX[] = {0x03U};
uint8_t ACK[] = {0x06U};
uint8_t SYN[] = {0x16U};

uint8_t *subString(uint8_t *s, int pos, int index)
{
	uint8_t *t = &s[pos];
	s[pos-1] = '\0';
	for (int i = index; i < (strlen((char *)t) + 1); i++)
  {
   t[i] = '\0';
  }
return t;
}

bool StrCompare(uint8_t *pBuff, uint8_t *Sample, uint8_t nSize)
{
	for	(int i = 0; i < nSize; i++)
	{
		if (pBuff[i] != Sample[i])
			{
				return false;
			}
	}
	return true;
}

bool WriteComm(uint8_t *pBuff, uint8_t nSize)
{
 return HAL_UART_Transmit(&huart2, pBuff, nSize, 1000);
}

bool ReadComm (uint8_t *pBuff, uint8_t nSize)
{
  if ((pBuff[0] == STX[0]) && (pBuff[17] == ETX[0]))
	{
   memcpy (strCommand, subString (pBuff, 1, 4), 4);
   memcpy (strOpt, subString (pBuff, 5, 3), 3);
   memcpy (strData, subString (pBuff, 8, 8), 8);
		
   bDataAvailable = true;
  }
	else
	{
   bDataAvailable = false;
	}
	
  return bDataAvailable;
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
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t *)nRxData, MAX_LEN);
	bool serialProcess();
	DS18B20_Init(&DS1, &htim4, DS18B20_GPIO_Port, DS18B20_Pin);
	
	

  /* USER CODE END 2 */
uint8_t data_to_write[5];
					data_to_write[0] = day;
					data_to_write[1] = month;
					data_to_write[2] = (uint8_t)(year >> 8);  // High byte of year
					data_to_write[3] = (uint8_t)(year & 0xFF);
					HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, 0 , 0xFF, data_to_write, 4,10);
					HAL_Delay(10);
	
					uint8_t data_read[4];
					HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, 0, 0xFF, data_read, 4,10 );
					//show the data eeprom to uart
					day = data_read[0];
					month = data_read[1];
					year = ((uint16_t)data_read[2] << 8) | data_read[3];
					// Format the read data
					sprintf(data_to_read, "%02d/%02d/%04d", day, month, year);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		serialProcess();	
  }
  /* USER CODE END 3 */
}
void Read_data(void)
{
	Temp = DS18B20_ReadTemp(&DS1);
	//HAL_Delay(2000);
	/*last = HAL_GetTick();
	while(1)
	{
		if(HAL_GetTick() - last >=1200)
		{
			i++;
			printf("%d.Temperature: %0.2f\r\n",i,Temp);
			last = HAL_GetTick();
			break;
		}
	} */
}
bool serialProcess(void)
{
	uint8_t nIndex = 0;
	static bool dataSent = false;
	if(!dataSent&&bDataAvailable)
	{
			Read_data();
			char tempString[10];
      sprintf(tempString, "%.2f", Temp);
			max7219_Init( 7 ); 
			max7219_Decode_On(); 
		  if(StrCompare (strCommand, (uint8_t *)"DISP", 4)) // du lieu luu trong bien nTxData de gui tu STM32 len PC(GUI)
			{ 
				
				memcpy(nTxData + nIndex, STX, 1);
				nIndex += 1;
				memcpy(nTxData + nIndex, strCommand, 4);
				nIndex += 4;
				memcpy(nTxData + nIndex, "ND:", 3);
				nIndex += 3;
				memcpy(nTxData + nIndex, tempString, strlen((char *)tempString));
				nIndex += 8;
				memcpy(nTxData + nIndex, ACK, 1);
				nIndex += 1;
				memcpy(nTxData + nIndex, ETX, 1);
				
				WriteComm(nTxData, MAX_LEN);
				dataSent = true;
			}
				else if(StrCompare (strCommand, (uint8_t *)"EEPR", 4)){
					
					
					memcpy(nTxData + nIndex, STX, 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, strCommand, 4);
					nIndex += 4;
					memcpy(nTxData + nIndex, "OM:", 3);
					nIndex += 3;
					memcpy(nTxData + nIndex,(uint8_t*) data_to_read , strlen(data_to_read));
					nIndex += strlen(data_to_read);
					memcpy(nTxData + nIndex, ACK, 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, ETX, 1);
				
					WriteComm(nTxData, nIndex); 
					dataSent = true;
			} else if(StrCompare (strCommand, (uint8_t *)"7SEG", 4)) {
				max7219_Clean();
				max7219_PrintNtos(DIGIT_8, 605,4);
				max7219_PrintItos(DIGIT_4, 2024);
				HAL_Delay(1500);
				memcpy(nTxData + nIndex, STX, 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, strCommand, 4);
					nIndex += 4;
					memcpy(nTxData + nIndex, "LED", 3);
					nIndex += 3;
					memcpy(nTxData + nIndex,"06052024", 8);
					nIndex += 8;
					memcpy(nTxData + nIndex, "*", 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, "*", 1);	
					WriteComm(nTxData, nIndex); 
					dataSent = true;
			}
			else if(StrCompare (strCommand, (uint8_t *)"ON13", 4)) {
				
				  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
					memcpy(nTxData + nIndex, STX, 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, strCommand, 4);
					nIndex += 4;
					memcpy(nTxData + nIndex, "LED", 3);
					nIndex += 3;
					memcpy(nTxData + nIndex,"LEDSANG", 7);
					nIndex += 7;
					memcpy(nTxData + nIndex, "O", 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, "O", 1);	
					WriteComm(nTxData, nIndex); 
					dataSent = true;
			}
			else if(StrCompare (strCommand, (uint8_t *)"OFFP", 4)) {
				
				  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
					memcpy(nTxData + nIndex, STX, 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, strCommand, 4);
					nIndex += 4;
					memcpy(nTxData + nIndex, "C13", 3);
					nIndex += 3;
					memcpy(nTxData + nIndex,"LED13TAT", 8);
					nIndex += 8;
					memcpy(nTxData + nIndex, "F", 1);
					nIndex += 1;
					memcpy(nTxData + nIndex, "F", 1);	
					WriteComm(nTxData, nIndex); 
					dataSent = true;
			}
				else 
			{
				memcpy(nTxData + nIndex, STX, 1);
				nIndex += 1;
				memcpy(nTxData + nIndex, "NULL", 4);
				nIndex += 4;
				memcpy(nTxData + nIndex, strOpt, 3);
				nIndex += 3;
				memcpy(nTxData + nIndex, strData, 8);
				nIndex += 8;
				memcpy(nTxData + nIndex, ACK, 1);
				nIndex += 1;
				memcpy(nTxData + nIndex, ETX, 1);
				
				WriteComm(nTxData, MAX_LEN);
				dataSent = true;
			}
		
	}
	if (bDataAvailable) {
        // Reset the flag and process received data when data is available
        dataSent = false;
        bDataAvailable = false;
        // Process received data here
    }
	return true;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
         if (huart->Instance == huart2.Instance)
				   ReadComm(nRxData, MAX_LEN);
				   HAL_UART_Receive_IT(&huart2, (uint8_t *)nRxData, MAX_LEN);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_MAX7219_Pin|DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_MAX7219_Pin DS18B20_Pin */
  GPIO_InitStruct.Pin = CS_MAX7219_Pin|DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
