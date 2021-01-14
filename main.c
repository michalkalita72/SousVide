

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ONEWIRE_PIN_NUM                 4
#define ONEWIRE_PIN_MASK                (2<<ONEWIRE_PIN_NUM)
#define  ONEWIRE_PORT                   GPIOB

//#define  ONEWIRE_INPUT_READ             ONEWIRE_PORT->IDR&ONEWIRE_PIN_MASK
#define  ONEWIRE_INPUT_READ             ONEWIRE_PORT->IDR&0x10


#define  ONEWIRE_OUTPUT_HIGH            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define  ONEWIRE_OUTPUT_LOW             HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define  RELAY_OUTPUT_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define  RELAY_OUTPUT_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define RELAY_OUTPUT_HIGH_T HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5)
#define  RELAY_OUTPUT_LOW_T HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5)
#define  RELAY_OUTPUT_HIGH_A8 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define  RELAY_OUTPUT_LOW_A8 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

#define  ONEWIRE_CONFIG_OUTPUT ONEWIRE_PORT->MODER |= (0x0001<<8);
#define  ONEWIRE_CONFIG_INPUT           ONEWIRE_PORT->MODER &= (0x0000<<8);
#define  SKIP_ROM 0xCC
#define  CONVERT_TEMPERATURE 0x44
#define  READ_SCRATCHPAD 0xBE
#define  INTEGER_MASK 0x0FF0
#define  DECIMAL_MASK 0x000F
#define  COOK_TEMPERATURE 40.0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void temp_initialize(void);
void delay_us(uint16_t);
static inline void delay_us2(uint16_t);
static inline void delay_long(uint16_t);
static void  SendByte(uint8_t  val);
static  uint8_t  ReadByte(void);
void DisplayData(void);
static void ConvertTemperature(void);
static float ReadTemperature(void);
int OWReadBit(void);
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
  double Current_Temperature = 0.0;

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  TIM3->CR1 |= TIM_CR1_CEN; // starts timer 3 used in delay functions
  //ConvertTemperature();
  //ReadTemperature();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

 ConvertTemperature();
 Current_Temperature = ReadTemperature();

 if(Current_Temperature < COOK_TEMPERATURE){
 RELAY_OUTPUT_HIGH_A8;
 }

 else{
 RELAY_OUTPUT_LOW_A8;
 }
 HAL_Delay(1000);

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void temp_initialize(void) //RESET = high, SET = low
{
/*
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // SET = high
delay_us2(1);

HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
delay_us2(1);
*/

ONEWIRE_OUTPUT_HIGH; //Sets the output to high before configuring MODER.
ONEWIRE_CONFIG_OUTPUT; //Changes the GPIO bank into output mode, driven from master.
delay_us2(500);

ONEWIRE_OUTPUT_LOW; //Holds the line low
delay_us2(500);

ONEWIRE_OUTPUT_HIGH; //Puts it back to high briefly before switching modes.
ONEWIRE_CONFIG_INPUT; //Sets the master into High Z to receive presence pulse.
delay_us2(500); //Need to reach 480us master RX minimum
}

static void  SendByte(uint8_t  val)
{
uint8_t n;
for (n=0; n<8; n++)
{
   ONEWIRE_OUTPUT_LOW;
   ONEWIRE_CONFIG_OUTPUT;
   delay_us2(5); // output low for 5 microseconds
   if (val & 1)  ONEWIRE_OUTPUT_HIGH; // output high for 1, low for 0
   delay_us2(95); //60 - 120 us delay
   ONEWIRE_OUTPUT_HIGH;
   delay_us2(5);
   val = val >> 1; // one bit at a time
}
ONEWIRE_CONFIG_INPUT;
}

static  uint8_t  ReadByte(void)
{
    uint8_t m;
    uint8_t val;

    val = 0;
    for (m=0; m<8; m++) // only store the two temperature registers data.
    {
        val = val >> 1;
        ONEWIRE_OUTPUT_LOW;
        ONEWIRE_CONFIG_OUTPUT;
        delay_us2(10);
        //ONEWIRE_OUTPUT_HIGH;
        ONEWIRE_CONFIG_INPUT; // high z mode for 10 micro
        delay_us2(15);
        if (ONEWIRE_INPUT_READ)  val = val | 0x80;
        delay_us2(35);
    }

    return  val;
}

int OWReadBit(void)
{
        int result = 0;

        //outp(PORTADDRESS,0x00); // Drives DQ low
        ONEWIRE_OUTPUT_LOW;

        //tickDelay(A);
        delay_us2(6);

        //outp(PORTADDRESS,0x01); // Something might not be releasing the line.
        ONEWIRE_OUTPUT_HIGH;

        //tickDelay(E);
        delay_us2(9);

        //result = inp(PORTADDRESS) & 0x01; // Sample the bit value from the slave
        if(ONEWIRE_INPUT_READ){
        result = result | 0x80;
        }
        //tickDelay(F); // Complete the time slot and 10us recovery
        delay_us2(55);
        return result;
}

static float  ReadTemperature(void) // will not work for negative temperatures.
{
uint16_t raw_data; // used to be unsigned, leading 0s are getting truncated.
uint16_t data_1;
    uint8_t  decimal;
    uint8_t  integer;
    uint8_t  pad[9];
    float result;

    temp_initialize();
    delay_us2(100);
    SendByte(SKIP_ROM);
    SendByte(READ_SCRATCHPAD);

    for (int n=0; n<9; n++)
    {
        pad[n] = ReadByte();
    }
    data_1 = (pad[1] << 8);
    raw_data = data_1 | (pad[0]); // extracts the 2-byte temperature data in big-endian
    integer = ((raw_data & INTEGER_MASK) >> 4); // takes the integer data from the raw data
    decimal = raw_data & DECIMAL_MASK; // takes the decimal data from the raw data
    result = integer + (float)decimal*0.0625; // adds the integer + (0.0625/lsb) * (decimal in binary)
    return result;
}

static void ConvertTemperature(void)
{
int i = 0;
temp_initialize();
delay_us2(100);
SendByte(SKIP_ROM);
SendByte(CONVERT_TEMPERATURE);
while (i < 15){
delay_us2(50000);
i++;
}
}

static inline void delay_long(uint16_t us) // untested, probably dont need this function?
// more versatile than delay_us2.
{
int multiples = (int)(us/50000);
float remainder = (us % 50000);
for(int i = 0; i < multiples; i++){ // do 'multiples' number of 50000 us delays
__HAL_TIM_SET_COUNTER(&htim3, 0);
while (__HAL_TIM_GET_COUNTER(&htim3) < 50000); // a single 50000 microsecond delay
}
while (__HAL_TIM_GET_COUNTER(&htim3) < remainder); // broken up this way so that we can fit the delays
// into 16-bit registers.
}

static inline void delay_us2(uint16_t us)
{
__HAL_TIM_SET_COUNTER(&htim3, 0);
while (__HAL_TIM_GET_COUNTER(&htim3) < us);
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
