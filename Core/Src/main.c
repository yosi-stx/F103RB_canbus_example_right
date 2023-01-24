/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEFT_SIDE_NUCLEO 0
#if ( LEFT_SIDE_NUCLEO == 1)
    // FILTER_ID_LOW is the filter used, for catching packets from the other device
    #define FILTER_ID_LOW   0x104
    // TRANSMITER_STANDARD_ID - the standard ID of this device.
    #define TRANSMITER_STANDARD_ID 0x103
#else
    // FILTER_ID_LOW is the filter used, for catching packets from the other device
    #define FILTER_ID_LOW   0x103
    // TRANSMITER_STANDARD_ID - the standard ID of this device.
    #define TRANSMITER_STANDARD_ID 0x104
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

uint8_t Uart_DMA_in_buffer[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];

CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

uint32_t              TxMailbox;
int datacheck = 0;
uint8_t     CAN_TxData[8];
uint8_t     CAN_RxData[8];

uint8_t     UART_TxData[32];
uint8_t     UART_RxData[32];

uint8_t count;
uint32_t led_timer;
uint8_t led = 0;

void led_off()
{
//    HAL_GPIO_WritePin(LED_PC_13_GPIO_Port, LED_PC_13_Pin,1);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,0);
    led = 0;
}

void led_on()
{
//    HAL_GPIO_WritePin(LED_PC_13_GPIO_Port, LED_PC_13_Pin,0);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1);
    led = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t in_len;
    uint32_t out_len=0;
    uint8_t lf_cr[2] = {0xa,0xd};

    count++;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RxData);
//    HAL_GPIO_TogglePin(LED_PC_13_GPIO_Port, LED_PC_13_Pin);
    in_len = RxHeader.DLC;
    out_len = in_len;
    for(int i=0;i<in_len; i++)
    {
        if(CAN_RxData[i] >= 0x20)
        {
            UART_TxData[i] = CAN_RxData[i];
        }
        else
        {
            UART_TxData[i] = CAN_RxData[i] + 0x20;
        }
    }
    HAL_UART_Transmit(&huart1, UART_TxData, out_len, 10);
    HAL_UART_Transmit(&huart1, lf_cr, 2, 10);

    led_on();
    led_timer = HAL_GetTick();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        TxData[0] = 100; // 100ms
        TxData[1] = 10;  // repeat 10 times.

        uint8_t temp_uart_tx[] = "blue button this side\r\n";
        HAL_UART_Transmit(&huart1, temp_uart_tx, sizeof(temp_uart_tx), 10);

        HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
    }
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//
//}
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }

  //  if ((RxHeader.StdId == 0x103))
  if ((RxHeader.DLC == 2))
  {
      datacheck = 1;
  }
}
*/


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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);


  // Activate the notification
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeader.DLC = 1;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = TRANSMITER_STANDARD_ID; // this ID can be 11bit wide. (for left = 0x103)
  TxHeader.TransmitGlobalTime = DISABLE;

  CAN_TxData[0] = 0x73; // = 's' , for start.
  TxHeader.DLC = 2;
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_TxData, &TxMailbox);
  HAL_UART_Receive_DMA(&huart1, Uart_DMA_in_buffer, 1);//

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t main_tickstart = HAL_GetTick();
#if ( LEFT_SIDE_NUCLEO == 1)
  uint8_t temp_uart_tx[] = "Hello CAN left side\r\n";
#else
  uint8_t temp_uart_tx[] = "Hello CAN right side\r\n";

#endif
  HAL_UART_Transmit(&huart1, temp_uart_tx, sizeof(temp_uart_tx), 10);

  while (1)
  {
      if (datacheck)
      {
          for(int i=0; i< RxData[1]; i++)
          {
//              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
              HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
              HAL_Delay(RxData[0]);
          }
          datacheck = 0;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if( (HAL_GetTick() - main_tickstart) > 1000 ) // do every 1000ms
    {
//        HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_TxData, &TxMailbox);
//        CAN_TxData[0] += 1;
        main_tickstart = HAL_GetTick();
        //HAL_UART_Transmit(&huart1, temp_uart_tx, sizeof(temp_uart_tx), 10);
    }
    if( led && (HAL_GetTick() - led_timer > 100))
    {
//        HAL_GPIO_WritePin(LED_PC_13_GPIO_Port, LED_PC_13_Pin,0);
        led_off();
    }

//    HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
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
  CAN_FilterTypeDef canfilterconfig;
  
#if ( LEFT_SIDE_NUCLEO == 1)
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0;
  canfilterconfig.FilterIdLow = FILTER_ID_LOW<<5;
  canfilterconfig.FilterMaskIdHigh = 0;
  canfilterconfig.FilterMaskIdLow = FILTER_ID_LOW<<5;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN1 (master can)
#else
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0;
  canfilterconfig.FilterIdLow = FILTER_ID_LOW<<5;
  canfilterconfig.FilterMaskIdHigh = 0;
  canfilterconfig.FilterMaskIdLow = FILTER_ID_LOW<<5;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN1 (master can)
#endif
/*  
#if ( LEFT_SIDE_NUCLEO == 1)
   canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
   canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
   canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   canfilterconfig.FilterIdHigh = 0x104<<5; // 0x104  - this ID should be assigned to the other side.
   canfilterconfig.FilterIdLow = 0;
   canfilterconfig.FilterMaskIdHigh = 0x104<<5;
   canfilterconfig.FilterMaskIdLow = 0x0000;
   canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
   canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
   canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN
#else
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 10;  // which filter bank to use from the assigned ones
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterIdHigh = 0x103<<5; // 0x104  - this ID should be assigned to the other side.
    canfilterconfig.FilterIdLow = 0;
    canfilterconfig.FilterMaskIdHigh = 0x103<<5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 0;  // how many filters to assign to the CAN
#endif
*/
   HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint16_t in_ptr=0;
    static uint16_t out_ptr=0;
  uint8_t send_length;
  uint8_t lf_cr[2] = {0xa,0xd};
  uint8_t bs_sp[2] = {0x8,0x20};


  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */

    UART_RxData[in_ptr++] = Uart_DMA_in_buffer[0];
    if( Uart_DMA_in_buffer[0] >= 0x20 )
    {
        HAL_UART_Transmit(&huart1, Uart_DMA_in_buffer, 1, 10);
        out_ptr++;
    }
    if( Uart_DMA_in_buffer[0] == 0x8 ) // if BS
    {
        HAL_UART_Transmit(&huart1, bs_sp, 1, 10);
        in_ptr--;
        out_ptr--;
    }

    if(in_ptr > 8)
        in_ptr=0;
    if(out_ptr > 8)
        out_ptr=0;
    if(Uart_DMA_in_buffer[0] == 0xd)
    {
        send_length = out_ptr;
  //    HAL_UART_Transmit(&huart2, Uart_Rx_buffer, send_length, 10);
        HAL_UART_Transmit(&huart1, lf_cr, 2, 10);
        in_ptr = 0;
        out_ptr = 0;
//        if(send_length>8)
//            send_length = 8;
        for(int i=0;i<send_length;i++)
        {
            if(i>=8)
                break;
            CAN_TxData[i] = UART_RxData[i];
        }

        TxHeader.DLC = send_length; //Specifies the length of the frame that will be transmitted
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_TxData, &TxMailbox);

    }


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
