/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_cordic_wr;

extern DMA_HandleTypeDef hdma_cordic_rd;

extern DMA_HandleTypeDef hdma_fmac_rd;

extern DMA_HandleTypeDef hdma_fmac_wr;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief CORDIC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hcordic: CORDIC handle pointer
  * @retval None
  */
void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
    /* USER CODE BEGIN CORDIC_MspInit 0 */

    /* USER CODE END CORDIC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* CORDIC DMA Init */
    /* CORDIC_WR Init */
    hdma_cordic_wr.Instance = DMA1_Stream0;
    hdma_cordic_wr.Init.Request = DMA_REQUEST_CORDIC_WRITE;
    hdma_cordic_wr.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cordic_wr.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_wr.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_wr.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_wr.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_wr.Init.Mode = DMA_NORMAL;
    hdma_cordic_wr.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cordic_wr.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cordic_wr) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hcordic,hdmaIn,hdma_cordic_wr);

    /* CORDIC_RD Init */
    hdma_cordic_rd.Instance = DMA1_Stream1;
    hdma_cordic_rd.Init.Request = DMA_REQUEST_CORDIC_READ;
    hdma_cordic_rd.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cordic_rd.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_rd.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_rd.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_rd.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_rd.Init.Mode = DMA_NORMAL;
    hdma_cordic_rd.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cordic_rd.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cordic_rd) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hcordic,hdmaOut,hdma_cordic_rd);

    /* USER CODE BEGIN CORDIC_MspInit 1 */

    /* USER CODE END CORDIC_MspInit 1 */

  }

}

/**
  * @brief CORDIC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hcordic: CORDIC handle pointer
  * @retval None
  */
void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
    /* USER CODE BEGIN CORDIC_MspDeInit 0 */

    /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();

    /* CORDIC DMA DeInit */
    HAL_DMA_DeInit(hcordic->hdmaIn);
    HAL_DMA_DeInit(hcordic->hdmaOut);
    /* USER CODE BEGIN CORDIC_MspDeInit 1 */

    /* USER CODE END CORDIC_MspDeInit 1 */
  }

}

/**
  * @brief FDCAN MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hfdcan: FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hfdcan->Instance==FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspInit 0 */

    /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspInit 1 */

    /* USER CODE END FDCAN1_MspInit 1 */

  }

}

/**
  * @brief FDCAN MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hfdcan: FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
  if(hfdcan->Instance==FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

    /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* FDCAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

    /* USER CODE END FDCAN1_MspDeInit 1 */
  }

}

/**
  * @brief FMAC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hfmac: FMAC handle pointer
  * @retval None
  */
void HAL_FMAC_MspInit(FMAC_HandleTypeDef* hfmac)
{
  if(hfmac->Instance==FMAC)
  {
    /* USER CODE BEGIN FMAC_MspInit 0 */

    /* USER CODE END FMAC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_FMAC_CLK_ENABLE();

    /* FMAC DMA Init */
    /* FMAC_RD Init */
    hdma_fmac_rd.Instance = DMA1_Stream2;
    hdma_fmac_rd.Init.Request = DMA_REQUEST_FMAC_READ;
    hdma_fmac_rd.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_fmac_rd.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_fmac_rd.Init.MemInc = DMA_MINC_ENABLE;
    hdma_fmac_rd.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_fmac_rd.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_fmac_rd.Init.Mode = DMA_NORMAL;
    hdma_fmac_rd.Init.Priority = DMA_PRIORITY_LOW;
    hdma_fmac_rd.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_fmac_rd) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hfmac,hdmaOut,hdma_fmac_rd);

    /* FMAC_WR Init */
    hdma_fmac_wr.Instance = DMA1_Stream3;
    hdma_fmac_wr.Init.Request = DMA_REQUEST_FMAC_WRITE;
    hdma_fmac_wr.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_fmac_wr.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_fmac_wr.Init.MemInc = DMA_MINC_ENABLE;
    hdma_fmac_wr.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_fmac_wr.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_fmac_wr.Init.Mode = DMA_NORMAL;
    hdma_fmac_wr.Init.Priority = DMA_PRIORITY_LOW;
    hdma_fmac_wr.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_fmac_wr) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hfmac,hdmaIn,hdma_fmac_wr);

    /* USER CODE BEGIN FMAC_MspInit 1 */

    /* USER CODE END FMAC_MspInit 1 */

  }

}

/**
  * @brief FMAC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hfmac: FMAC handle pointer
  * @retval None
  */
void HAL_FMAC_MspDeInit(FMAC_HandleTypeDef* hfmac)
{
  if(hfmac->Instance==FMAC)
  {
    /* USER CODE BEGIN FMAC_MspDeInit 0 */

    /* USER CODE END FMAC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FMAC_CLK_DISABLE();

    /* FMAC DMA DeInit */
    HAL_DMA_DeInit(hfmac->hdmaOut);
    HAL_DMA_DeInit(hfmac->hdmaIn);
    /* USER CODE BEGIN FMAC_MspDeInit 1 */

    /* USER CODE END FMAC_MspDeInit 1 */
  }

}

/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(huart->Instance==USART3)
  {
    /* USER CODE BEGIN USART3_MspInit 0 */

    /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = STLK_VCP_RX_Pin|STLK_VCP_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN USART3_MspInit 1 */

    /* USER CODE END USART3_MspInit 1 */

  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART3)
  {
    /* USER CODE BEGIN USART3_MspDeInit 0 */

    /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, STLK_VCP_RX_Pin|STLK_VCP_TX_Pin);

    /* USER CODE BEGIN USART3_MspDeInit 1 */

    /* USER CODE END USART3_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
