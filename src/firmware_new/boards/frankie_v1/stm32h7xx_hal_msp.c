/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : stm32h7xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_adc2;

extern DMA_HandleTypeDef hdma_adc3;

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
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

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hadc->Instance == ADC1)
    {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* Peripheral clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if (HAL_RCC_ADC12_CLK_ENABLED == 1)
        {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }

        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**ADC1 GPIO Configuration
        PF11     ------> ADC1_INP2
        */
        GPIO_InitStruct.Pin  = wheel_encoder_front_left_adc_0_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(wheel_encoder_front_left_adc_0_GPIO_Port, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        /* ADC1 Init */
        hdma_adc1.Instance                 = DMA1_Stream1;
        hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hdma_adc1.Init.Mode                = DMA_CIRCULAR;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

        /* ADC1 interrupt Init */
        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
        /* USER CODE BEGIN ADC1_MspInit 1 */

        /* USER CODE END ADC1_MspInit 1 */
    }
    else if (hadc->Instance == ADC2)
    {
        /* USER CODE BEGIN ADC2_MspInit 0 */

        /* USER CODE END ADC2_MspInit 0 */
        /* Peripheral clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if (HAL_RCC_ADC12_CLK_ENABLED == 1)
        {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }

        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**ADC2 GPIO Configuration
        PB0     ------> ADC2_INP9
        PF14     ------> ADC2_INP6
        */
        GPIO_InitStruct.Pin  = wheel_encoder_back_right_adc_1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(wheel_encoder_back_right_adc_1_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = wheel_encoder_front_left_adc_1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(wheel_encoder_front_left_adc_1_GPIO_Port, &GPIO_InitStruct);

        /* ADC2 DMA Init */
        /* ADC2 Init */
        hdma_adc2.Instance                 = DMA1_Stream0;
        hdma_adc2.Init.Request             = DMA_REQUEST_ADC2;
        hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc2.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hdma_adc2.Init.Mode                = DMA_CIRCULAR;
        hdma_adc2.Init.Priority            = DMA_PRIORITY_MEDIUM;
        hdma_adc2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

        /* ADC2 interrupt Init */
        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
        /* USER CODE BEGIN ADC2_MspInit 1 */

        /* USER CODE END ADC2_MspInit 1 */
    }
    else if (hadc->Instance == ADC3)
    {
        /* USER CODE BEGIN ADC3_MspInit 0 */

        /* USER CODE END ADC3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_ADC3_CLK_ENABLE();

        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**ADC3 GPIO Configuration
        PF7     ------> ADC3_INP3
        PF8     ------> ADC3_INP7
        PF9     ------> ADC3_INP2
        PC2_C     ------> ADC3_INP0
        PC3_C     ------> ADC3_INP1
        */
        GPIO_InitStruct.Pin = wheel_encoder_back_right_adc_0_Pin |
                              wheel_encoder_back_left_adc_1_Pin |
                              wheel_encoder_back_left_adc_0_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_OPEN);

        HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_OPEN);

        /* ADC3 DMA Init */
        /* ADC3 Init */
        hdma_adc3.Instance                 = BDMA_Channel0;
        hdma_adc3.Init.Request             = BDMA_REQUEST_ADC3;
        hdma_adc3.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc3.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc3.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_adc3.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
        hdma_adc3.Init.Mode                = DMA_CIRCULAR;
        hdma_adc3.Init.Priority            = DMA_PRIORITY_MEDIUM;
        if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc3);

        /* ADC3 interrupt Init */
        HAL_NVIC_SetPriority(ADC3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC3_IRQn);
        /* USER CODE BEGIN ADC3_MspInit 1 */

        /* USER CODE END ADC3_MspInit 1 */
    }
}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if (HAL_RCC_ADC12_CLK_ENABLED == 0)
        {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }

        /**ADC1 GPIO Configuration
        PF11     ------> ADC1_INP2
        */
        HAL_GPIO_DeInit(wheel_encoder_front_left_adc_0_GPIO_Port,
                        wheel_encoder_front_left_adc_0_Pin);

        /* ADC1 DMA DeInit */
        HAL_DMA_DeInit(hadc->DMA_Handle);

        /* ADC1 interrupt DeInit */
        /* USER CODE BEGIN ADC1:ADC_IRQn disable */
        /**
         * Uncomment the line below to disable the "ADC_IRQn" interrupt
         * Be aware, disabling shared interrupt may affect other IPs
         */
        /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
        /* USER CODE END ADC1:ADC_IRQn disable */

        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    }
    else if (hadc->Instance == ADC2)
    {
        /* USER CODE BEGIN ADC2_MspDeInit 0 */

        /* USER CODE END ADC2_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if (HAL_RCC_ADC12_CLK_ENABLED == 0)
        {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }

        /**ADC2 GPIO Configuration
        PB0     ------> ADC2_INP9
        PF14     ------> ADC2_INP6
        */
        HAL_GPIO_DeInit(wheel_encoder_back_right_adc_1_GPIO_Port,
                        wheel_encoder_back_right_adc_1_Pin);

        HAL_GPIO_DeInit(wheel_encoder_front_left_adc_1_GPIO_Port,
                        wheel_encoder_front_left_adc_1_Pin);

        /* ADC2 DMA DeInit */
        HAL_DMA_DeInit(hadc->DMA_Handle);

        /* ADC2 interrupt DeInit */
        /* USER CODE BEGIN ADC2:ADC_IRQn disable */
        /**
         * Uncomment the line below to disable the "ADC_IRQn" interrupt
         * Be aware, disabling shared interrupt may affect other IPs
         */
        /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
        /* USER CODE END ADC2:ADC_IRQn disable */

        /* USER CODE BEGIN ADC2_MspDeInit 1 */

        /* USER CODE END ADC2_MspDeInit 1 */
    }
    else if (hadc->Instance == ADC3)
    {
        /* USER CODE BEGIN ADC3_MspDeInit 0 */

        /* USER CODE END ADC3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC3_CLK_DISABLE();

        /**ADC3 GPIO Configuration
        PF7     ------> ADC3_INP3
        PF8     ------> ADC3_INP7
        PF9     ------> ADC3_INP2
        PC2_C     ------> ADC3_INP0
        PC3_C     ------> ADC3_INP1
        */
        HAL_GPIO_DeInit(GPIOF, wheel_encoder_back_right_adc_0_Pin |
                                   wheel_encoder_back_left_adc_1_Pin |
                                   wheel_encoder_back_left_adc_0_Pin);

        /* ADC3 DMA DeInit */
        HAL_DMA_DeInit(hadc->DMA_Handle);

        /* ADC3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(ADC3_IRQn);
        /* USER CODE BEGIN ADC3_MspDeInit 1 */

        /* USER CODE END ADC3_MspDeInit 1 */
    }
}

/**
 * @brief CRC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
    if (hcrc->Instance == CRC)
    {
        /* USER CODE BEGIN CRC_MspInit 0 */

        /* USER CODE END CRC_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_CRC_CLK_ENABLE();
        /* USER CODE BEGIN CRC_MspInit 1 */

        /* USER CODE END CRC_MspInit 1 */
    }
}

/**
 * @brief CRC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
    if (hcrc->Instance == CRC)
    {
        /* USER CODE BEGIN CRC_MspDeInit 0 */

        /* USER CODE END CRC_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_CRC_CLK_DISABLE();
        /* USER CODE BEGIN CRC_MspDeInit 1 */

        /* USER CODE END CRC_MspDeInit 1 */
    }
}

/**
 * @brief TIM_PWM MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
    if (htim_pwm->Instance == TIM4)
    {
        /* USER CODE BEGIN TIM4_MspInit 0 */

        /* USER CODE END TIM4_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();
        /* USER CODE BEGIN TIM4_MspInit 1 */

        /* USER CODE END TIM4_MspInit 1 */
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim->Instance == TIM4)
    {
        /* USER CODE BEGIN TIM4_MspPostInit 0 */

        /* USER CODE END TIM4_MspPostInit 0 */

        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM4 GPIO Configuration
        PD13     ------> TIM4_CH2
        PD14     ------> TIM4_CH3
        PD15     ------> TIM4_CH4
        PB6     ------> TIM4_CH1
        */
        GPIO_InitStruct.Pin = wheel_motor_back_left_pwm_Pin |
                              wheel_motor_back_right_pwm_Pin |
                              wheel_motor_front_right_pwm_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = wheel_motor_front_left_pwm_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(wheel_motor_front_left_pwm_GPIO_Port, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM4_MspPostInit 1 */

        /* USER CODE END TIM4_MspPostInit 1 */
    }
}
/**
 * @brief TIM_PWM MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
    if (htim_pwm->Instance == TIM4)
    {
        /* USER CODE BEGIN TIM4_MspDeInit 0 */

        /* USER CODE END TIM4_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();
        /* USER CODE BEGIN TIM4_MspDeInit 1 */

        /* USER CODE END TIM4_MspDeInit 1 */
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
    if (huart->Instance == USART3)
    {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        GPIO_InitStruct.Pin       = STLK_RX_Pin | STLK_TX_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
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
    if (huart->Instance == USART3)
    {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOD, STLK_RX_Pin | STLK_TX_Pin);

        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
}

/**
 * @brief PCD MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hpcd: PCD handle pointer
 * @retval None
 */
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hpcd->Instance == USB_OTG_FS)
    {
        /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

        /* USER CODE END USB_OTG_FS_MspInit 0 */

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USB_OTG_FS GPIO Configuration
        PA8     ------> USB_OTG_FS_SOF
        PA9     ------> USB_OTG_FS_VBUS
        PA10     ------> USB_OTG_FS_ID
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        GPIO_InitStruct.Pin       = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = USB_VBUS_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

        /* USER CODE END USB_OTG_FS_MspInit 1 */
    }
}

/**
 * @brief PCD MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hpcd: PCD handle pointer
 * @retval None
 */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
    if (hpcd->Instance == USB_OTG_FS)
    {
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

        /* USER CODE END USB_OTG_FS_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

        /**USB_OTG_FS GPIO Configuration
        PA8     ------> USB_OTG_FS_SOF
        PA9     ------> USB_OTG_FS_VBUS
        PA10     ------> USB_OTG_FS_ID
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        HAL_GPIO_DeInit(
            GPIOA, USB_SOF_Pin | USB_VBUS_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin);

        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

        /* USER CODE END USB_OTG_FS_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
