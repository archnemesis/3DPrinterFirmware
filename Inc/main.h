/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define STEPPER_CLK_0_Pin GPIO_PIN_2
#define STEPPER_CLK_0_GPIO_Port GPIOE
#define LIM_X1_Pin GPIO_PIN_3
#define LIM_X1_GPIO_Port GPIOE
#define LIM_X1_EXTI_IRQn EXTI3_IRQn
#define LIM_X2_Pin GPIO_PIN_4
#define LIM_X2_GPIO_Port GPIOE
#define LIM_X2_EXTI_IRQn EXTI4_IRQn
#define LIM_Y1_Pin GPIO_PIN_5
#define LIM_Y1_GPIO_Port GPIOE
#define LIM_Y1_EXTI_IRQn EXTI9_5_IRQn
#define LIM_Y2_Pin GPIO_PIN_6
#define LIM_Y2_GPIO_Port GPIOE
#define LIM_Y2_EXTI_IRQn EXTI9_5_IRQn
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define USER_Btn_EXTI_IRQn EXTI15_10_IRQn
#define STEPPER_EN_2_Pin GPIO_PIN_4
#define STEPPER_EN_2_GPIO_Port GPIOF
#define LIM_Z1_Pin GPIO_PIN_7
#define LIM_Z1_GPIO_Port GPIOF
#define LIM_Z1_EXTI_IRQn EXTI9_5_IRQn
#define LIM_Z2_Pin GPIO_PIN_8
#define LIM_Z2_GPIO_Port GPIOF
#define LIM_Z2_EXTI_IRQn EXTI9_5_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define STEPPER_CLK_2_Pin GPIO_PIN_2
#define STEPPER_CLK_2_GPIO_Port GPIOC
#define STEPPER_EN_0_Pin GPIO_PIN_0
#define STEPPER_EN_0_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define STEPPER_DIR_0_Pin GPIO_PIN_0
#define STEPPER_DIR_0_GPIO_Port GPIOB
#define STEPPER_FAULT_6_Pin GPIO_PIN_1
#define STEPPER_FAULT_6_GPIO_Port GPIOB
#define STEPPER_FAULT_6_EXTI_IRQn EXTI1_IRQn
#define STEPPER_FAULT_2_Pin GPIO_PIN_2
#define STEPPER_FAULT_2_GPIO_Port GPIOB
#define STEPPER_FAULT_2_EXTI_IRQn EXTI2_IRQn
#define STEPPER_CLK_5_Pin GPIO_PIN_13
#define STEPPER_CLK_5_GPIO_Port GPIOF
#define STEPPER_FAULT_5_Pin GPIO_PIN_14
#define STEPPER_FAULT_5_GPIO_Port GPIOF
#define STEPPER_FAULT_5_EXTI_IRQn EXTI15_10_IRQn
#define STEPPER_EN_4_Pin GPIO_PIN_15
#define STEPPER_EN_4_GPIO_Port GPIOF
#define STEPPER_EN_3_Pin GPIO_PIN_7
#define STEPPER_EN_3_GPIO_Port GPIOE
#define STEPPER_CLK_3_Pin GPIO_PIN_8
#define STEPPER_CLK_3_GPIO_Port GPIOE
#define STEPPER_EN_5_Pin GPIO_PIN_9
#define STEPPER_EN_5_GPIO_Port GPIOE
#define STEPPER_DIR_3_Pin GPIO_PIN_10
#define STEPPER_DIR_3_GPIO_Port GPIOE
#define STEPPER_DIR_5_Pin GPIO_PIN_11
#define STEPPER_DIR_5_GPIO_Port GPIOE
#define STEPPER_FAULT_3_Pin GPIO_PIN_12
#define STEPPER_FAULT_3_GPIO_Port GPIOE
#define STEPPER_FAULT_3_EXTI_IRQn EXTI15_10_IRQn
#define STEPPER_CLK_4_Pin GPIO_PIN_13
#define STEPPER_CLK_4_GPIO_Port GPIOE
#define STEPPER_CLK_1_Pin GPIO_PIN_14
#define STEPPER_CLK_1_GPIO_Port GPIOE
#define STEPPER_EN_1_Pin GPIO_PIN_15
#define STEPPER_EN_1_GPIO_Port GPIOE
#define STEPPER_DIR_1_Pin GPIO_PIN_10
#define STEPPER_DIR_1_GPIO_Port GPIOB
#define STEPPER_FAULT_1_Pin GPIO_PIN_11
#define STEPPER_FAULT_1_GPIO_Port GPIOB
#define STEPPER_FAULT_1_EXTI_IRQn EXTI15_10_IRQn
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define HEAT_A_Pin GPIO_PIN_12
#define HEAT_A_GPIO_Port GPIOD
#define HEAT_B_Pin GPIO_PIN_13
#define HEAT_B_GPIO_Port GPIOD
#define FAN_A_Pin GPIO_PIN_14
#define FAN_A_GPIO_Port GPIOD
#define FAN_B_Pin GPIO_PIN_15
#define FAN_B_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define STEPPER_CLK_6_Pin GPIO_PIN_15
#define STEPPER_CLK_6_GPIO_Port GPIOA
#define STEPPER_FAULT_4_Pin GPIO_PIN_9
#define STEPPER_FAULT_4_GPIO_Port GPIOG
#define STEPPER_FAULT_4_EXTI_IRQn EXTI9_5_IRQn
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define STEPPER_DIR_4_Pin GPIO_PIN_14
#define STEPPER_DIR_4_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define STEPPER_DIR_6_Pin GPIO_PIN_4
#define STEPPER_DIR_6_GPIO_Port GPIOB
#define STEPPER_EN_6_Pin GPIO_PIN_5
#define STEPPER_EN_6_GPIO_Port GPIOB
#define STEPPER_DIR_2_Pin GPIO_PIN_6
#define STEPPER_DIR_2_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define STEPPER_FAULT_0_Pin GPIO_PIN_0
#define STEPPER_FAULT_0_GPIO_Port GPIOE
#define STEPPER_FAULT_0_EXTI_IRQn EXTI0_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
