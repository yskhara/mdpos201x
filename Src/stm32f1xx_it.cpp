/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_it.h"

#include "motor_ctrl.hpp"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern MotorCtrl control;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

extern "C"
{

    /******************************************************************************/
    /*           Cortex-M3 Processor Interruption and Exception Handlers          */
    /******************************************************************************/
    /**
     * @brief This function handles Non maskable interrupt.
     */
    void NMI_Handler(void)
    {
        /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

        /* USER CODE END NonMaskableInt_IRQn 0 */
        /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

        /* USER CODE END NonMaskableInt_IRQn 1 */
    }

    /**
     * @brief This function handles Hard fault interrupt.
     */
    void HardFault_Handler(void)
    {
        /* USER CODE BEGIN HardFault_IRQn 0 */

        /* USER CODE END HardFault_IRQn 0 */
        while (1)
        {
            /* USER CODE BEGIN W1_HardFault_IRQn 0 */
            /* USER CODE END W1_HardFault_IRQn 0 */
        }
    }

    /**
     * @brief This function handles Memory management fault.
     */
    void MemManage_Handler(void)
    {
        /* USER CODE BEGIN MemoryManagement_IRQn 0 */

        /* USER CODE END MemoryManagement_IRQn 0 */
        while (1)
        {
            /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
            /* USER CODE END W1_MemoryManagement_IRQn 0 */
        }
    }

    /**
     * @brief This function handles Prefetch fault, memory access fault.
     */
    void BusFault_Handler(void)
    {
        /* USER CODE BEGIN BusFault_IRQn 0 */

        /* USER CODE END BusFault_IRQn 0 */
        while (1)
        {
            /* USER CODE BEGIN W1_BusFault_IRQn 0 */
            /* USER CODE END W1_BusFault_IRQn 0 */
        }
    }

    /**
     * @brief This function handles Undefined instruction or illegal state.
     */
    void UsageFault_Handler(void)
    {
        /* USER CODE BEGIN UsageFault_IRQn 0 */

        /* USER CODE END UsageFault_IRQn 0 */
        while (1)
        {
            /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
            /* USER CODE END W1_UsageFault_IRQn 0 */
        }
    }

    /**
     * @brief This function handles System service call via SWI instruction.
     */
    void SVC_Handler(void)
    {
        /* USER CODE BEGIN SVCall_IRQn 0 */

        /* USER CODE END SVCall_IRQn 0 */
        /* USER CODE BEGIN SVCall_IRQn 1 */

        /* USER CODE END SVCall_IRQn 1 */
    }

    /**
     * @brief This function handles Debug monitor.
     */
    void DebugMon_Handler(void)
    {
        /* USER CODE BEGIN DebugMonitor_IRQn 0 */

        /* USER CODE END DebugMonitor_IRQn 0 */
        /* USER CODE BEGIN DebugMonitor_IRQn 1 */

        /* USER CODE END DebugMonitor_IRQn 1 */
    }

    /**
     * @brief This function handles Pendable request for system service.
     */
    void PendSV_Handler(void)
    {
        /* USER CODE BEGIN PendSV_IRQn 0 */

        /* USER CODE END PendSV_IRQn 0 */
        /* USER CODE BEGIN PendSV_IRQn 1 */

        /* USER CODE END PendSV_IRQn 1 */
    }

    /**
     * @brief This function handles System tick timer.
     */
    void SysTick_Handler(void)
    {
        /* USER CODE BEGIN SysTick_IRQn 0 */

        /* USER CODE END SysTick_IRQn 0 */
        HAL_IncTick();
        HAL_SYSTICK_IRQHandler();
        /* USER CODE BEGIN SysTick_IRQn 1 */

        /* USER CODE END SysTick_IRQn 1 */
    }

    /******************************************************************************/
    /* STM32F1xx Peripheral Interrupt Handlers                                    */
    /* Add here the Interrupt Handlers for the used peripherals.                  */
    /* For the available peripheral interrupt handler names,                      */
    /* please refer to the startup file (startup_stm32f1xx.s).                    */
    /******************************************************************************/

    /**
     * @brief This function handles EXTI line2 interrupt.
     */
    void EXTI2_IRQHandler(void)
    {
        if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
        {
            LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);

#ifdef CTRL_POS
            control.LimitSwitch0Handler();
#endif
        }
    }

    /**
     * @brief This function handles EXTI line3 interrupt.
     */
    void EXTI3_IRQHandler(void)
    {
        if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
        {
            LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);

#ifdef CTRL_POS
            control.LimitSwitch1Handler();
#endif
        }
    }

    /**
     * @brief This function handles USB low priority or CAN RX0 interrupts.
     */
    void USB_LP_CAN1_RX0_IRQHandler(void)
    {
        /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

        /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
        HAL_CAN_IRQHandler(&hcan);
        /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

        /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
    }

    void TIM1_UP_IRQHandler(void)
    {
        if ((TIM1->SR & TIM_SR_UIF) != RESET)
        {
            TIM1->SR = ~TIM_SR_UIF;

            //control.PollEnc();
        }
    }

    /**
     * @brief This function handles TIM3 global interrupt.
     * This is the workhorse of the md201x.
     * this handler is called @ 1 kHz.
     */
    void TIM3_IRQHandler(void)
    {
        if ((TIM3->SR & TIM_SR_UIF) != RESET)
        {
            TIM3->SR = ~TIM_SR_UIF;

            control.Control();
        }
    }

    /**
     * @brief This function handles DMA1 channel4 global interrupt.
     */
    void DMA1_Channel4_IRQHandler(void)
    {
        HAL_DMA_IRQHandler(&hdma_usart1_tx);
    }

    /**
     * @brief This function handles DMA1 channel5 global interrupt.
     */
    void DMA1_Channel5_IRQHandler(void)
    {
        HAL_DMA_IRQHandler(&hdma_usart1_rx);
    }

    void USART1_IRQHandler(void)
    {
        HAL_UART_IRQHandler(&huart1);
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
