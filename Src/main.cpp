/*
 * MD201x firmware
 *
 * main.cpp
 */

/* Includes ------------------------------------------------------------------*/
#include <led.hpp>
#include <cmath>

#include "motor_ctrl.hpp"

#include "stm32f1xx_hal.h"
#include "main.h"

#include "can.hpp"
#include "SerialClass.hpp"
#include "uart_com.hpp"

#include "conf.h"

#include <cstdio>
#include <cstring>

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_WWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

MotorCtrl control;
extern SerialClass serial;

//#define SQUARE_TEST

//#define BROADCAST_STAT

// if conf0 pin is low on boot, diagnostic info is available on UART.
bool conf_diag_uart = false;
// if conf1 pin is low on boot, diagnostic info is available on CAN.
bool conf_diag_can = false;
bool conf2 = false;
bool conf3 = false;
bool conf4 = false;

void readPinConf(void)
{
    if ((GPIO_CONF0->IDR & GPIO_IDR_CONF0) == 0)
    {
        conf_diag_uart = true;
    }

    if ((GPIO_CONF1->IDR & GPIO_IDR_CONF1) == 0)
    {
        conf_diag_can = true;
    }

    if ((GPIO_CONF2->IDR & GPIO_IDR_CONF2) == 0)
    {
        conf2 = true;
    }

    if ((GPIO_CONF3->IDR & GPIO_IDR_CONF3) == 0)
    {
        conf3 = true;
    }

    if ((GPIO_CONF4->IDR & GPIO_IDR_CONF4) == 0)
    {
        conf4 = true;
    }
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();
    LL_SYSTICK_EnableIT();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_CRC_Init();
    //MX_IWDG_Init();
    //MX_WWDG_Init();
    //MX_ADC1_Init();
    //MX_ADC2_Init();

    readPinConf();

    readConf();
    control.ReadConfig();
    can_read_conf();

    //LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
    //LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);

    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_Delay(100);

    serial.start_dma();

    can_init();
    can_set_bitrate(CAN_BITRATE_1000K);

    const char * buf = nullptr;

    if (!conf_diag_uart)
    {
#ifdef CTRL_POS
        buf = "MDPOS201x BETA Position Control \r\nPWM_50K\r\nCAN_BITRATE_1000K \r\nInitializing...";
#else
        buf = "MDPOS201x BETA Velocity Control \r\nPWM_50K\r\nCAN_BITRATE_1000K \r\nInitializing...";
#endif
        serial.write((const uint8_t *) buf, strlen(buf));
    }

    GPIO_LED0->BSRR = GPIO_BSRR_BS_LED0;
    GPIO_LED1->BSRR = GPIO_BSRR_BS_LED1;
    GPIO_LED2->BSRR = GPIO_BSRR_BS_LED2;
    GPIO_LEDCAN->BSRR = GPIO_BSRR_BS_LEDCAN;

    HAL_Delay(100);

    GPIO_LED0->BSRR = GPIO_BSRR_BR_LED0;
    GPIO_LED1->BSRR = GPIO_BSRR_BR_LED1;
    GPIO_LED2->BSRR = GPIO_BSRR_BR_LED2;
    GPIO_LEDCAN->BSRR = GPIO_BSRR_BR_LEDCAN;

    // enable tim2 for encoder input
    LL_TIM_EnableCounter(TIM2);

    // enable tim3 for control interrupt
    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM3);

    int i = 0;

#ifdef CTRL_POS
    control.ResetPosition();
#endif
    control.SetTarget(0);
    HAL_Delay(100);

    // enable tim1 for pwm generation
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
    LL_TIM_EnableCounter(TIM1);
    //LL_TIM_EnableAllOutputs(TIM1);

    can_enable();
    //can_set_filter(0x04b4, 0x07fc);
    can_set_filter(confStruct.can_id_cmd, 0x07fc);

    if (conf_diag_uart)
    {
#ifdef CTRL_POS
        buf = "time [ms], cur_pos_pul, tgt_pos_pul, vel_cur, vel_tgt, err, err_prev, u_p, u_i, trq_tgt, volt_tgt\r\n";
#else
        buf = "time [ms], pulse, vel_cur, vel_tgt, err, err_prev, u_p, u_i, trq_tgt, volt_tgt\r\n";
#endif
        serial.write((const uint8_t *) buf, strlen(buf));
    }
    else
    {
        buf = "done.\r\n";
        serial.write((const uint8_t *) buf, strlen(buf));
        uart::prompt();
    }

#ifdef BROADCAST_STAT
    uint32_t last_stat_time = HAL_GetTick();
    uint32_t stat_interval = 200;
#endif

#ifdef SQUARE_TEST
    while ((GPIO_EMS->IDR & GPIO_IDR_EMS) == 0)
    {
    }

    GPIO_LED0->BSRR = GPIO_BSRR_BS_LED0;
    HAL_Delay(1000);
    GPIO_LED0->BSRR = GPIO_BSRR_BR_LED0;
    HAL_Delay(1000);
    GPIO_LED0->BSRR = GPIO_BSRR_BS_LED0;
    HAL_Delay(1000);
    GPIO_LED0->BSRR = GPIO_BSRR_BR_LED0;
    control.Recover();

    control.Home();

    while (control.IsHoming())
    {
    }

    control.Recover();

#endif

    /* Infinite loop */
    while (1)
    {
        //TIM1->BDTR |= TIM_BDTR_MOE;
        //TIM1->BDTR &= ~TIM_BDTR_MOE;
        //TIM1->CCR1 = 690 - 1;
        //TIM1->CCR2 = 0;

        //continue;

#ifndef SQUARE_TEST

#ifdef BROADCAST_STAT
        if (HAL_GetTick() - last_stat_time > stat_interval)
        {
            //uint16_t data = 0xaabb;

            /*
             if((GPIOC->IDR & GPIO_IDR_IDR14) != 0)
             {
             data = 0x0001;
             //GPIOB->BSRR = GPIO_BSRR_BS2;
             }
             else
             {
             //GPIOB->BSRR = GPIO_BSRR_BR2;
             }
             */

            CAN_TxHeaderTypeDef tx_header;
            uint8_t tx_payload[CAN_MTU];
            //uint32_t status;

            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.StdId = confStruct.can_id_stat;
            tx_header.DLC = 1;

            can_pack(tx_payload, static_cast<uint8_t>(control.GetStatusCode()));

            can_tx(&tx_header, tx_payload);

            last_stat_time = HAL_GetTick();
        }
#endif

        if (conf_diag_uart)
        {
            HAL_Delay(1);
            control.Print();
        }
        else
        {
            uart::process();
        }
        led::process();
        //HAL_Delay(1);

        continue;

#else

        if (i < 1000)
        {
#ifdef CTRL_POS
            control.SetTarget(0);
#else
            control.SetTarget(0);
#endif
        }
        else if (i < 2000)
        {
#ifdef CTRL_POS
            control.SetTarget(M_PI / 4);
#else
            control.SetTarget(1);
#endif
        }
        else if (i < 3000)
        {
#ifdef CTRL_POS
            //control.SetTarget(1 + (99 * (i - 2000) / 1000.0));
            control.SetTarget(0);
#else
            control.SetTarget(10);
#endif
        }
        else if (i < 4000)
        {
#ifdef CTRL_POS
            control.SetTarget(-M_PI / 4);
#else
            control.SetTarget(-10);
#endif
        }
        else if (i < 5000)
        {
#ifdef CTRL_POS
            control.SetTarget(0);
#else
            control.SetTarget(0);
#endif
        }
        else
        {
            //i = 0;

            control.Shutdown();

            buf = "\r\n";
            serial.write((const uint8_t *) buf, strlen(buf));

            while (1)
            {
                __NOP();
            }
        }

        /*
         CAN_TxHeaderTypeDef tx_header;
         tx_header.StdId = 0x0a;
         tx_header.RTR = CAN_RTR_DATA;
         tx_header.IDE = CAN_ID_STD;
         tx_header.DLC = 2;
         uint8_t tx_payload[CAN_MTU];
         tx_payload[0] = 0xab;
         tx_payload[1] = 0xab;
         can_tx(&tx_header, tx_payload);
         */

        i++;
        HAL_Delay(1);

        //TIM1->CCR1 = 0;
        //TIM1->CCR2 = 354;

        //HAL_Delay(1000);

        control.Print();
#endif
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
    {
        Error_Handler();
    }
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
    {

    }
    LL_RCC_LSI_Enable();

    /* Wait till LSI is ready */
    while (LL_RCC_LSI_IsReady() != 1)
    {

    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {

    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {

    }
    LL_Init1msTick(72000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(72000000);

    //LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
    LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = { 0 };
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**ADC1 GPIO Configuration
     PA4   ------> ADC1_IN4
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**Common config
     */
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = 1;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    /**Configure Regular Channel
     */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_1CYCLE_5);
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{
    LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**ADC2 GPIO Configuration
     PA5   ------> ADC2_IN5
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**Common config
     */
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
    ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
    LL_ADC_Init(ADC2, &ADC_InitStruct);
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = 1;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
    LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
    /**Configure Regular Channel
     */
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_1CYCLE_5);
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 18;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{
    /* Peripheral clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void)
{
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);
    LL_IWDG_SetReloadCounter(IWDG, 4095);
    while (LL_IWDG_IsReady(IWDG) != 1)
    {
    }

    LL_IWDG_ReloadCounter(IWDG);
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1440 - 1;//50khz;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM1);
    LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = 26;//14; 24th, Mayに焼け死にまくったのを受けてデッド・タイムを若干長くする
    // 実演用：22，短め
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**TIM1 GPIO Configuration
     PB13   ------> TIM1_CH1N
     PB14   ------> TIM1_CH2N
     PA8   ------> TIM1_CH1
     PA9   ------> TIM1_CH2
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**TIM2 GPIO Configuration
     PA0-WKUP   ------> TIM2_CH1
     PA1   ------> TIM2_CH2
     */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 0xffff;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM2);
    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    /* TIM3 interrupt Init */
    NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM_InitStruct.Prescaler = 72 - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1000 - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM3, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM3);
    LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM3);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 460800;
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
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE()
    ;

    /* DMA interrupt init */
    /* DMA1_Channel4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief WWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_WWDG_Init(void)
{
    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);

    /* WWDG interrupt Init */
    NVIC_SetPriority(WWDG_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(WWDG_IRQn);

    LL_WWDG_SetCounter(WWDG, 64);
    LL_WWDG_Enable(WWDG);
    LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_1);
    LL_WWDG_SetWindow(WWDG, 64);
    LL_WWDG_EnableIT_EWKUP(WWDG);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    LL_EXTI_InitTypeDef EXTI_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

    /**/
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_14);

    /**/
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6 | LL_GPIO_PIN_7);

    /**/
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**/
    LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE2);

    /**/
    LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE3);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_2;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    //LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_DOWN);
    /**/
    //LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_DOWN);
    /**/
    //LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
    /**/
    //LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
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
