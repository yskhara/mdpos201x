#include <led.hpp>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "can.hpp"
#include "conf.h"
#include "motor_ctrl.hpp"

extern CAN_HandleTypeDef hcan;
CAN_FilterTypeDef filter;
uint32_t prescaler;
enum can_bus_state bus_state;

CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];

static constexpr uint8_t cmd_shutdown = 0x00;
static constexpr uint8_t cmd_recover = 0x01;
static constexpr uint8_t cmd_home = 0x10;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_payload[CAN_MTU];

    uint32_t status = can_rx(&rx_header, rx_payload);

    if (status != HAL_OK)
    {
        // or raise an error
        return;
    }
    //led::turn_on_can_led();

    /*
    if ((rx_header.IDE != 0) || (rx_header.RTR != 0))
    {
        return;
    }
    */

    if ((rx_header.StdId == can_id_cmd) && (rx_header.DLC == 1))
    {
        uint8_t cmd;
        can_unpack(rx_payload, cmd);

        switch (cmd)
        {
            case cmd_shutdown:
                control.Shutdown();
                break;
            case cmd_recover:
                control.Recover();
                break;
#ifdef CTRL_POS
            case cmd_home:
                control.Home();
                break;
#endif
            default:
                break;
        }

        led::turn_on_can_led();
    }
    else if ((rx_header.StdId == can_id_vel) && (rx_header.DLC == 4))
    {
        float vel_cmd;
        can_unpack(rx_payload, vel_cmd);
        control.SetTarget(vel_cmd);

        led::turn_on_can_led();
    }
    else
    {
        led::process();
        return;
    }

    led::process();
}
/*
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) != 0)
    {

    }
}
*/

void can_init(void)
{
    // default to 125 kbit/s
    prescaler = 18;
    hcan.Instance = CAN1;
    bus_state = OFF_BUS;

    //can_id_cmd = confStruct.can_id_cmd;
    //can_id_vel = can_id_cmd + 1;
    //can_id_stat = can_id_cmd + 3;
    //can_id_global = confStruct.can_id_global;
}

void can_read_conf(void)
{
    can_id_cmd = confStruct.can_id_cmd;
    can_id_vel = confStruct.can_id_vel;
    can_id_stat = confStruct.can_id_stat;
}

void can_set_filter(uint32_t id, uint32_t mask)
{
    // see page 825 of RM0091 for details on filters
    // set the standard ID part
    filter.FilterIdHigh = (id & 0x7FF) << 5;
    // add the top 5 bits of the extended ID
    filter.FilterIdHigh += (id >> 24) & 0xFFFF;
    // set the low part to the remaining extended ID bits
    filter.FilterIdLow += ((id & 0x1FFFF800) << 3);

    // set the standard ID part
    filter.FilterMaskIdHigh = (mask & 0x7FF) << 5;
    // add the top 5 bits of the extended ID
    filter.FilterMaskIdHigh += (mask >> 24) & 0xFFFF;
    // set the low part to the remaining extended ID bits
    filter.FilterMaskIdLow += ((mask & 0x1FFFF800) << 3);

    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.SlaveStartFilterBank = 0;
    filter.FilterActivation = ENABLE;

    if (bus_state == ON_BUS)
    {
        HAL_CAN_ConfigFilter(&hcan, &filter);
    }
}

void can_enable(void)
{
    if (bus_state == OFF_BUS)
    {
        hcan.Init.Prescaler = prescaler;
        hcan.Init.Mode = CAN_MODE_NORMAL;
        hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
        hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
        hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
        hcan.Init.TimeTriggeredMode = DISABLE;
        hcan.Init.AutoBusOff = DISABLE;
        hcan.Init.AutoWakeUp = DISABLE;
        hcan.Init.AutoRetransmission = ENABLE;
        hcan.Init.ReceiveFifoLocked = DISABLE;
        hcan.Init.TransmitFifoPriority = DISABLE;
        //hcan.pTxMsg = NULL;
        HAL_CAN_Init(&hcan);
        bus_state = ON_BUS;
        can_set_filter(0, 0);

        /* Start the CAN peripheral */
        if (HAL_CAN_Start(&hcan) != HAL_OK)
        {
            /* Start Error */
            Error_Handler();
        }

        /* Activate CAN RX notification */
        if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
            /* Notification Error */
            Error_Handler();
        }
    }
}

void can_disable(void)
{
    if (bus_state == ON_BUS)
    {
        // do a bxCAN reset (set RESET bit to 1)
        hcan.Instance->MCR |= CAN_MCR_RESET;
        bus_state = OFF_BUS;
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void can_set_bitrate(enum can_bitrate bitrate)
{
    if (bus_state == ON_BUS)
    {
        // cannot set bitrate while on bus
        return;
    }

    switch (bitrate)
    {
        case CAN_BITRATE_10K:
            prescaler = 450;
            break;
        case CAN_BITRATE_20K:
            prescaler = 225;
            break;
        case CAN_BITRATE_50K:
            prescaler = 90;
            break;
        case CAN_BITRATE_100K:
            prescaler = 45;
            break;
        case CAN_BITRATE_125K:
            prescaler = 36;
            break;
        case CAN_BITRATE_250K:
            prescaler = 18;
            break;
        case CAN_BITRATE_500K:
            prescaler = 9;
            break;
        case CAN_BITRATE_750K:
            prescaler = 6;
            break;
        case CAN_BITRATE_1000K:
            prescaler = 4;
            break;
    }
}

void can_set_silent(uint8_t silent)
{
    if (bus_state == ON_BUS)
    {
        // cannot set silent mode while on bus
        return;
    }
    if (silent)
    {
        hcan.Init.Mode = CAN_MODE_SILENT;
    }
    else
    {
        hcan.Init.Mode = CAN_MODE_NORMAL;
    }
}

uint32_t can_tx(CAN_TxHeaderTypeDef *tx_header, uint8_t (&buf)[CAN_MTU])
{
    uint32_t status;

    // transmit can frame
    //hcan.pTxMsg = tx_msg;
    //status = HAL_CAN_Transmit(&hcan, timeout);
    uint32_t tx_mailbox;
    status = HAL_CAN_AddTxMessage(&hcan, tx_header, buf, &tx_mailbox);

    //led::turn_on_can_led();
    return status;
}

uint32_t can_rx(CAN_RxHeaderTypeDef *rx_header, uint8_t (&buf)[CAN_MTU])
{
    uint32_t status;

    //hcan.pRxMsg = rx_msg;
    //status = HAL_CAN_Receive(&hcan, CAN_FIFO0, timeout);

    status = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, rx_header, buf);

    //led::turn_on_can_led();
    return status;
}

uint8_t is_can_msg_pending(uint8_t fifo)
{
    if (bus_state == OFF_BUS)
    {
        return 0;
    }
    return (HAL_CAN_GetRxFifoFillLevel(&hcan, fifo) > 0);
}

