/*
 * MD201x firmware
 *
 * led.hpp
 */

#pragma once

#include "stm32f1xx_hal.h"

#define GPIO_LED0 GPIOA

#define GPIO_BSRR_BS_LED0 GPIO_BSRR_BS6
#define GPIO_BSRR_BR_LED0 GPIO_BSRR_BR6

#define GPIO_LED1 GPIOA

#define GPIO_BSRR_BS_LED1 GPIO_BSRR_BS7
#define GPIO_BSRR_BR_LED1 GPIO_BSRR_BR7

#define GPIO_LED2 GPIOB

#define GPIO_BSRR_BS_LED2 GPIO_BSRR_BS0
#define GPIO_BSRR_BR_LED2 GPIO_BSRR_BR0

#define GPIO_LEDCAN GPIOC

#define GPIO_BSRR_BS_LEDCAN GPIO_BSRR_BS14
#define GPIO_BSRR_BR_LEDCAN GPIO_BSRR_BR14

#define LED_DURATION 25

#define LED_CAN_ON_DUR 1900
#define LED_CAN_OFF_DUR 100

/*
 * error mode 0:
 *
 * |<->|<-500ms
 *
 * +-+
 * | |
 * + +--------
 *
 * mode 1:
 *
 * +-+ +-+
 * | | | |
 * + +-+ +------
 *
 * mode 2:
 *
 * +-+ +-+ +-+
 * | | | | | |
 * + +-+ +-+ +---
 *
 */

namespace led
{

    enum class lighting_mode
    {
        shutdown,
        operational,
        error_0,
        error_1,
        error_2,
        error_3,
    };

    extern lighting_mode mode;

    void turn_on_can_led(void);
    void process(void);

    constexpr auto led_stat_on_duration = 1900;
    constexpr auto led_stat_off_duration = 100;

    constexpr auto led_error_on_duration = 50;
    constexpr auto led_error_off_duration = 200;

    constexpr auto led_can_duration = 25;
}
