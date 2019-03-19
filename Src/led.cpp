/*
 * MD201x firmware
 *
 * led.cpp
 */

#include "led.hpp"

namespace led
{
    uint32_t led_can_laston = 0;
    uint32_t led_can_lastoff = 0;

    uint32_t led_stat_laston = 0;
    uint32_t led_stat_lastoff = 0;

    uint32_t led_error_laston = 0;
    uint32_t led_error_lastoff = 0;
    uint8_t led_error_cnt = 0;

    lighting_mode mode = lighting_mode::shutdown;
}

// Try to turn on CAN LED
void led::turn_on_can_led(void)
{
    // Make sure the LED has been off for at least LED_DURATION before turning on again
    // This prevents a solid status LED on a busy canbus
    if (led::led_can_laston == 0 && HAL_GetTick() - led::led_can_lastoff > led::led_can_duration)
    {
        GPIO_LEDCAN->BSRR = GPIO_BSRR_BS_LEDCAN;
        led::led_can_laston = HAL_GetTick();
    }
}

// handles all the light show lol
void led::process(void)
{
    // If LED has been on for long enough, turn it off
    auto now_time = HAL_GetTick();

    if (led::led_can_laston > 0 && now_time - led::led_can_laston > led::led_can_duration)
    {
        GPIO_LEDCAN->BSRR = GPIO_BSRR_BR_LEDCAN;
        led::led_can_laston = 0;
        led::led_can_lastoff = now_time;
    }

    if (led::led_stat_laston > 0 && now_time - led::led_stat_laston > led::led_stat_on_duration)
    {
        GPIO_LED0->BSRR = GPIO_BSRR_BR_LED0;
        led::led_stat_laston = 0;
        led::led_stat_lastoff = now_time;
    }

    if (led::led_stat_laston == 0 && now_time - led::led_stat_lastoff > led::led_stat_off_duration)
    {
        GPIO_LED0->BSRR = GPIO_BSRR_BS_LED0;
        led::led_stat_laston = now_time;
    }

    if (led::led_error_laston > 0 && ((now_time - led::led_error_laston) > led::led_error_on_duration))
    {
        if (led::mode != led::lighting_mode::shutdown)
        {
            // turn off led2 (aka error_led)
            GPIO_LED2->BSRR = GPIO_BSRR_BR_LED2;
            led::led_error_laston = 0;
            led::led_error_lastoff = now_time;
        }
    }

    if (led::led_error_laston == 0 && now_time - led::led_error_lastoff > led::led_error_off_duration)
    {
        led::led_error_cnt++;
        if (led::led_error_cnt >= 5)
        {
            led::led_error_cnt = 1;
        }

        if (led::mode == led::lighting_mode::shutdown)
        {
            GPIO_LED2->BSRR = GPIO_BSRR_BS_LED2;
            //led::led_error_laston = now_time;
            led::led_error_cnt = 0;
        }
        else if (led::mode == led::lighting_mode::operational)
        {
            GPIO_LED2->BSRR = GPIO_BSRR_BR_LED2;
            led::led_error_laston = 0;
            led::led_error_lastoff = now_time;
            led::led_error_cnt = 0;
        }
        else if ((led::mode == led::lighting_mode::error_0) && (led::led_error_cnt > 1))
        {
            led::led_error_laston = now_time;
        }
        else if ((led::mode == led::lighting_mode::error_1) && (led::led_error_cnt > 2))
        {
            led::led_error_laston = now_time;
        }
        else if ((led::mode == led::lighting_mode::error_2) && (led::led_error_cnt > 3))
        {
            led::led_error_laston = now_time;
        }
        else if ((led::mode == led::lighting_mode::error_3) && (led::led_error_cnt > 4))
        {
            led::led_error_laston = now_time;
        }
        else
        {
            GPIO_LED2->BSRR = GPIO_BSRR_BS_LED2;
            led::led_error_laston = now_time;
        }
    }

    if (led::mode == led::lighting_mode::shutdown)
    {
        GPIO_LED2->BSRR = GPIO_BSRR_BS_LED2;
        led::led_error_laston = now_time;
        //led::led_error_lastoff = 0;
        led::led_error_cnt = 0;
    }
    if (led::mode == led::lighting_mode::operational)
    {
        GPIO_LED2->BSRR = GPIO_BSRR_BR_LED2;
        led::led_error_laston = 0;
        led::led_error_lastoff = now_time;
        led::led_error_cnt = 0;
    }
}

