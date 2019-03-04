#ifndef _LED_H
#define _LED_H

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


#ifdef __cplusplus
 extern "C" {
#endif

#define LED_DURATION 25

#define LED_STAT_ON_DUR 1900
#define LED_STAT_OFF_DUR 100

void led_on(void);
void led_process(void);

#ifdef __cplusplus
}
#endif

#endif
