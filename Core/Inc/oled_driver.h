#ifndef __OLED_DRIVER_H__
#define __OLED_DRIVER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "u8g2.h"

uint8_t u8x8_gpio_and_delay_hw(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void draw(u8g2_t *u8g2);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_DRIVER_H__ */
