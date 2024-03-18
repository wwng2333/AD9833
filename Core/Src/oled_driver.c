#include "oled_driver.h"
#include "stdlib.h"
#include "spi.h"
#include "u8g2.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t u8x8_gpio_and_delay_hw(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
    break;
  case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
    break;
  case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
    LL_mDelay(1);
    break;
  case U8X8_MSG_DELAY_I2C:      // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
    break;                      // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
  case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
    break;                      // arg_int=1: Input dir with pullup high for I2C clock pin
  case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
    break;                      // arg_int=1: Input dir with pullup high for I2C data pin
  case U8X8_MSG_GPIO_MENU_SELECT:
    u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
    break;
  case U8X8_MSG_GPIO_MENU_NEXT:
    u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
    break;
  case U8X8_MSG_GPIO_MENU_PREV:
    u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
    break;
  case U8X8_MSG_GPIO_MENU_HOME:
    u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
    break;
  default:
    u8x8_SetGPIOResult(u8x8, 1); // default return value
    break;
  }
  return 1;
}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;
 
  switch(msg){
		
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;      
      while( arg_int > 0 ){
				buffer[buf_idx++] = *data;
				data++;
				arg_int--;
			}      
    break;
			
    case U8X8_MSG_BYTE_INIT:
      /* add your custom code to init i2c subsystem */
    break;
		
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
    break;
		
    case U8X8_MSG_BYTE_END_TRANSFER:
      HAL_I2C_Master_Transmit(&hi2c1,u8x8_GetI2CAddress(u8x8), buffer, buf_idx,1000);
    break;
		
    default:
      return 0;
  }
  return 1;
}

void draw(u8g2_t *u8g2)
{
  u8g2_SetFontMode(u8g2, 1); // Transparent
  u8g2_SetFontDirection(u8g2, 0);
  u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
  u8g2_DrawStr(u8g2, 0, 20, "U");

  u8g2_SetFontDirection(u8g2, 1);
  u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
  u8g2_DrawStr(u8g2, 21, 8, "8");

  u8g2_SetFontDirection(u8g2, 0);
  u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
  u8g2_DrawStr(u8g2, 51, 30, "g");
  u8g2_DrawStr(u8g2, 67, 30, "\xb2");

  u8g2_DrawHLine(u8g2, 2, 35, 47);
  u8g2_DrawHLine(u8g2, 3, 36, 47);
  u8g2_DrawVLine(u8g2, 45, 32, 12);
  u8g2_DrawVLine(u8g2, 46, 33, 12);

  u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
  u8g2_DrawStr(u8g2, 1, 54, "github.com/olikraus/u8g2");
}