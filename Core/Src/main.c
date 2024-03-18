/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "oled_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRI_WAVE 	0  		//输出三角�?
#define SIN_WAVE 	1		//输出正弦�?
#define SQU_WAVE 	2		//输出方波
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static u8g2_t u8g2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AD9833_Write(uint16_t dat);
void AD9833_WaveSeting(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay_hw);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);

	AD9833_WaveSeting(1700000, 0, SQU_WAVE, 0);
	//AD9833_Write(0x01C0); //Sleep
	
	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		u8g2_FirstPage(&u8g2);
		do
    {
			draw(&u8g2);
    } while (u8g2_NextPage(&u8g2));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void AD9833_Write(uint16_t dat)
{
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	LL_SPI_TransmitData16(SPI1, dat);
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
}

/*
*********************************************************************************************************
*	�? �? �?: AD9833_WaveSeting
*	功能说明: 向SPI总线发�??16个bit数据
*	�?    �?: 1.Freq: 频率�?, 0.1 hz - 12Mhz
			  2.Freq_SFR: 0 �? 1
			  3.WaveMode: TRI_WAVE(三角�?),SIN_WAVE(正弦�?),SQU_WAVE(方波)
			  4.Phase : 波形的初相位
*	�? �? �?: �?
*********************************************************************************************************
*/ 
void AD9833_WaveSeting(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase )
{

		int frequence_LSB,frequence_MSB,Phs_data;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;

		/*********************************计算频率�?16进制�?***********************************/
		frequence_mid=268435456/25;//适合25M晶振
		//如果时钟频率不为25MHZ，修改该处的频率值，单位MHz ，AD9833�?大支�?25MHz
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1000000;
		frequence_DATA=frequence_DATA*frequence_mid;
		frequence_hex=frequence_DATA;  //这个frequence_hex的�?�是32位的�?个很大的数字，需要拆分成两个14位进行处理；
		frequence_LSB=frequence_hex; //frequence_hex�?16位�?�给frequence_LSB
		frequence_LSB=frequence_LSB&0x3fff;//去除�?高两位，16位数换去掉高位后变成�?14�?
		frequence_MSB=frequence_hex>>14; //frequence_hex�?16位�?�给frequence_HSB
		frequence_MSB=frequence_MSB&0x3fff;//去除�?高两位，16位数换去掉高位后变成�?14�?

		Phs_data=Phase|0xC000;	//相位�?
		AD9833_Write(0x0100); //复位AD9833,即RESET位为1
		AD9833_Write(0x2100); //选择数据�?次写入，B28位和RESET位为1

		if(Freq_SFR==0)				  //把数据设置到设置频率寄存�?0
		{
		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			 //使用频率寄存�?0输出波形
			AD9833_Write(frequence_LSB); //L14，�?�择频率寄存�?0的低14位数据输�?
			AD9833_Write(frequence_MSB); //H14 频率寄存器的�?14位数据输�?
			AD9833_Write(Phs_data);	//设置相位
			//AD9833_Write(0x2000); /**设置FSELECT位为0，芯片进入工作状�?,频率寄存�?0输出波形**/
	    }
		if(Freq_SFR==1)				//把数据设置到设置频率寄存�?1
		{
			 frequence_LSB=frequence_LSB|0x8000;
			 frequence_MSB=frequence_MSB|0x8000;
			//使用频率寄存�?1输出波形
			AD9833_Write(frequence_LSB); //L14，�?�择频率寄存�?1的低14位输�?
			AD9833_Write(frequence_MSB); //H14 频率寄存�?1�?
			AD9833_Write(Phs_data);	//设置相位
			//AD9833_Write(0x2800); /**设置FSELECT位为0，设置FSELECT位为1，即使用频率寄存�?1的�?�，芯片进入工作状�??,频率寄存�?1输出波形**/
		}

		if(WaveMode==TRI_WAVE) //输出三角波波�?
		 	AD9833_Write(0x2002); 
		if(WaveMode==SQU_WAVE)	//输出方波波形
			AD9833_Write(0x2028); 
		if(WaveMode==SIN_WAVE)	//输出正弦波形
			AD9833_Write(0x2000); 

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
