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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "oled_driver.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRI_WAVE 	0
#define SIN_WAVE 	1
#define SQU_WAVE 	2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BASE_FREQ 1630000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static u8g2_t u8g2;
char sprintf_tmp[16];
uint32_t AD9833_freq = BASE_FREQ;
uint16_t ra_adc, cur_adc;
uint16_t adc_buf[3];
uint16_t PWM1_T_Count, PWM1_D_Count;
float PWM1_Duty, PWM1_Freq, FlowRate;
uint16_t chip_temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Ra_ADC_Read(void);
void Cur_ADC_Read(void);
void AD9833_Write(uint16_t dat);
void AD9833_WaveSet(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 3);
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay_hw);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);

	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableIT_CC1(TIM3);
	LL_TIM_EnableIT_CC2(TIM3);

	//AD9833_WaveSet(AD9833_freq, 0, SQU_WAVE, 0);
	//AD9833_Write(0x01C0); //Sleep
	
	//LL_TIM_EnableAllOutputs(TIM1);
	//LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	//LL_TIM_EnableCounter(TIM1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Cur_ADC_Read();
		HAL_Delay(10);
		Ra_ADC_Read();
		u8g2_FirstPage(&u8g2);
		do
    {
			u8g2_SetFontMode(&u8g2, 1);  // Transparent
			u8g2_SetFontDirection(&u8g2, 0);
			u8g2_SetFont(&u8g2, u8g2_font_9x15_te);
			u8g2_DrawStr(&u8g2, 0, 10, "Freq:");
			sprintf(sprintf_tmp, "%uHz", AD9833_freq);
			u8g2_DrawStr(&u8g2, 45, 10, sprintf_tmp);
			u8g2_DrawStr(&u8g2, 0, 22, "Flow:");
			sprintf(sprintf_tmp, "%.2fL/min", FlowRate);
			u8g2_DrawStr(&u8g2, 45, 22, sprintf_tmp);
			u8g2_DrawStr(&u8g2, 0, 34, "Current:");
			sprintf(sprintf_tmp, "%dmA", cur_adc);
			u8g2_DrawStr(&u8g2, 72, 34, sprintf_tmp);
			u8g2_DrawStr(&u8g2, 0, 46, "Temp:");
			chip_temp = __LL_ADC_CALC_TEMPERATURE(3300, adc_buf[2], LL_ADC_RESOLUTION_12B);
			sprintf(sprintf_tmp, "%dC", chip_temp);
			u8g2_DrawStr(&u8g2, 45, 46, sprintf_tmp);
    } while (u8g2_NextPage(&u8g2));
		LL_mDelay(50);
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

void Ra_ADC_Read(void)
{
	ra_adc = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_buf[0], LL_ADC_RESOLUTION_12B);
	AD9833_freq = (BASE_FREQ - 1) + ra_adc*35;
	AD9833_WaveSet(AD9833_freq, 0, SQU_WAVE, 0);
}

void Cur_ADC_Read(void)
{
	cur_adc = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, adc_buf[1], LL_ADC_RESOLUTION_12B) * 4;
}

void AD9833_Write(uint16_t dat)
{
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
	LL_SPI_TransmitData16(SPI1, dat);
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
}

void AD9833_WaveSet(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase )
{

		int frequence_LSB,frequence_MSB,Phs_data;
		double   frequence_mid,frequence_DATA;
		long int frequence_hex;

		frequence_mid=268435456/25;//For 25Mhz
		frequence_DATA=Freq;
		frequence_DATA=frequence_DATA/1000000;
		frequence_DATA=frequence_DATA*frequence_mid;
		frequence_hex=frequence_DATA;
		frequence_LSB=frequence_hex;
		frequence_LSB=frequence_LSB&0x3fff;
		frequence_MSB=frequence_hex>>14;
		frequence_MSB=frequence_MSB&0x3fff;

		Phs_data=Phase|0xC000;	//Phase
		AD9833_Write(0x0100); //Reset AD9833
		AD9833_Write(0x2100); //Set B28 and RESET to 1

		if(Freq_SFR==0)				  //Set freq to Freq_SFR_0
		{
		 	frequence_LSB=frequence_LSB|0x4000;
		 	frequence_MSB=frequence_MSB|0x4000;
			AD9833_Write(frequence_LSB);
			AD9833_Write(frequence_MSB);
			AD9833_Write(Phs_data);
	    }
		if(Freq_SFR==1)
		{
			 frequence_LSB=frequence_LSB|0x8000;
			 frequence_MSB=frequence_MSB|0x8000;
			AD9833_Write(frequence_LSB);
			AD9833_Write(frequence_MSB);
			AD9833_Write(Phs_data);
		}

		if(WaveMode==TRI_WAVE)
		 	AD9833_Write(0x2002); 
		if(WaveMode==SQU_WAVE)
			AD9833_Write(0x2028); 
		if(WaveMode==SIN_WAVE)
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
