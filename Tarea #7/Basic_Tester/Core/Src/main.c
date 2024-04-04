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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESTADO_INICIAL  	1
#define ESTADO_MEDIDOR  	0
#define ESTADO_INTERMEDIO  	2
#define MEDIDOR_ON  		1
#define MEDIDOR_OFF  		0
#define CAL 			1.625
#define TRUE  			1
#define FALSE 			0
#define MUESTRA      		20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int Func_ESTADO_INICIAL(void);
int Func_ESTADO_MEDIDOR(void);
int Func_ESTADO_INTERMEDIO(void);
char VOL_STR[5]; //String del Voltage
unsigned int Estado_Medidor;
volatile int ESTADO_ANTERIOR = ESTADO_INICIAL;
volatile int ESTADO_ACTUAL = ESTADO_INICIAL;
volatile int ESTADO_SIGUIENTE = ESTADO_INICIAL;
volatile struct INOUT
{
    unsigned int Test:1;
} inout;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2,  (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  lcd_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(ESTADO_SIGUIENTE == ESTADO_INICIAL)
	  	{
	  		ESTADO_SIGUIENTE = Func_ESTADO_INICIAL();
	  	}
	  	if(ESTADO_SIGUIENTE == ESTADO_INTERMEDIO)
	  	{
	  		ESTADO_SIGUIENTE = Func_ESTADO_INTERMEDIO();
	  	}
	  	if(ESTADO_SIGUIENTE == ESTADO_MEDIDOR)
	  	{
	  		ESTADO_SIGUIENTE = Func_ESTADO_MEDIDOR();
	  	}
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int Func_ESTADO_INICIAL(void)
{
    // Actualiza el estado anterior y actual
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_INICIAL;
    // Apaga el medidor
    Estado_Medidor = MEDIDOR_OFF;
    // Bucle infinito para mantener el estado
    for(;;)
    {
        // Si la señal de test es verdadera, cambia al estado intermedio
        if(inout.Test == TRUE)
        {
            return ESTADO_INTERMEDIO;
        }
    }
}

int Func_ESTADO_INTERMEDIO(void)
{
    // Actualiza el estado anterior y actual
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_INTERMEDIO;
    // Bucle infinito para mantener el estado
    for(;;)
    {
        // Si la señal de test es falsa y el estado anterior era inicial, cambia al estado del medidor
        if((inout.Test == FALSE) && (ESTADO_ANTERIOR == ESTADO_INICIAL))
        {
            return ESTADO_MEDIDOR;
        }
        // Si la señal de test es falsa y el estado anterior era del medidor, regresa al estado inicial
        if((inout.Test == FALSE) && (ESTADO_ANTERIOR == ESTADO_MEDIDOR))
        {
            return ESTADO_INICIAL;
        }
    }
}

int Func_ESTADO_MEDIDOR(void)
{
    // Actualiza el estado anterior y actual
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_MEDIDOR;
    // Enciende el medidor
    Estado_Medidor = MEDIDOR_ON;
    // Bucle infinito para mantener el estado
    for(;;)
    {
        // Si la señal de test es verdadera, regresa al estado intermedio
        if(inout.Test == TRUE)
        {
            return ESTADO_INTERMEDIO;
        }
        // Limpia la pantalla del LCD y muestra el voltaje
        lcd_clear();
        lcd_enviar("Voltage:", 0, 2); // Palabra Voltage, Fila 0, Columna 2
        lcd_enviar(VOL_STR, 1, 2);
        HAL_Delay(1000); // Espera 1 segundo
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Lee el estado del pin de entrada y actualiza inout.Test
    inout.Test = (HAL_GPIO_ReadPin(PUERTO_PB_A, PIN_PB_A) == TRUE) ? TRUE : FALSE;

    // Variables locales estáticas para el cálculo del voltaje
    static unsigned cont_VOL = 0;
    static float Media = 0.0;
    static float Voltage = 0.00;
    static uint32_t MED_ADC = 0; // Medir ADC

    // Realiza el cálculo del voltaje solo cuando el medidor está encendido
    if (Estado_Medidor == MEDIDOR_ON)
    {
        // Inicia la conversión ADC
        HAL_ADC_Start_IT(&hadc1);
        MED_ADC = HAL_ADC_GetValue(&hadc1);

        // Si se han recolectado suficientes muestras, calcula el voltaje
        if (cont_VOL >= MUESTRA)
        {
            Media = Media / cont_VOL;
            Media = sqrtf(Media) * 0.0005;
            Voltage = Media * CAL;
            cont_VOL = 0;
            Media = 0;
        }
        else
        {
            MED_ADC = MED_ADC * MED_ADC;
            Media += MED_ADC;
            cont_VOL++;
            MED_ADC = 0;
        }

        // Detiene la conversión ADC
        HAL_ADC_Stop_IT(&hadc1);
    }
    else
    {
        // Detiene la conversión ADC si el medidor está apagado
        HAL_ADC_Stop_IT(&hadc1);
    }
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
