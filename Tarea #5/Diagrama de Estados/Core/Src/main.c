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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define lim1 10
#define lim2 50
volatile int limit = lim2;
//USOS DEL PROGRAMA DE LA M�?QUINA DE ESTADO
#define ESTADO_ERROR        0
#define ESTADO_ABIERTO      1
#define ESTADO_CERRADO      2
#define ESTADO_ABRIENDO     3
#define ESTADO_CERRANDO     4
#define ESTADO_INTERMEDIO   5
#define ESTADO_INIT         6
#define FALSE               0
#define TRUE                1
#define inTrue              0
#define inFalse             1
#define TIME_Ca             60

int Func_ESTADO_ERROR(void);
int Func_ESTADO_ABIERTO(void);
int Func_ESTADO_CERRADO(void);
int Func_ESTADO_ABRIENDO(void);
int Func_ESTADO_CERRANDO(void);
int Func_ESTADO_INTERMEDIO(void);
int Func_ESTADO_INIT(void);

volatile int ESTADO_ANTERIOR = ESTADO_INIT;
volatile int ESTADO_ACTUAL = ESTADO_INIT;
volatile int ESTADO_SIGUIENTE = ESTADO_INIT;
volatile int contador = 0;

volatile struct INOUT {
    unsigned int Sc:1;
    unsigned int Sa:1;
    unsigned int Mc:1;
    unsigned int Ma:1;
    unsigned int Bc:1;
    unsigned int Ba:1;
    unsigned int Led:1;
} inout;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
   MX_USART2_UART_Init();
   MX_TIM2_Init();
   /* USER CODE BEGIN 2 */
   HAL_TIM_Base_Start_IT(&htim2);
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	 /* HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	  	  HAL_Delay(delay);
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
	  	  HAL_Delay(delay);*/
	  if (ESTADO_SIGUIENTE == ESTADO_INIT) {
	      ESTADO_SIGUIENTE = Func_ESTADO_INIT();
	  }

	  if (ESTADO_SIGUIENTE == ESTADO_ABIERTO) {
	      ESTADO_SIGUIENTE = Func_ESTADO_ABIERTO();
	  }

	  if (ESTADO_SIGUIENTE == ESTADO_CERRADO) {
	      ESTADO_SIGUIENTE = Func_ESTADO_CERRADO();
	  }

	  if (ESTADO_SIGUIENTE == ESTADO_ABRIENDO) {
	      ESTADO_SIGUIENTE = Func_ESTADO_ABRIENDO();
	  }

	  if (ESTADO_SIGUIENTE == ESTADO_CERRANDO) {
	      ESTADO_SIGUIENTE = Func_ESTADO_CERRANDO();
	  }

	  if (ESTADO_SIGUIENTE == ESTADO_INTERMEDIO) {
	      ESTADO_SIGUIENTE = Func_ESTADO_INTERMEDIO();
	  }

	  if (ESTADO_SIGUIENTE == ESTADO_ERROR) {
	      ESTADO_SIGUIENTE = Func_ESTADO_ERROR();
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */


  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MCA9_Pin|MA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MC_Pin */
  GPIO_InitStruct.Pin = MC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BA_Pin BC_Pin */
  GPIO_InitStruct.Pin = BA_Pin|BC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MCA9_Pin MA_Pin */
  GPIO_InitStruct.Pin = MCA9_Pin|MA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SA_Pin SC_Pin */
  GPIO_InitStruct.Pin = SA_Pin|SC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim2)
{
  /* Prevent unused argument(s) compilation warning */
	/*contador++;
if(contador == limit)
{
// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
 contador = 0;
}*/
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedHalfCpltCallback could be implemented in the user file
   */
	static int cont_int = 0;

	// Condición para reiniciar contador
	if (cont_int == lim1) {
	    contador++;
	    cont_int = 0;
	}

	// Control del LED
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, inout.Led);

	// Control del botón cerrar
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == TRUE) {
	    inout.Bc = TRUE;
	} else {
	    inout.Bc = FALSE;
	}

	// Control del botón abrir
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == TRUE) {
	    inout.Ba = TRUE;
	} else {
	    inout.Ba = FALSE;
	}

	// Control del sensor cerrado
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == TRUE) {
	    inout.Sc = TRUE;
	} else {
	    inout.Sc = FALSE;
	}

	// Control del sensor abierto
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == TRUE) {
	    inout.Sa = TRUE;
	} else {
	    inout.Sa = FALSE;
	}

	// Control del motor abrir
	if (inout.Ma == TRUE) {
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	} else {
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
	}

	// Control del motor cerrar
	if (inout.Mc == TRUE) {
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	} else {
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	}

	// Incremento del contador
	cont_int++;
}
/*
void cambio_de_valor(int *variable)
{
	static int status = 0;
	if(*variable <= 1000 && status == 0)
	{
		*variable = *variable + lim1;
		status = 1;
	}
	else
	{
		*variable = lim1;
		status = 0;
	}
}
*/
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if (GPIO_Pin == B1_Pin)
  {
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }

}*/
int Func_ESTADO_ERROR(void) {
    for (;;) {
        inout.Led = TRUE;
    }
}

int Func_ESTADO_ABIERTO(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_ABIERTO;
    inout.Ma = FALSE;
    inout.Mc = FALSE;
    inout.Led = 1;
    contador = 0; // reset de contador

    for (;;) {
        if ((contador > TIME_Ca - 1) || (inout.Bc == TRUE)) {
            return ESTADO_CERRANDO;
        }
    }
}

int Func_ESTADO_CERRADO(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_CERRADO;
    inout.Mc = 0;
    inout.Ma = 0;
    inout.Led = 1;

    for (;;) {
        if (inout.Ba == TRUE) {
            return ESTADO_ABRIENDO;
        }
    }
}

int Func_ESTADO_ABRIENDO(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_ABRIENDO;
    inout.Led = FALSE;
    inout.Mc = FALSE;
    inout.Ma = TRUE;

    for (;;) {
        if (inout.Sa == TRUE) {
            return ESTADO_ABIERTO;
        } else if (inout.Bc == TRUE) {
            return ESTADO_CERRANDO;
        }
    }
}

int Func_ESTADO_CERRANDO(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_CERRANDO;
    inout.Mc = TRUE;
    inout.Ma = FALSE;
    inout.Led = FALSE;

    for (;;) {
        if (inout.Sc == TRUE) {
            return ESTADO_CERRADO;
        } else if (inout.Ba == TRUE) {
            return ESTADO_ABRIENDO;
        }
    }
}

int Func_ESTADO_INTERMEDIO(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_INTERMEDIO;
    inout.Mc = FALSE;
    inout.Ma = 0;
    inout.Led = TRUE;

    for (;;) {
        if (inout.Ba == TRUE) {
            return ESTADO_ABRIENDO;
        }

        if (inout.Bc == TRUE) {
            return ESTADO_CERRANDO;
        }
    }
}

int Func_ESTADO_INIT(void) {
    ESTADO_ANTERIOR = ESTADO_ACTUAL;
    ESTADO_ACTUAL = ESTADO_INIT;
    inout.Led = 1;
    inout.Ma = FALSE;
    inout.Mc = FALSE;

    for (;;) {
        if ((inout.Sa == 1) && (inout.Sc == 1)) {
            return ESTADO_ERROR;
        }

        if ((inout.Sa == 1) && (inout.Sc == 0)) {
            return ESTADO_ABIERTO;
        }

        if ((inout.Sc == 1) && (inout.Sa == 0)) {
            return ESTADO_CERRADO;
        }

        if ((inout.Sa == 0) && (inout.Sc == 0)) {
            return ESTADO_INTERMEDIO;
        }
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
