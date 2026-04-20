/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Programa principal
  *
  * Esse código lê temperatura do sensor BMP180 via I2C (usando DMA),
  * aplica uma máquina de estados baseada na temperatura e controla:
  *  - PWM (velocidade de um cooler, por exemplo)
  *  - LEDs indicadores de estado
  *  - Envia dados via UART
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bmp180.h"   // Driver do sensor BMP180
#include <stdio.h>    // Para printf / snprintf
#include <string.h>   // Manipulação de strings
/* USER CODE END Includes */


/* USER CODE BEGIN PV */

/**
 * Máquina de estados baseada na temperatura
 */
typedef enum {
    ESTADO_MINIMO = 0,   // Temperatura baixa → PWM desligado
    ESTADO_MEDIO,        // Temperatura média → PWM intermediário
    ESTADO_RAPIDO        // Temperatura alta → PWM máximo
} EstadoTemperatura;

EstadoTemperatura estado_atual = ESTADO_MINIMO;   // Estado atual
EstadoTemperatura proximo_estado = ESTADO_MINIMO; // Próximo estado

BMP180_HandleTypedef bmp;  // Estrutura do sensor BMP180
float temperatura;         // Variável para armazenar temperatura lida

char msg[50];              // Buffer para mensagem UART

volatile uint8_t bmp_ready = 0; // Flag de controle (não usada aqui diretamente)

/* USER CODE END PV */


/* USER CODE BEGIN 0 */
void SystemClock_Config(void);
/**
 * Redireciona printf para UART1
 * Assim você pode usar printf() normalmente
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 0 */


/**
  * @brief  Função principal
  */
int main(void)
{
  /* Inicialização básica da HAL */
  HAL_Init();

  /* Configuração do clock */
  SystemClock_Config();

  /* Inicialização dos periféricos */
  MX_GPIO_Init();        // GPIO (LEDs etc.)
  MX_DMA_Init();         // DMA
  MX_I2C1_Init();        // I2C (BMP180)
  MX_USART1_UART_Init(); // UART
  MX_TIM1_Init();        // Timer (PWM)

  /* USER CODE BEGIN 2 */

  // Inicializa o sensor BMP180
  BMP180_Init(&bmp, &hi2c1);

  // Inicia PWM no canal 1 do TIM1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // Habilita saída PWM (necessário para TIM1 - advanced timer)
  __HAL_TIM_MOE_ENABLE(&htim1);

  /* USER CODE END 2 */

  /* Loop infinito */
  while (1)
  {
    // --- Leitura da temperatura via DMA ---
    if (BMP180_ReadTemperature_DMA(&bmp, &temperatura) == HAL_OK)
    {
        /**
         * TRANSIÇÃO DE ESTADO
         * Define o próximo estado baseado na temperatura
         */
        if (temperatura < 26.0)
        {
            proximo_estado = ESTADO_MINIMO;
        }
        else if (temperatura < 29.0)
        {
            proximo_estado = ESTADO_MEDIO;
        }
        else
        {
            proximo_estado = ESTADO_RAPIDO;
        }

        // Atualiza estado atual
        estado_atual = proximo_estado;

        uint32_t pwm_atual = 0; // Duty cycle atual

        /**
         * AÇÕES DE CADA ESTADO
         */
        switch (estado_atual)
        {
            case ESTADO_MINIMO:
                pwm_atual = 0; // PWM desligado

                // LED verde ON
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
                break;

            case ESTADO_MEDIO:
                pwm_atual = 500; // ~50% duty cycle

                // LED amarelo ON
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
                break;

            case ESTADO_RAPIDO:
                pwm_atual = 999; // ~100% duty cycle

                // LED vermelho ON
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
                break;
        }

        // Atualiza valor do PWM no timer
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_atual);

        /**
         * FORMATAÇÃO DA TEMPERATURA
         * Separa parte inteira e decimal
         */
        int32_t p_int = (int32_t)temperatura;
        int32_t p_dec = (int32_t)((temperatura - p_int) * 100);

        if (p_dec < 0) p_dec *= -1; // Corrige decimal negativo

        /**
         * Monta string para envio via UART
         */
        int len = snprintf(msg, sizeof(msg),
                          "Temp: %ld.%02ld C | PWM: %lu\r\n",
                          p_int, p_dec, pwm_atual);

        // Envia mensagem via UART
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
    }

    // Delay entre leituras
    HAL_Delay(500);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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