/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "logger_types.h"
#include "logger_state_machine.h"
#include "gps.h"
#include "lidar.h"
#include "bdbg.h"
#include "bdbg_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_lpuart1_rx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;



/* USER CODE BEGIN PV */
uint16_t adc_value=0;

/*
 * Основной объект стейт-машины LOGGER.
 * Содержит текущее состояние всей логики программы.
 */
LoggerStateMachine logger;


/*
 * Глобальный счётчик секунд RTC.
 *
 * Увеличивается на единицу при каждом прерывании
 * RTC WakeUp Timer (период 1 секунда).
 *
 * Значение никогда не обнуляется программой.
 * Основная стейт-машина только читает счётчик.
 *
 * volatile необходим, поскольку счётчик изменяется
 * в обработчике прерывания RTC.
 */
volatile uint32_t rtc_seconds = 0U;


/*
 * Размер DMA-буфера приёма USART2 (RadioLine).
 *
 * В этот буфер DMA непосредственно записывает байты,
 * поступающие по USART2.
 */
#define RADIOLINE_DMA_RX_BUFFER_SIZE    256U

/*
 * Размер общего буфера RadioLine.
 *
 * В него будем последовательно накапливать весь burst:
 * один пакет либо несколько подряд идущих копий пакета.
 *
 * Конкретный размер позже при необходимости скорректируем
 * по максимальному размеру команды протокола.
 */
#define RADIOLINE_RX_BUFFER_SIZE        2048U

/*
 * Рабочий буфер DMA для USART2.
 *
 * HAL_UARTEx_ReceiveToIdle_DMA() будет принимать сюда
 * очередную порцию данных от RadioLine.
 */
static uint8_t radioline_dma_rx_buffer[RADIOLINE_DMA_RX_BUFFER_SIZE];


/*
 * Общий накопительный буфер RadioLine.
 * После каждого события приёма данные из DMA-буфера
 * будут добавляться сюда. После 100 мс тишины этот
 * буфер будет готов к обработке в COMMAND_INFO.
 */
uint8_t radioline_rx_buffer[RADIOLINE_RX_BUFFER_SIZE];


/*
 * Текущее количество принятых байтов в общем
 * накопительном буфере RadioLine.
 */
volatile uint16_t radioline_rx_length = 0U;


/*
 * Флаг завершения приёма burst RadioLine.
 *
 * 0 - burst ещё принимается;
 * 1 - после последнего принятого байта прошло 100 мс.
 */
volatile uint8_t com_info = 0U;

/*
 * ============================================================================
 * Флаг срабатывания общего таймера контроля ответа прибора TIM3.
 *
 * TIM3 используется для ограничения времени ожидания ответа
 * от текущего прибора.
 *
 * Сам callback таймера НЕ принимает никаких решений и НЕ изменяет
 * состояние LOGGER. Он только устанавливает этот флаг.
 *
 * Обработка таймаута выполняется позже основной стейт-машиной,
 * которая по logger->dev_type определяет, ответ какого прибора ожидался.
 *
 * volatile необходим, поскольку значение изменяется из прерывания,
 * а читается из основного цикла программы.
 *
 * 0 - таймаут не произошёл;
 * 1 - время ожидания ответа прибора истекло.
 * ============================================================================
 */
volatile uint8_t device_timeout = 0U;


/* Буферы и управление GPS вынесены в gps.c. */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/*
*/

#include <string.h>
#include <stdint.h>


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /*
		 * Общий callback приёма данных UART.
		 *
		 * USART2 — приём команд RadioLine.
		 * USART3 — приём данных GPS.
		 *
		 * Обработка каждого интерфейса выполняется
		 * независимо от остальных.
		 */

		/*
		* Последняя обработанная позиция в циклическом DMA-буфере USART2.
		*
		* DMA USART2 работает в режиме DMA_CIRCULAR.
		* Поэтому после события IDLE DMA не перезапускается, а продолжает
		* записывать следующие байты дальше в radioline_dma_rx_buffer.
		*
		* Эта переменная позволяет определить, какие именно байты появились
		* с момента предыдущего вызова HAL_UARTEx_RxEventCallback().
		*/
		static uint16_t radioline_dma_old_pos = 0U;	
			
			
		if (huart->Instance == USART2)
		{
				/*
				 * USART2 RX DMA работает в режиме DMA_CIRCULAR.
				 *
				 * В этом режиме параметр Size НЕ является просто размером
				 * очередной независимой порции данных.
				 *
				 * Size указывает текущую позицию DMA в циклическом буфере:
				 * данные доступны до позиции Size.
				 *
				 * Поэтому необходимо копировать только те байты,
				 * которые появились после предыдущего вызова callback.
				 */

				uint16_t new_data_length = 0U;


				/*
				 * СЛУЧАЙ 1.
				 *
				 * DMA ещё не дошёл до конца буфера.
				 *
				 * Например:
				 *
				 * radioline_dma_old_pos = 0
				 * Size                  = 14
				 *
				 * Значит новые данные находятся:
				 *
				 * radioline_dma_rx_buffer[0 ... 13]
				 */
				if (Size > radioline_dma_old_pos)
				{
						new_data_length = Size - radioline_dma_old_pos;

						/*
						 * Проверяем, помещаются ли новые данные
						 * в накопительный буфер RadioLine.
						 */
						if ((radioline_rx_length + new_data_length) <=
								RADIOLINE_RX_BUFFER_SIZE)
						{
								memcpy(&radioline_rx_buffer[radioline_rx_length],
											 &radioline_dma_rx_buffer[radioline_dma_old_pos],
											 new_data_length);

								radioline_rx_length += new_data_length;
						}
				}


				/*
				 * СЛУЧАЙ 2.
				 *
				 * DMA дошёл до конца циклического буфера и начал запись
				 * снова с его начала.
				 *
				 * Например:
				 *
				 * старая позиция = 250
				 * новая Size     = 10
				 *
				 * Тогда новые данные расположены двумя участками:
				 *
				 * [250 ... 255]
				 *
				 * и
				 *
				 * [0 ... 9]
				 */
				else if (Size < radioline_dma_old_pos)
				{
						uint16_t tail_length =
								RADIOLINE_DMA_RX_BUFFER_SIZE - radioline_dma_old_pos;

						uint16_t head_length = Size;

						new_data_length = tail_length + head_length;

						/*
						 * Проверяем место сразу для обоих участков,
						 * чтобы не записать только половину новых данных.
						 */
						if ((radioline_rx_length + new_data_length) <=
								RADIOLINE_RX_BUFFER_SIZE)
						{
								/*
								 * Сначала копируем остаток данных
								 * от старой позиции до конца DMA-буфера.
								 */
								if (tail_length > 0U)
								{
										memcpy(&radioline_rx_buffer[radioline_rx_length],
													 &radioline_dma_rx_buffer[radioline_dma_old_pos],
													 tail_length);

										radioline_rx_length += tail_length;
								}

								/*
								 * Затем копируем данные, которые DMA уже записал
								 * с начала циклического буфера.
								 */
								if (head_length > 0U)
								{
										memcpy(&radioline_rx_buffer[radioline_rx_length],
													 &radioline_dma_rx_buffer[0],
													 head_length);

										radioline_rx_length += head_length;
								}
						}
				}


				/*
				 * Запоминаем текущую позицию DMA.
				 *
				 * Особый случай Size == RADIOLINE_DMA_RX_BUFFER_SIZE:
				 * DMA дошёл до конца буфера и в circular mode следующая запись
				 * начнётся снова с позиции 0.
				 */
				if (Size >= RADIOLINE_DMA_RX_BUFFER_SIZE)
				{
						radioline_dma_old_pos = 0U;
				}
				else
				{
						radioline_dma_old_pos = Size;
				}


				/*
				 * Таймер 100 мс перезапускаем только тогда,
				 * когда действительно появились новые данные.
				 *
				 * TIM6 используется для определения окончания
				 * сообщения RadioLine по паузе.
				 */
				if (new_data_length > 0U)
				{
						/*
						 * Останавливаем предыдущий отсчёт.
						 */
						HAL_TIM_Base_Stop_IT(&htim6);

						/*
						 * Начинаем отсчёт 100 мс заново.
						 */
						__HAL_TIM_SET_COUNTER(&htim6, 0U);

						HAL_TIM_Base_Start_IT(&htim6);
				}


				/*
				 * ВАЖНО:
				 *
				 * HAL_UARTEx_ReceiveToIdle_DMA() здесь НЕ вызываем.
				 *
				 * USART2 RX DMA настроен в DMA_CIRCULAR.
				 * После IDLE DMA автоматически продолжает работать.
				 *
				 * Повторный вызов HAL_UARTEx_ReceiveToIdle_DMA()
				 * здесь привёл бы к HAL_BUSY, потому что текущий
				 * DMA-приём всё ещё активен.
				 */
		}		
		
		
		
		
    /* GPS: ISR только переносит данные; разбор NMEA выполняется в main loop. */
    if (huart->Instance == USART3)
    {
        GPS_RxEventCallback(Size);
        return;
    }

}


/* Ошибки GPS USART3 сигнализируем стейт-машине без блокировки в ISR. */
/* DMA normal mode: событие полного приёма БДБГ. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) { BDBG_UartErrorCallback(huart); return; }
    if (huart->Instance == USART3)
        GPS_NotifyUartError();
		
		/*
		 * Ошибка приёма данных LIDAR через LPUART1.
		 * Сообщаем модулю LIDAR о неисправности текущего приёма.
		 */
		if (huart->Instance == LPUART1)
		{
				LIDAR_NotifyUartError();
				return;
		}
		
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /*
     * ========================================================================
     * TIM6
     * ========================================================================
     *
     * Таймер окончания burst RadioLine.
     *
     * Если после последнего принятого байта RadioLine прошло 100 мс,
     * считаем текущий burst завершённым.
     *
     * Эту существующую логику не изменяем.
     */
    if (htim->Instance == TIM6)
    {
        /*
         * Останавливаем TIM6.
         *
         * Следующий запуск произойдёт после поступления новых данных
         * RadioLine.
         */
        HAL_TIM_Base_Stop_IT(&htim6);

        /*
         * Сообщаем основному циклу, что накопленный burst RadioLine
         * готов к обработке.
         */
        com_info = 1U;

        return;
    }


    /*
     * ========================================================================
     * TIM3
     * ========================================================================
     *
     * Общий таймер контроля времени ожидания ответа прибора.
     *
     * TIM3 настроен в One Pulse Mode, поэтому после истечения заданного
     * интервала используется как одноразовое событие таймаута.
     *
     * ВАЖНО:
     *
     * В прерывании:
     *
     * - НЕ определяем тип прибора;
     * - НЕ изменяем logger->state;
     * - НЕ отправляем пакеты RadioLine;
     * - НЕ выполняем обработку ошибки;
     * - НЕ запускаем стейт-машину.
     *
     * Callback только сообщает основной программе о том, что время
     * ожидания ответа текущего прибора истекло.
     *
     * Какой прибор ожидался, основная стейт-машина определит
     * по logger->dev_type.
     */
    if (htim->Instance == TIM3)
    {
        /*
         * Фиксируем событие таймаута.
         */
        device_timeout = 1U;

        return;
    }
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
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
	
  /* USER CODE BEGIN 2 */
	
	/*
	* Инициализация программной стейт-машины LOGGER.
	*
	* После этого вызова:
	* req_mode = MODE_TEST;
	* state    = COMMAND_INFO.
	*/
	LoggerStateMachine_Init(&logger);
	GPS_Init();
	/*
	* Инициализация программного модуля LIDAR.
	*
	* Сбрасываем внутренние буферы, счётчики и флаги.
	* Приём данных пока не запускаем.
	*/
	LIDAR_Init();
    BDBG_Init();

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	#ifdef DEBUG_MODE
	HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_RESET);
	#else
	HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_SET);
	#endif
	
	
	/*
	* Запускаем постоянный приём данных RadioLine по USART2 через DMA.
	*
	* HAL_UARTEx_ReceiveToIdle_DMA():
	* - USART2 принимает данные;
	* - DMA записывает их в radioline_dma_rx_buffer;
	* - при событии приёма HAL вызовет HAL_UARTEx_RxEventCallback().
	*
	* Сейчас мы только запускаем механизм приёма.
	* Обработку полученных данных добавим следующим шагом.
	*/
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, radioline_dma_rx_buffer, RADIOLINE_DMA_RX_BUFFER_SIZE) != HAL_OK)
	{
		Error_Handler();
	}

	/*
	* Отключаем прерывание DMA по половине заполненного буфера.
	*
	* Для RadioLine оно нам не требуется. Нас интересуют полученные
	* данные целиком и события окончания текущей порции приёма.
	*/
	__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

			/*
			* Выполняем один шаг основной стейт-машины LOGGER.
			*
			* Функция не блокирует главный цикл и после обработки
			* текущего состояния возвращает управление сюда.
			*/
			LoggerStateMachine_Process(&logger);	
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = BDBG_UART_BAUDRATE;
  huart4.Init.WordLength = BDBG_UART_WORD_LENGTH;
  huart4.Init.StopBits = BDBG_UART_STOP_BITS;
  huart4.Init.Parity = BDBG_UART_PARITY;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  //if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 59, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999; 	
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, LED_Pin|RE485_Pin|DE485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CARD_CS_GPIO_Port, SD_CARD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin RE485_Pin DE485_Pin */
  GPIO_InitStruct.Pin = LED_Pin|RE485_Pin|DE485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CARD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CARD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CARD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_RESET_Pin */
  GPIO_InitStruct.Pin = GPS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_RESET_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Обработка завершения приёма UART через DMA.
 *
 * Для LIDAR используется LPUART1.
 * После приёма 45 байтов управление передаётся
 * модулю lidar.c.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /*
     * Проверяем, что прерывание относится именно к LPUART1.
     * Другие UART здесь не обрабатываем.
     */
    if (huart->Instance == LPUART1)
    {
        /*
         * Модуль LIDAR остановит DMA-приём
         * и установит флаг готовности lidar_info.
         */
        LIDAR_RxCpltCallback();
        return;
    }

    /* UART4: завершён приём ответа БДБГ-15 через DMA. */
    if (huart->Instance == UART4)
    {
        BDBG_RxCompleteCallback(huart);
        return;
    }
}

/*
 * Callback секундного прерывания RTC WakeUp Timer.
 *
 * Вызывается библиотекой HAL после обработки
 * прерывания RTC_WKUP_IRQHandler().
 *
 * Каждое срабатывание соответствует одной секунде.
 *
 * В прерывании выполняется только увеличение счётчика.
 * Никаких переходов состояний, запросов приборов
 * и других операций здесь нет.
 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    if (hrtc->Instance == RTC) rtc_seconds++;
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
