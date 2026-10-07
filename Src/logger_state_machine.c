/* ============================================================================
 * LOGGER
 *
 * Реализация основной стейт-машины логгера.
 * ========================================================================== */

#include "main.h"
#include "logger_state_machine.h"
#include "gps.h"

/*
 * Коэффициент дублирования пакетов RadioLine.
 *
 * Начальное значение — 2.
 * В дальнейшем изменяется командой оператора с земли.
 */
static uint8_t repeat_factor = LOGGER_DEFAULT_REPEAT_FACTOR;

/*
 * Общий накопительный буфер RadioLine.
 *
 * Сам буфер создаётся в main.c и заполняется
 * в HAL_UARTEx_RxEventCallback().
 *
 * Обработка содержимого выполняется только
 * в состоянии COMMAND_INFO.
 */
extern uint8_t radioline_rx_buffer[];


/*
 * Количество байтов, накопленных в буфере RadioLine.
 *
 * Значение изменяется при приёме данных USART2.
 */
extern volatile uint16_t radioline_rx_length;


/*
 * Флаг завершения приёма пакета RadioLine.
 *
 * Устанавливается обработчиком TIM6 после 100 мс
 * отсутствия новых данных RadioLine.
 *
 * 0 - приём ещё не завершён;
 * 1 - накопленные данные готовы к обработке
 *     состоянием COMMAND_INFO.
 */
extern volatile uint8_t com_info;


/* ============================================================================
 * Инициализация стейт-машины LOGGER.
 *
 * Вызывается один раз после инициализации периферии STM32.
 *
 * Согласно принятой логике LOGGER после включения:
 *
 * 1. Система находится в режиме предполетной проверки MODE_TEST.
 * 2. Основная стейт-машина находится в COMMAND_INFO и ожидает
 *    команду на начало предполетной проверки.
 * 3. Активного ожидания ответа от приборов еще нет.
 * 4. БДБГ по умолчанию подготовлен к обычному запросу ПАЕД.
 * ========================================================================== */
void LoggerStateMachine_Init(LoggerStateMachine *logger)
{
    /*
     * Защита от передачи нулевого указателя.
     *
     * Если функция по ошибке будет вызвана с NULL, просто выходим,
     * не обращаясь к памяти по недопустимому адресу.
     */
    if (logger == 0)
    {
        return;
    }

    /*
     * После включения LOGGER всегда начинает работу
     * с предполетной проверки оборудования.
     */
    logger->req_mode = MODE_TEST;

    /*
     * До получения конфигурации рабочий режим измерений
     * устанавливаем в операторский.
     *
     * Реальное значение будет изменено после получения
     * параметров радиационной разведки с земли.
     */
    logger->work_mode = MODE_OPERATOR;

    /*
     * Предыдущий режим первоначально также устанавливаем
     * в операторский.
     *
     * Значение станет значимым при входе в режим спектра.
     */
    logger->old_work_mode = MODE_OPERATOR;

    /*
     * После запуска основным состоянием является обработка команд.
     */
    logger->state = COMMAND_INFO;

    /*
     * В момент запуска ответ ни от одного прибора не ожидается.
     */
    logger->dev_type = DEV_NONE;

    /*
     * Начальный тип запроса БДБГ — измерение ПАЕД.
     */
    logger->bdbg_request = PAED_REQ;
		
		/*
		* После запуска готовых данных RadioLine ещё нет.
		*/
		logger->radioline_data = 0;
		logger->radioline_length = 0U;		
}


/* ============================================================================
 * Передача завершённого burst RadioLine в стейт-машину.
 *
 * Функция вызывается из main(), когда TIM6 уже определил окончание
 * приёма RadioLine по паузе 100 мс.
 *
 * Здесь данные НЕ разбираются и НЕ копируются.
 * Стейт-машина получает только адрес буфера и количество принятых байтов.
 * ========================================================================== */
void LoggerStateMachine_SetRadioLineData(LoggerStateMachine *logger,
                                         const uint8_t *data,
                                         uint16_t length)
{
    /*
     * Проверяем корректность переданных параметров.
     */
    if ((logger == 0) || (data == 0) || (length == 0U))
    {
        return;
    }

    /*
     * Запоминаем адрес накопительного буфера RadioLine.
     * Сам массив остаётся в main.c.
     */
    logger->radioline_data = data;

    /*
     * Запоминаем фактическое количество принятых байтов.
     */
    logger->radioline_length = length;
}


uint16_t calculateCrc16(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}



/*
 * Дескриптор USART2, созданный в main.c.
 *
 * USART2 используется для обмена с наземной станцией
 * по каналу RadioLine.
 */
extern UART_HandleTypeDef huart2;



/*
 * Формирует и передаёт пакет RadioLine.
 *
 * packet_id      — идентификатор сообщения.
 * payload        — указатель на данные сообщения.
 * payload_length — длина данных в байтах.
 *
 * При отсутствии Payload используется один байт 0x01.
 *
 * Каждый сформированный пакет передаётся repeat_factor раз.
 * Повторное вычисление CRC16 не требуется.
 */
static HAL_StatusTypeDef RadioLine_SendPacket( uint8_t packet_id, const uint8_t *payload, uint16_t payload_length)
{
    /* Буфер для формирования пакета RadioLine. */
    uint8_t packet[256];

    /* Стандартное содержимое отсутствующего Payload. */
    const uint8_t default_payload = 0x01U;

    /* Полный размер сформированного пакета. */
    uint16_t packet_length;

    /* Контрольная сумма пакета. */
    uint16_t crc;

    /* Счётчик переданных копий. */
    uint8_t repeat;

    /* Результат передачи через USART2. */
    HAL_StatusTypeDef status;

		/*
		 * Если данные отсутствуют, используем стандартный
		 * Payload, состоящий из одного байта 0x01.
		 */
    if ((payload == NULL) || (payload_length == 0U))
    {
        payload = &default_payload;
        payload_length = 1U;
    }

    /*
     * Проверяем, что пакет поместится в буфер.
     *
     * Помимо Payload пакет содержит 13 служебных байтов.
     */
    if (payload_length > (sizeof(packet) - 13U))
    {
        return HAL_ERROR;
    }

    /* Маркер начала пакета. */
    packet[0] = 0xAAU;
    packet[1] = 0x55U;
    packet[2] = 0xAAU;
    packet[3] = 0x55U;

    /* Идентификатор сообщения. */
    packet[4] = packet_id;

    /* Длина Payload в формате Little Endian. */
    packet[5] = (uint8_t)(payload_length & 0xFFU);
    packet[6] = (uint8_t)((payload_length >> 8U) & 0xFFU);

    /* Копируем данные сообщения. */
    memcpy(&packet[7], payload, payload_length);

    /*
     * Рассчитываем CRC16 по ID, LENGTH и PAYLOAD.
     * Стартовый и конечный маркеры в расчёт не входят.
     */
    crc = calculateCrc16(
        &packet[4],
        (uint16_t)(3U + payload_length)
    );

    /* Записываем CRC16 в формате Little Endian. */
    packet[7U + payload_length] =
        (uint8_t)(crc & 0xFFU);

    packet[8U + payload_length] =
        (uint8_t)((crc >> 8U) & 0xFFU);

    /* Маркер окончания пакета. */
    packet[9U + payload_length]  = 0x0DU;
    packet[10U + payload_length] = 0x0AU;
    packet[11U + payload_length] = 0x0DU;
    packet[12U + payload_length] = 0x0AU;

    /* Полный размер одного пакета. */
    packet_length = (uint16_t)(13U + payload_length);

    /*
     * Проверяем коэффициент дублирования.
     * Некорректное значение не должно приводить
     * к отсутствию передачи или чрезмерному числу копий.
     */
    if ((repeat_factor < LOGGER_MIN_REPEAT_FACTOR) ||
        (repeat_factor > LOGGER_MAX_REPEAT_FACTOR))
    {
        return HAL_ERROR;
    }

    /*
     * Последовательно передаём одинаковые копии пакета.
     *
     * HAL_UART_Transmit() — блокирующая функция.
     * Следующая передача начинается только после
     * успешного завершения предыдущей.
     *
     * Дополнительные задержки между копиями отсутствуют.
     */
    for (repeat = 0U; repeat < repeat_factor; repeat++)
    {
        status = HAL_UART_Transmit(
            &huart2,
            packet,
            packet_length,
            1000U
        );

        /*
         * Если передача завершилась с ошибкой
         * или истёк тайм-аут, прекращаем отправку
         * оставшихся копий и возвращаем ошибку.
         */
        if (status != HAL_OK)
        {
            return status;
        }
    }

    /* Все копии успешно переданы. */
    return HAL_OK;
}







/* ============================================================================
 * Выполнение одного шага основной стейт-машины LOGGER.
 *
 * Функция вызывается из главного цикла программы и выполняет обработку
 * ТОЛЬКО текущего состояния.
 *
 * Здесь не должно быть бесконечных циклов ожидания.
 * После обработки текущего состояния функция возвращает управление в main().
 * ========================================================================== */
void LoggerStateMachine_Process(LoggerStateMachine *logger)
{
    /*
     * Защита от передачи нулевого указателя.
     */
    if (logger == 0)
    {
        return;
    }
		
		#ifdef DEBUG_MODE
		/*
		 * Запоминаем состояние, для которого
		 * уже отправлено DEBUG-сообщение.
		 */
		static DroneState debug_previous_state = (DroneState)-1;

		/*
		 * При переходе в другое состояние разрешаем
		 * отправку нового DEBUG-сообщения.
		 */
		if (debug_previous_state != logger->state)
		{
				debug_previous_state = logger->state;

				if (logger->state == COMMAND_INFO)
				{
						/*
						 * Сообщение о входе в COMMAND_INFO.
						 */
						(void)RadioLine_SendPacket(70U, NULL, 0U);						
				}
                else if (logger->state == GPS_PROC)
                {
                    (void)RadioLine_SendPacket(81U, NULL, 0U);
                }
                else if (logger->state == GPS_ERROR_PROC)
                {
                    (void)RadioLine_SendPacket(82U, NULL, 0U);
                }
				else if (logger->state == REQ_GPS)
				{
						/*
						 * Сообщение о входе в REQ_GPS.
						 */
						(void)RadioLine_SendPacket(80U, NULL, 0U);
				}
				HAL_Delay(500U);
		}
		#endif

    /*
     * Основной диспетчер состояний LOGGER.
     */
    switch (logger->state)
    {
        /*
         * ============================================================
         * COMMAND_INFO
         *
         * Обработка накопленного массива RadioLine.
         *
         * Наземная станция отправляет дублирующиеся пакеты
         * одной передачей.
         *
         * Ищем первый корректный пакет.
         *
         * Если пакет найден:
         *   - отправляем ответ 01 "Пакет принят";
         *   - прекращаем поиск;
         *   - отбрасываем остальные дубликаты.
         *
         * Если корректный пакет не найден:
         *   - отправляем ответ 02 "Пакет не принят".
         *
         * Земля подтверждений не отправляет.
         * ============================================================
         */
        case COMMAND_INFO:
        {
            /*
             * Ожидаем завершения приёма.
             *
             * TIM6 устанавливает com_info после паузы 100 мс.
             */
            if (com_info == 0U)
            {
                break;
            }

            /*
             * Сбрасываем флаг завершения приёма.
             */
            com_info = 0U;

            /*
             * Минимальный размер пакета RadioLine — 14 байт.
             */
            if (radioline_rx_length < 14U)
            {
                /*
                 * Данных недостаточно для корректного пакета.
                 *
                 * Очищаем накопленный буфер и отправляем
                 * отрицательное подтверждение.
                 */
                radioline_rx_length = 0U;

                (void)RadioLine_SendPacket(0x02U, NULL, 0U);

                break;
            }

            /*
             * Переменные поиска.
             */
            uint8_t start_found = 0U;
            uint8_t packet_found = 0U;

            uint16_t start_pos = 0U;
            uint16_t end_pos = 0U;

            /*
             * Идентификатор команды первого корректного пакета.
             */
            uint8_t received_command = 0U;

            /*
             * Единый цикл поиска по накопленному массиву.
             */
            for (uint16_t i = 0U;
                 (uint32_t)i + 3U < radioline_rx_length;
                 i++)
            {
                /*
                 * ----------------------------------------------------
                 * Поиск маркера начала AA 55 AA 55.
                 * ----------------------------------------------------
                 */
                if ((radioline_rx_buffer[i]      == 0xAAU) &&
                    (radioline_rx_buffer[i + 1U] == 0x55U) &&
                    (radioline_rx_buffer[i + 2U] == 0xAAU) &&
                    (radioline_rx_buffer[i + 3U] == 0x55U))
                {
                    /*
                     * Запоминаем новое начало.
                     *
                     * Если предыдущее начало уже было найдено,
                     * считаем предыдущий фрагмент оборванным.
                     */
                    start_pos = i;
                    start_found = 1U;

                    /*
                     * Пропускаем остальные три байта маркера.
                     */
                    i += 3U;

                    continue;
                }

                /*
                 * Пока начало не найдено, остальные данные
                 * не обрабатываем.
                 */
                if (start_found == 0U)
                {
                    continue;
                }

                /*
                 * ----------------------------------------------------
                 * Поиск маркера окончания 0D 0A 0D 0A.
                 * ----------------------------------------------------
                 */
                if ((radioline_rx_buffer[i]      == 0x0DU) &&
                    (radioline_rx_buffer[i + 1U] == 0x0AU) &&
                    (radioline_rx_buffer[i + 2U] == 0x0DU) &&
                    (radioline_rx_buffer[i + 3U] == 0x0AU))
                {
                    end_pos = i;

                    /*
                     * Минимальное расстояние между началом
                     * и окончанием составляет 10 байт.
                     */
                    if (((uint32_t)end_pos - start_pos) < 10U)
                    {
                        /*
                         * Фрагмент слишком короткий.
                         *
                         * Отбрасываем его и продолжаем
                         * поиск следующего пакета.
                         */
                        start_found = 0U;

                        i += 3U;

                        continue;
                    }

                    /*
                     * ------------------------------------------------
                     * Извлекаем длину Payload.
                     *
                     * Поле длины имеет формат Little Endian.
                     * ------------------------------------------------
                     */
                    uint16_t payload_length =
                        (uint16_t)radioline_rx_buffer[start_pos + 5U] |
                        ((uint16_t)radioline_rx_buffer[start_pos + 6U]
                         << 8U);

                    /*
                     * Вычисляем фактическую длину Payload.
                     */
                    uint16_t actual_payload_length =
                        (uint16_t)((uint32_t)end_pos -
                                   (uint32_t)start_pos - 9U);

                    /*
                     * Проверяем длину.
                     */
                    if ((payload_length == 0U) ||
                        (payload_length != actual_payload_length))
                    {
                        /*
                         * Длина не совпадает.
                         *
                         * Продолжаем поиск следующего пакета.
                         */
                        start_found = 0U;

                        i += 3U;

                        continue;
                    }

                    /*
                     * ------------------------------------------------
                     * Извлекаем принятую CRC16.
                     *
                     * Порядок байтов Little Endian.
                     * ------------------------------------------------
                     */
                    uint16_t received_crc =
                        (uint16_t)radioline_rx_buffer[end_pos - 2U] |
                        ((uint16_t)radioline_rx_buffer[end_pos - 1U]
                         << 8U);

                    /*
                     * Вычисляем CRC16 по команде,
                     * полю длины и Payload.
                     */
                    uint16_t calculated_crc =
                        calculateCrc16(
                            &radioline_rx_buffer[start_pos + 4U],
                            (uint16_t)(3U + payload_length)
                        );

                    /*
                     * Сравниваем контрольные суммы.
                     */
                    if (received_crc != calculated_crc)
                    {
                        /*
                         * CRC16 не совпадает.
                         *
                         * Продолжаем поиск следующего пакета.
                         */
                        start_found = 0U;

                        i += 3U;

                        continue;
                    }

                    /*
                     * ------------------------------------------------
                     * Первый корректный пакет найден.
                     * ------------------------------------------------
                     */
                    received_command =
                        radioline_rx_buffer[start_pos + 4U];

                    packet_found = 1U;

                    /*
                     * Прекращаем поиск.
                     *
                     * Остальные дубликаты игнорируем.
                     */
                    break;
                }
            }

            /*
             * --------------------------------------------------------
             * Обработка результата поиска.
             * --------------------------------------------------------
             */

            /*
             * Освобождаем накопительный буфер.
             *
             * Содержимое массива обнулять не требуется.
             */
            radioline_rx_length = 0U;

            if (packet_found != 0U)
            {
                /*
                 * Найден первый корректный пакет.
                 *
                 * Отправляем положительное подтверждение.
                 *
                 * Результат передачи пока не обрабатываем.
                 * Обработку ошибок USART2 добавим отдельно.
                 */
                (void)RadioLine_SendPacket(0x01U, NULL, 0U);

                /*
								 * Обработка команды в режиме предполётной подготовки.
								 *
								 * Команда 01 — провести предполётное тестирование.
								 *
								 * Подтверждение приёма пакета уже отправлено выше.
								 */
								if ((logger->req_mode == MODE_TEST) && (received_command == 0x01U))
								{
										/*
										 * Переходим к первому этапу предполётного
										 * тестирования — проверке GPS.
										 *
										 * Сам запрос GPS будет реализован
										 * непосредственно в состоянии REQ_GPS.
										 */
										logger->dev_type = DEV_GPS;
										logger->state = REQ_GPS;
								}
            }
            else
            {
                /*
                 * Ни одного корректного пакета не найдено.
                 *
                 * Отправляем отрицательное подтверждение.
                 */
                (void)RadioLine_SendPacket(0x02U, NULL, 0U);
            }

            break;
        }

        /*
         * ============================================================
         * Остальные состояния.
         *
         * Их реализацию добавим последовательно.
         * ============================================================
         */
        /* Запуск GPS: тайм-аут начинается после отладочной задержки. */
        case REQ_GPS:
        {
            logger->dev_type = DEV_GPS;
            if (GPS_Start() == HAL_OK)
            {
                logger->state = GPS_PROC;
            }
            else
            {
                logger->state = GPS_ERROR_PROC;
            }
            break;
        }

        /* Неблокирующая обработка поступивших NMEA-предложений. */
        case GPS_PROC:
        {
            GPS_Fix fix;
            GPS_Result result = GPS_Poll(&fix);
            if (result == GPS_RESULT_FIX)
            {
                uint8_t payload[11];
                /* Формат пакета 19: float lat, float lon, HH MM SS. */
                memcpy(&payload[0], &fix.latitude, sizeof(float));
                memcpy(&payload[4], &fix.longitude, sizeof(float));
                payload[8] = fix.hour;
                payload[9] = fix.minute;
                payload[10] = fix.second;

                GPS_Stop();
                if (GPS_SetRTC(&fix) != HAL_OK)
                {
                    logger->state = GPS_ERROR_PROC;
                    break;
                }
                /* MODE_TEST: геозона ещё не загружена, сообщение «вне зоны». */
                (void)RadioLine_SendPacket(19U, payload, sizeof(payload));
                logger->dev_type = DEV_NONE;
                logger->state = REQ_LIDAR;
            }
            else if ((result == GPS_RESULT_TIMEOUT) ||
                     (result == GPS_RESULT_RX_ERROR))
            {
                logger->state = GPS_ERROR_PROC;
            }
            break;
        }

        /* Ошибка предполётной проверки GPS: сообщение 15, возврат к командам. */
        case GPS_ERROR_PROC:
        {
            GPS_Stop();
            (void)RadioLine_SendPacket(15U, NULL, 0U);
            logger->dev_type = DEV_NONE;
            logger->state = COMMAND_INFO;
            break;
        }

        case REQ_LIDAR:
        case LIDAR_PROC:
        case LIDAR_ERROR_PROC:

        case REQ_BDBG:
        case BDBG_PROC:
        case BDBG_ERROR_PROC:

        case REQ_BATTERY:
        case BATTERY_PROC:
        case BATTERY_ERROR_PROC:

        case WORK_SD:
        {
            break;
        }

        /*
         * Защита от некорректного состояния.
         */
        default:
        {
            logger->state = COMMAND_INFO;

            break;
        }
    }
}