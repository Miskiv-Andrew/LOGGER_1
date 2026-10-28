#ifndef LIDAR_H
#define LIDAR_H

#include "main.h"
#include <stdint.h>

/* Число байтов потока, принимаемых за один опрос. */
#define LIDAR_RX_SIZE             45U
#define LIDAR_FRAME_SIZE           9U
#define LIDAR_MIN_DISTANCE_CM    100U
#define LIDAR_MAX_DISTANCE_CM   5000U
#define LIDAR_MIN_STRENGTH       100U

/*
 * Рабочие данные LIDAR.
 * В пакет ID22 сериализуются ТОЛЬКО distance_cm и strength
 * (по два байта каждый). Поле valid в радиопакет не входит.
 */
typedef struct
{
    uint16_t distance_cm;
    uint16_t strength;
    uint8_t valid;
} LIDAR_Data;

/* Признак завершения приёма 45 байтов; устанавливается из прерывания. */
extern volatile uint8_t lidar_info;

/* Инициализация программного состояния, без запуска периферии. */
void LIDAR_Init(void);

/* Запуск нового приёма 45 байтов через LPUART1 + DMA. TIM3 не затрагивается. */
HAL_StatusTypeDef LIDAR_StartReceive(void);

/* Остановка приёма, без деинициализации LPUART1. TIM3 не затрагивается. */
void LIDAR_StopReceive(void);

/* Вызывать из HAL_UART_RxCpltCallback() только для LPUART1. */
void LIDAR_RxCpltCallback(void);

/* Вызывать из HAL_UART_ErrorCallback() только для LPUART1. */
void LIDAR_NotifyUartError(void);

/*
 * Поиск первого валидного кадра в 45 принятых байтах.
 * Возвращает 1 при успехе, 0 при отсутствии валидного кадра.
 * Вызывать после lidar_info == 1; результат копируется в out.
 */
uint8_t LIDAR_Process(LIDAR_Data *out);

/* Признак ошибки UART/DMA в текущем опросе. */
uint8_t LIDAR_HasUartError(void);

#endif /* LIDAR_H */
