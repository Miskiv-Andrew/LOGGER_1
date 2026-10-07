#ifndef LOGGER_GPS_H
#define LOGGER_GPS_H

#include "main.h"
#include <stdint.h>

/* Результат одного неблокирующего шага обработки GPS. */
typedef enum
{
    GPS_RESULT_WAIT = 0,
    GPS_RESULT_FIX,
    GPS_RESULT_TIMEOUT,
    GPS_RESULT_RX_ERROR
} GPS_Result;

/* Только проверенная NMEA RMC информация. */
typedef struct
{
    float latitude;
    float longitude;
    uint8_t hour, minute, second;
    uint8_t day, month, year;
} GPS_Fix;

void GPS_Init(void);
HAL_StatusTypeDef GPS_Start(void);
void GPS_Stop(void);
void GPS_RxEventCallback(uint16_t size);
void GPS_NotifyUartError(void);
GPS_Result GPS_Poll(GPS_Fix *fix);
HAL_StatusTypeDef GPS_SetRTC(const GPS_Fix *fix);

#endif /* LOGGER_GPS_H */
