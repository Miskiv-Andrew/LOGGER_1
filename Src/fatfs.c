/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */

#include "rtc_service.h"

extern RTC_HandleTypeDef hrtc;

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}



/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
    RTC_TimeTypeDef rtcTime = {0};
    RTC_DateTypeDef rtcDate = {0};

    /*
     * Получаем текущее время из RTC.
     *
     * RTC_FORMAT_BIN означает, что часы, минуты
     * и секунды возвращаются в двоичном формате.
     */
    if (HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        return 0U;
    }

    /*
     * После HAL_RTC_GetTime() обязательно вызываем
     * HAL_RTC_GetDate(), чтобы разблокировать
     * теневые регистры RTC.
     */
    if (HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        return 0U;
    }

    /*
     * Проверяем диапазоны даты и времени.
     *
     * FatFs поддерживает годы начиная с 1980.
     * Здесь предполагается стандартное представление
     * STM32 RTC: Year = 0..99 для 2000..2099.
     */
    if (rtcDate.Year > 99U ||
        rtcDate.Month < 1U || rtcDate.Month > 12U ||
        rtcDate.Date < 1U || rtcDate.Date > 31U ||
        rtcTime.Hours > 23U ||
        rtcTime.Minutes > 59U ||
        rtcTime.Seconds > 59U)
    {
        return 0U;
    }

    /*
     * Формируем 32-битную временную метку FAT:
     *
     * Биты 31..25: год относительно 1980
     * Биты 24..21: месяц
     * Биты 20..16: день
     * Биты 15..11: часы
     * Биты 10..5 : минуты
     * Биты 4..0  : секунды / 2
     */
    return ((DWORD)(rtcDate.Year + 20U) << 25) |
           ((DWORD)rtcDate.Month << 21) |
           ((DWORD)rtcDate.Date << 16) |
           ((DWORD)rtcTime.Hours << 11) |
           ((DWORD)rtcTime.Minutes << 5) |
           ((DWORD)(rtcTime.Seconds / 2U));
  /* USER CODE END get_fattime */
}
/* USER CODE BEGIN Application */

/* USER CODE END Application */
