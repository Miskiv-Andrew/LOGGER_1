/**
 ******************************************************************************
  * @file    user_diskio_spi.c
  * @brief   This file contains the implementation of the user_diskio_spi FatFs  
  *          driver.
  ******************************************************************************
  * Portions copyright (C) 2014, ChaN, all rights reserved.
  * Portions copyright (C) 2017, kiwih, all rights reserved.
  *
  * This software is a free software and there is NO WARRANTY.
  * No restriction on use. You can use, modify and redistribute it for
  * personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
  * Redistributions of source code must retain the above copyright notice.
  *
  ******************************************************************************
  */

//This code was ported by kiwih from a copywrited (C) library written by ChaN
//available at http://elm-chan.org/fsw/ff/ffsample.zip 
//(text at http://elm-chan.org/fsw/ff/00index_e.html)

//This file provides the FatFs driver functions and SPI code required to manage
//an SPI-connected MMC or compatible SD card with FAT

//It is designed to be wrapped by a cubemx generated user_diskio.c file.

#include "main.h" /* Provide the low-level HAL functions */
#include "user_diskio_spi.h"

//If you want to use a different SPI port change hspi1 here and elsewhere in this file
extern SPI_HandleTypeDef hspi1; 

/* Function prototypes */

//(Note that the _256 is used as a mask to clear the prescalar bits as it provides binary 111 in the correct position)
/*
#define FCLK_SLOW() do { \
    __HAL_SPI_DISABLE(&hspi1); \
    MODIFY_REG(hspi1.Instance->CR1, SPI_CR1_BR_Msk, SPI_BAUDRATEPRESCALER_256); \
    __HAL_SPI_ENABLE(&hspi1); \
} while(0)

#define FCLK_FAST() do { \
    __HAL_SPI_DISABLE(&hspi1); \
    MODIFY_REG(hspi1.Instance->CR1, SPI_CR1_BR_Msk, SPI_BAUDRATEPRESCALER_8); \
    __HAL_SPI_ENABLE(&hspi1); \
} while(0)
*/
//#define FCLK_SLOW() { MODIFY_REG(hspi1.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_128); }	/* Set SCLK = slow, approx 280 KBits/s*/
//#define FCLK_FAST() { MODIFY_REG(hspi1.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_8); }	/* Set SCLK = fast, approx 4.5 MBits/s */

/*
 * ============================================================
 * Управление частотой SPI1 при работе с SD-картой.
 *
 * При инициализации SD-карты используется низкая частота.
 * После успешной инициализации — рабочая частота.
 *
 * Перед изменением делителя SPI отключается.
 * После изменения — включается обратно.
 * ============================================================
 */

#define FCLK_SLOW()                                      \
    do                                                  \
    {                                                   \
        __HAL_SPI_DISABLE(&hspi1);                      \
        MODIFY_REG(hspi1.Instance->CR1,                  \
                   SPI_CR1_BR_Msk,                       \
                   SPI_BAUDRATEPRESCALER_128);           \
        __HAL_SPI_ENABLE(&hspi1);                       \
    } while (0)

#define FCLK_FAST()                                      \
    do                                                  \
    {                                                   \
        __HAL_SPI_DISABLE(&hspi1);                      \
        MODIFY_REG(hspi1.Instance->CR1,                  \
                   SPI_CR1_BR_Msk,                       \
                   SPI_BAUDRATEPRESCALER_8);             \
        __HAL_SPI_ENABLE(&hspi1);                       \
    } while (0)





/*
#define CS_HIGH()	{HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);}
#define CS_LOW()	{HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);}
*/
#define CS_HIGH()	{HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);}
#define CS_LOW()	{HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);}
/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* MMC/SD command */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

static volatile
DSTATUS Stat = STA_NOINIT;	/* Physical drive status */


static
BYTE CardType;			/* Card type flags */

uint32_t spiTimerTickStart;
uint32_t spiTimerTickDelay;

void SPI_Timer_On(uint32_t waitTicks) {
    spiTimerTickStart = HAL_GetTick();
    spiTimerTickDelay = waitTicks;
}

uint8_t SPI_Timer_Status() {
    return ((HAL_GetTick() - spiTimerTickStart) < spiTimerTickDelay);
}

/*-----------------------------------------------------------------------*/
/* SPI controls (Platform dependent)                                     */
/*-----------------------------------------------------------------------*/

/* Exchange a byte */
/*
static BYTE xchg_spi (	BYTE dat)
{
	BYTE rxDat;
    HAL_SPI_TransmitReceive(&hspi1, &dat, &rxDat, 1, 50);
    return rxDat;
}
*/

/*
 * Обмен одним байтом с SD-картой по SPI1.
 *
 * Возвращает:
 *   - принятый байт при успешном обмене;
 *   - 0xFF при ошибке или тайм-ауте HAL SPI.
 *
 * Важно:
 * 0xFF также является обычным ответом SD-карты
 * в состоянии ожидания. Поэтому для диагностики
 * дополнительно используется флаг ошибки SPI.
 */
static volatile uint8_t sd_spi_error = 0U;

static BYTE xchg_spi(BYTE dat)
{
    BYTE rxDat = 0xFFU;

    /*
     * Если ранее обнаружена ошибка SPI, не выполняем
     * дальнейшие обращения к периферии до сброса
     * флага при новой инициализации карты.
     */
    if (sd_spi_error != 0U)
    {
        return 0xFFU;
    }

    /*
     * Выполняем обмен одним байтом.
     * Максимальное время ожидания HAL — 50 мс.
     */
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
        &hspi1,
        &dat,
        &rxDat,
        1U,
        50U
    );

    /*
     * Любая ошибка HAL (ERROR, BUSY, TIMEOUT)
     * переводит SPI-обмен в состояние ошибки.
     */
    if (result != HAL_OK)
    {
        sd_spi_error = 1U;
        return 0xFFU;
    }

    return rxDat;
}

/* Receive multiple byte */
static
void rcvr_spi_multi (
	BYTE *buff,		/* Pointer to data buffer */
	UINT btr		/* Number of bytes to receive (even number) */
)
{
	for(UINT i=0; i<btr; i++) {
		*(buff+i) = xchg_spi(0xFF);
	}
}


#if _USE_WRITE
/* Send multiple byte */
static
void xmit_spi_multi (
	const BYTE *buff,	/* Pointer to the data */
	UINT btx			/* Number of bytes to send (even number) */
)
{
	for(UINT i=0; i<btx; i++) {
		xchg_spi(*(buff+i));
	}
}
#endif


/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static
int wait_ready (	/* 1:Ready, 0:Timeout */
	UINT wt			/* Timeout [ms] */
)
{
	BYTE d;
	//wait_ready needs its own timer, unfortunately, so it can't use the
	//spi_timer functions
	uint32_t waitSpiTimerTickStart;
	uint32_t waitSpiTimerTickDelay;

	waitSpiTimerTickStart = HAL_GetTick();
	waitSpiTimerTickDelay = (uint32_t)wt;
	do {
		d = xchg_spi(0xFF);
		/* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
	} while (d != 0xFF && ((HAL_GetTick() - waitSpiTimerTickStart) < waitSpiTimerTickDelay));	/* Wait for card goes ready or timeout */

	return (d == 0xFF) ? 1 : 0;
}



/*-----------------------------------------------------------------------*/
/* Despiselect card and release SPI                                         */
/*-----------------------------------------------------------------------*/

static
void despiselect (void)
{
	CS_HIGH();		/* Set CS# high */
	xchg_spi(0xFF);	/* Dummy clock (force DO hi-z for multiple slave SPI) */

}



/*-----------------------------------------------------------------------*/
/* Select card and wait for ready                                        */
/*-----------------------------------------------------------------------*/

static
int spiselect (void)	/* 1:OK, 0:Timeout */
{
	CS_LOW();		/* Set CS# low */
	xchg_spi(0xFF);	/* Dummy clock (force DO enabled) */
	if (wait_ready(500)) return 1;	/* Wait for card ready */

	despiselect();
	return 0;	/* Timeout */
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from the MMC                                    */
/*-----------------------------------------------------------------------*/

static
int rcvr_datablock (	/* 1:OK, 0:Error */
	BYTE *buff,			/* Data buffer */
	UINT btr			/* Data block length (byte) */
)
{
	BYTE token;


	SPI_Timer_On(200);
	do {							/* Wait for DataStart token in timeout of 200ms */
		token = xchg_spi(0xFF);
		/* This loop will take a time. Insert rot_rdq() here for multitask envilonment. */
	} while ((token == 0xFF) && SPI_Timer_Status());
	if(token != 0xFE) return 0;		/* Function fails if invalid DataStart token or timeout */

	rcvr_spi_multi(buff, btr);		/* Store trailing data to the buffer */
	xchg_spi(0xFF); xchg_spi(0xFF);			/* Discard CRC */

	return 1;						/* Function succeeded */
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to the MMC                                         */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
static
int xmit_datablock (	/* 1:OK, 0:Failed */
	const BYTE *buff,	/* Ponter to 512 byte data to be sent */
	BYTE token			/* Token */
)
{
	BYTE resp;


	if (!wait_ready(500)) return 0;		/* Wait for card ready */

	xchg_spi(token);					/* Send token */
	if (token != 0xFD) {				/* Send data if token is other than StopTran */
		xmit_spi_multi(buff, 512);		/* Data */
		xchg_spi(0xFF); xchg_spi(0xFF);	/* Dummy CRC */

		resp = xchg_spi(0xFF);				/* Receive data resp */
		if ((resp & 0x1F) != 0x05) return 0;	/* Function fails if the data packet was not accepted */
	}
	return 1;
}
#endif


/*-----------------------------------------------------------------------*/
/* Send a command packet to the MMC                                      */
/*-----------------------------------------------------------------------*/

static
BYTE send_cmd (		/* Return value: R1 resp (bit7==1:Failed to send) */
	BYTE cmd,		/* Command index */
	DWORD arg		/* Argument */
)
{
	BYTE n, res;


	if (cmd & 0x80) {	/* Send a CMD55 prior to ACMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready except to stop multiple block read */
	if (cmd != CMD12) {
		despiselect();
		if (!spiselect()) return 0xFF;
	}

	/* Send command packet */
	xchg_spi(0x40 | cmd);				/* Start + command index */
	xchg_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xchg_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xchg_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xchg_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xchg_spi(n);

	/* Receive command resp */
	if (cmd == CMD12) xchg_spi(0xFF);	/* Diacard following one byte when CMD12 */
	n = 10;								/* Wait for response (10 bytes max) */
	do {
		res = xchg_spi(0xFF);
	} while ((res & 0x80) && --n);

	return res;							/* Return received response */
}


/*--------------------------------------------------------------------------

   Public FatFs Functions (wrapped in user_diskio.c)

---------------------------------------------------------------------------*/

//The following functions are defined as inline because they aren't the functions that
//are passed to FatFs - they are wrapped by autogenerated (non-inline) cubemx template
//code. 
//If you do not wish to use cubemx, remove the "inline" from these functions here
//and in the associated .h


/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

inline DSTATUS USER_SPI_initialize ( BYTE drv		/* Physical drive number (0) */ )
{
	BYTE n, cmd, ty, ocr[4];

	if (drv != 0) return STA_NOINIT;		/* Supports only drive 0 */
	//assume SPI already init init_spi();	/* Initialize SPI */
	
	/*
	* Разрешаем новую попытку обмена с SD-картой
	* при повторной инициализации.
	*/
	sd_spi_error = 0U;

	if (Stat & STA_NODISK) return Stat;	/* Is card existing in the soket? */

	FCLK_SLOW();
	for (n = 10; n; n--) xchg_spi(0xFF);	/* Send 80 dummy clocks */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Put the card SPI/Idle state */
		SPI_Timer_On(1000);					/* Initialization timeout = 1 sec */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
			for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);	/* Get 32 bit return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* Is the card supports vcc of 2.7-3.6V? */
				while (SPI_Timer_Status() && send_cmd(ACMD41, 1UL << 30)) ;	/* Wait for end of initialization with ACMD41(HCS) */
				if (SPI_Timer_Status() && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* Card id SDv2 */
				}
			}
		} else {	/* Not SDv2 card */
			if (send_cmd(ACMD41, 0) <= 1) 	{	/* SDv1 or MMC? */
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
			}
			while (SPI_Timer_Status() && send_cmd(cmd, 0)) ;		/* Wait for end of initialization */
			if (!SPI_Timer_Status() || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
				ty = 0;
		}
	} 
	CardType = ty;	/* Card type */
	despiselect();

	if (ty) {			/* OK */
		FCLK_FAST();			/* Set fast clock */
		Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
	} else {			/* Failed */
		Stat = STA_NOINIT;
	}

	return Stat;
}



/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

inline DSTATUS USER_SPI_status (
	BYTE drv		/* Physical drive number (0) */
)
{
	if (drv) return STA_NOINIT;		/* Supports only drive 0 */

	return Stat;	/* Return disk status */
}



/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

inline DRESULT USER_SPI_read(
    BYTE drv,
    BYTE *buff,
    DWORD sector,
    UINT count
)
{
    /*
     * Проверяем номер диска, указатель буфера
     * и количество запрашиваемых секторов.
     */
    if (drv != 0U || buff == NULL || count == 0U)
    {
        return RES_PARERR;
    }

    /*
     * Проверяем готовность диска.
     */
    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    /*
     * Если ранее произошла ошибка SPI,
     * чтение выполнять запрещено.
     */
    if (sd_spi_error != 0U)
    {
        return RES_ERROR;
    }

    /*
     * Для карт без блочной адресации
     * преобразуем номер сектора в байтовый адрес.
     */
    if (!(CardType & CT_BLOCK))
    {
        sector *= 512U;
    }

    /*
     * Чтение одного сектора.
     */
    if (count == 1U)
    {
        if ((send_cmd(CMD17, sector) == 0U) &&
            rcvr_datablock(buff, 512U))
        {
            count = 0U;
        }
    }
    else
    {
        /*
         * Чтение нескольких последовательных секторов.
         */
        if (send_cmd(CMD18, sector) == 0U)
        {
            do
            {
                if (!rcvr_datablock(buff, 512U))
                {
                    break;
                }

                buff += 512U;

            } while (--count);

            /*
             * Завершаем многоблочное чтение.
             */
            (void)send_cmd(CMD12, 0U);
        }
    }

    /*
     * Освобождаем SD-карту.
     */
    despiselect();

    /*
     * Ошибка SPI имеет приоритет над результатом
     * выполнения команды SD-карты.
     */
    if (sd_spi_error != 0U)
    {
        return RES_ERROR;
    }

    /*
     * count == 0 означает, что все запрошенные
     * секторы успешно прочитаны.
     */
    return (count == 0U) ? RES_OK : RES_ERROR;
}


/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE

inline DRESULT USER_SPI_write(
    BYTE drv,
    const BYTE *buff,
    DWORD sector,
    UINT count
)
{
    /*
     * Проверяем параметры операции.
     */
    if (drv != 0U || buff == NULL || count == 0U)
    {
        return RES_PARERR;
    }

    /*
     * Проверяем готовность диска.
     */
    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    /*
     * Проверяем защиту от записи.
     */
    if (Stat & STA_PROTECT)
    {
        return RES_WRPRT;
    }

    /*
     * После ошибки SPI запрещаем дальнейшую запись.
     */
    if (sd_spi_error != 0U)
    {
        return RES_ERROR;
    }

    /*
     * Преобразуем адрес для карт с байтовой адресацией.
     */
    if (!(CardType & CT_BLOCK))
    {
        sector *= 512U;
    }

    /*
     * Запись одного сектора.
     */
    if (count == 1U)
    {
        if ((send_cmd(CMD24, sector) == 0U) &&
            xmit_datablock(buff, 0xFEU))
        {
            count = 0U;
        }
    }
    else
    {
        /*
         * Запись нескольких последовательных секторов.
         */
        if (CardType & CT_SDC)
        {
            (void)send_cmd(ACMD23, count);
        }

        if (send_cmd(CMD25, sector) == 0U)
        {
            do
            {
                if (!xmit_datablock(buff, 0xFCU))
                {
                    break;
                }

                buff += 512U;

            } while (--count);

            /*
             * Передаём токен завершения многоблочной записи.
             */
            if (!xmit_datablock(NULL, 0xFDU))
            {
                count = 1U;
            }
        }
    }

    /*
     * Ожидаем завершения внутреннего программирования
     * данных картой перед освобождением CS.
     *
     * Тайм-аут ограничен 500 мс.
     */
    if (count == 0U && sd_spi_error == 0U)
    {
        if (!wait_ready(500U))
        {
            count = 1U;
        }
    }

    /*
     * Освобождаем карту.
     */
    despiselect();

    /*
     * При аппаратной ошибке SPI возвращаем RES_ERROR.
     */
    if (sd_spi_error != 0U)
    {
        return RES_ERROR;
    }

    /*
     * Успех только при завершении записи всех секторов.
     */
    return (count == 0U) ? RES_OK : RES_ERROR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL

inline DRESULT USER_SPI_ioctl(
    BYTE drv,
    BYTE cmd,
    void *buff
)
{
    DRESULT res;
    BYTE n;
    BYTE csd[16];
    DWORD *dp;
    DWORD st;
    DWORD ed;
    DWORD csize;

    /*
     * Проверяем номер диска.
     */
    if (drv != 0U)
    {
        return RES_PARERR;
    }

    /*
     * Проверяем готовность карты.
     */
    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    /*
     * Если SPI неисправен, дальнейшие операции
     * с картой запрещены.
     */
    if (sd_spi_error != 0U)
    {
        return RES_ERROR;
    }

    /*
     * Для команд, возвращающих данные,
     * требуется действительный указатель.
     */
    if ((cmd == GET_SECTOR_COUNT ||
         cmd == GET_BLOCK_SIZE ||
         cmd == CTRL_TRIM) &&
        buff == NULL)
    {
        return RES_PARERR;
    }

    res = RES_ERROR;

    switch (cmd)
    {
        case CTRL_SYNC:
        {
            /*
             * Ожидаем завершения операций записи.
             */
            if (spiselect())
            {
                res = RES_OK;
            }

            break;
        }

        case GET_SECTOR_COUNT:
        {
            /*
             * Получаем CSD и определяем ёмкость карты.
             */
            if ((send_cmd(CMD9, 0U) == 0U) &&
                rcvr_datablock(csd, 16U))
            {
                if ((csd[0] >> 6) == 1U)
                {
                    /*
                     * SD версии 2.
                     */
                    csize =
                        (DWORD)csd[9] +
                        ((DWORD)csd[8] << 8) +
                        ((DWORD)(csd[7] & 63U) << 16) +
                        1U;

                    *(DWORD *)buff = csize << 10;
                }
                else
                {
                    /*
                     * SD версии 1 или MMC.
                     */
                    n =
                        (csd[5] & 15U) +
                        ((csd[10] & 128U) >> 7) +
                        ((csd[9] & 3U) << 1) +
                        2U;

                    csize =
                        (csd[8] >> 6) +
                        ((DWORD)csd[7] << 2) +
                        ((DWORD)(csd[6] & 3U) << 10) +
                        1U;

                    *(DWORD *)buff = csize << (n - 9U);
                }

                res = RES_OK;
            }

            break;
        }

        case GET_BLOCK_SIZE:
        {
            /*
             * Получаем размер блока стирания.
             */
            if (CardType & CT_SD2)
            {
                if (send_cmd(ACMD13, 0U) == 0U)
                {
                    (void)xchg_spi(0xFFU);

                    if (rcvr_datablock(csd, 16U))
                    {
                        for (n = 64U - 16U; n; n--)
                        {
                            (void)xchg_spi(0xFFU);
                        }

                        *(DWORD *)buff =
                            16UL << (csd[10] >> 4);

                        res = RES_OK;
                    }
                }
            }
            else
            {
                if ((send_cmd(CMD9, 0U) == 0U) &&
                    rcvr_datablock(csd, 16U))
                {
                    if (CardType & CT_SD1)
                    {
                        *(DWORD *)buff =
                            (((csd[10] & 63U) << 1) +
                             ((csd[11] & 128U) >> 7) +
                             1U)
                            << ((csd[13] >> 6) - 1U);
                    }
                    else
                    {
                        *(DWORD *)buff =
                            ((DWORD)((csd[10] & 124U) >> 2) + 1U) *
                            (((csd[11] & 3U) << 3) +
                             ((csd[11] & 224U) >> 5) +
                             1U);
                    }

                    res = RES_OK;
                }
            }

            break;
        }

        case CTRL_TRIM:
        {
            /*
             * TRIM пока не выполняем.
             *
             * Эта операция не требуется для записи CSV
             * и не должна влиять на основные функции.
             */
            res = RES_PARERR;
            break;
        }

        default:
        {
            res = RES_PARERR;
            break;
        }
    }

    /*
     * Освобождаем карту после выполнения команды.
     */
    despiselect();

    /*
     * Если во время выполнения операции произошла
     * ошибка SPI, результат операции недействителен.
     */
    if (sd_spi_error != 0U)
    {
        return RES_ERROR;
    }

    return res;
}

#endif