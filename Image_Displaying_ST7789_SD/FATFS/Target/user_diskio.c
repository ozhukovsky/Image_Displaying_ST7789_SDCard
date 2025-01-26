/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "user_diskio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	uint8_t buff[5] = {0x00, 0x00, 0x00, 0x00, 0x95};
	//ARG and CRC

    Stat = STA_NOINIT;

    HAL_Delay(5000);

    uint8_t maxByte = 0xFF;

    for (int i = 0; i < 10; ++i)
    {
  	  SD_transmitSPI(&maxByte, 1);
    }

    USER_ioctl(pdrv, SD_CMD0, buff);

    if (buff[0] == 0x01)
    {
    	buff[0] = 0x00;
    	buff[1] = 0x00;
    	buff[2] = 0x01;
    	buff[3] = 0xAA;
    	buff[4] = 0x87;

    	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);

    	USER_ioctl(pdrv, SD_CMD8, buff);

    	if (buff[0] == 0x05) //SDC v.1 or MMC ver.3
    	{
    		Stat = RES_OK;
    	}
    	else if (buff[3] == 0x01 && buff[4] == 0xAA)//SDC v.2
    	{
    		do
    		{
        		buff[0] = 0x00;
        		buff[1] = 0x00;
        		buff[2] = 0x00;
        		buff[3] = 0x00;
        		buff[4] = 0x01;
        		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);

        		USER_ioctl(pdrv, SD_CMD55, buff);

        		buff[0] = 0x40;
        		buff[1] = 0x00;
        		buff[2] = 0x00;
        		buff[3] = 0x00;
        		buff[4] = 0x01;

        		USER_ioctl(pdrv, SD_CMD41, buff);
    		} while(buff[0] != 0x00);

			buff[0] = 0x00;
			buff[1] = 0x00;
			buff[2] = 0x00;
			buff[3] = 0x00;
			buff[4] = 0x01;

			while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);

			USER_ioctl(pdrv, SD_CMD58, buff);

    		if (buff[1] & 0x40)
    		{
    			Stat = RES_OK;
    		}
    	}
    }
    else if (buff[0] == 0x00)
    {
    	Stat = RES_OK;
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    //Stat = STA_NOINIT;
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */

	uint8_t buffCmd[5] 	  = {sector >> 24, (sector >> 16) & 0xFF, (sector >> 8) & 0xFF, sector & 0xFF, 0x01};
	uint8_t dummyOne[512] = {[ 0 ... 511 ] = 255};
	DRESULT res		   	  = RES_OK;
	uint8_t byteMax	   	  = 0xFF;

	USER_ioctl(pdrv, SD_CMD18, buffCmd);

	if (buffCmd[0] == 0x00)
	{
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

		for (int i = 0; i < count; ++i)
		{
			do
			{
				SD_TransmitReceiveSPI_HAL(&byteMax, buffCmd, 1, HAL_MAX_DELAY); // DATATOKEN
			} while(buffCmd[0] == 0xFF);

			if (buffCmd[0] == DATASTART_TOKEN_1)
			{
				SD_TransmitReceiveSPI_HAL(dummyOne, buff + 512*i, 512, HAL_MAX_DELAY);

				SD_transmitSPI(&byteMax, 2); //CRC
			}
			else
			{
				res = RES_ERROR;
				break;
			}
		}

		if (res != RES_ERROR)
		{
	    	buffCmd[0] = 0x00;
	    	buffCmd[1] = 0x00;
	    	buffCmd[2] = 0x00;
	    	buffCmd[3] = 0x00;
	    	buffCmd[4] = 0x01;

			USER_ioctl(pdrv, SD_CMD12, buffCmd);
		}

		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	}

    return res;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
	uint8_t buffCmd[5] 	   = {sector >> 24, (sector >> 16) & 0xFF, (sector >> 8) & 0xFF, sector & 0xFF, 0x03};
	DRESULT res		   	   = RES_OK;
	uint8_t byteMax	   	   = 0xFF;
	uint16_t anyCrc	   	   = 0x01;
	uint8_t crcPart;
	uint8_t	dataStartToken = DATASTART_TOKEN_2;
	uint8_t stopTranToken  = STOPTRAN_TOKEN_1;

	USER_ioctl(pdrv, SD_CMD25, buffCmd);

	if (buffCmd[0] == 0x00)
	{
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

		SD_transmitSPI(&byteMax, 1);

		for (int i = 0; i < count; ++i)
		{
			SD_transmitSPI(&dataStartToken, 1);

			SD_transmitSPI(buff + 512*i, 512);

			crcPart = anyCrc >> 8;
			SD_transmitSPI(&crcPart, 1);
			crcPart = anyCrc & 0xFF;
			SD_transmitSPI(&crcPart, 1);

			SD_TransmitReceiveSPI_HAL(&byteMax, buffCmd, 1, HAL_MAX_DELAY);

			if ((buffCmd[0] & 0x1F) == 0x05) // DATA ACCEPTED
			{
				SD_transmitSPI(&byteMax, 1); // ONE BYTE NEEDED TO INITIATE WRITE

			    do
			    {
			    	SD_TransmitReceiveSPI_HAL(&byteMax, &buffCmd, 1, HAL_MAX_DELAY);
			    }
			    while(buffCmd[0] != byteMax);
			}
			else
			{
		    	buffCmd[0] = 0x00;
		    	buffCmd[1] = 0x00;
		    	buffCmd[2] = 0x00;
		    	buffCmd[3] = 0x00;
		    	buffCmd[4] = 0x01;

				USER_ioctl(pdrv, SD_CMD12, buffCmd);

				res = RES_ERROR;
				break;
			}
		}

		if (res != RES_ERROR)
		{
			SD_transmitSPI(&stopTranToken, 1);

			SD_transmitSPI(&byteMax, 1);
		}

		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
	}

	return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res 		 = RES_OK;
    uint8_t byteMax 	 = 0xFF;
    uint8_t rcvByte;
    uint8_t dataBytesNum = 0;

	if (cmd == GET_SECTOR_COUNT)
	{
	    uint8_t bufftmp[17] = {0};

	    bufftmp[0] = 0x00;
	    bufftmp[1] = 0x00;
	    bufftmp[2] = 0x00;
	    bufftmp[3] = 0x00;
	    bufftmp[4] = 0x01;

		while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET);

		res = USER_ioctl(pdrv, SD_CMD9, bufftmp);

		*(DWORD*)buff = ((bufftmp[10] | bufftmp[9] << 8 | bufftmp[8] << 16) + 1) * 1024;
		return res;
	}

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

    do
    {
    	SD_TransmitReceiveSPI_HAL(&byteMax, &rcvByte, 1, HAL_MAX_DELAY);
    }
    while(rcvByte != byteMax);

    SD_transmitSPI(&cmd, sizeof(cmd));

    SD_transmitSPI(buff, 5);

	do
	{
		SD_TransmitReceiveSPI_HAL(&byteMax, (uint8_t*)buff, 1, HAL_MAX_DELAY);
	}
	while(*(uint8_t*)buff == byteMax);

	switch (cmd)
	{
		case SD_CMD8:
		case SD_CMD58:
			dataBytesNum = 4;
			break;
		case SD_CMD9:
			dataBytesNum = 16;
		case SD_CMD13:
			dataBytesNum = 1;
			break;
	}

	if (cmd == SD_CMD9)
	{
		SD_TransmitReceiveSPI_HAL(&byteMax, (uint8_t*)buff + 1, 1, HAL_MAX_DELAY);

		if (*((uint8_t*)buff + 1) != DATASTART_TOKEN_1)
		{
			return RES_ERROR;
		}
	}

	for (int i = 0; i < dataBytesNum; ++i)
	{
		SD_TransmitReceiveSPI_HAL(&byteMax, (uint8_t*)buff + i + 1, 1, HAL_MAX_DELAY);
	}

	if (cmd == SD_CMD9)
	{
		SD_transmitSPI(&byteMax, 2); //skip CRC
	}

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

