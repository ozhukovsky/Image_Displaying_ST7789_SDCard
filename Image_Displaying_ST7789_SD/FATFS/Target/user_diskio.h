/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the user_diskio driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_DISKIO_H
#define __USER_DISKIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* USER CODE BEGIN 0 */
#define SD_CMD0  0x40
#define SD_CMD1  0x41
#define SD_CMD8  0x48
#define SD_CMD9  0x49
#define SD_CMD12 0x4C
#define SD_CMD13 0x4D
#define SD_CMD16 0x50
#define SD_CMD17 0x51
#define SD_CMD18 0x52
#define SD_CMD25 0x59
#define SD_CMD41 0x69
#define SD_CMD42 0x6A
#define SD_CMD55 0x77
#define SD_CMD58 0x7A

#define DATASTART_TOKEN_1 0xFE
#define DATASTART_TOKEN_2 0xFC
#define STOPTRAN_TOKEN_1  0xFD
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
 void SD_transmitSPI(uint8_t *data, uint16_t dataSize);
 void SD_TransmitReceiveSPI_HAL(uint8_t *pTxData, uint8_t *pRxData, uint16_t size, uint32_t timeout);
 void SD_ReceiveSPI_HAL(uint8_t *pRxData, uint16_t size, uint32_t timeout);
/* Exported functions ------------------------------------------------------- */
extern Diskio_drvTypeDef  USER_Driver;

/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif

#endif /* __USER_DISKIO_H */
