#include "main.h"
#include "fonts.h"
#include "st7789.h"

static void ST7789_SendData(uint8_t* buff, uint16_t buffSize)
{
	HAL_GPIO_WritePin(ST7789_DC_GPIO_Port, ST7789_DC_Pin, GPIO_PIN_SET);

	ST7789_transmitSPI(buff, buffSize);
}

void ST7789_SetAddressWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
	uint8_t xLimit[4] = {xs >> 8, xs & 0x00FF, xe >> 8, xe & 0x00FF};
	uint8_t yLimit[4] = {ys >> 8, ys & 0x00FF, ye >> 8, ye & 0x00FF};

	ST7789_SendCmd(ST7789_CASET);

	ST7789_SendData((uint8_t*)xLimit, sizeof(xLimit));

	ST7789_SendCmd(ST7789_RASET);
	ST7789_SendData((uint8_t*)yLimit, sizeof(yLimit));
}

void ST7789_SendCmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(ST7789_DC_GPIO_Port, ST7789_DC_Pin, ST7789_DC_CMD);

	ST7789_transmitSPI(&cmd, 1);
}

void ST7789_SendDataPortion(uint8_t data)
{
	ST7789_SendData(&data, 1);
}

void ST7789_Init()
{
	HAL_Delay(5000);

	HAL_GPIO_WritePin(ST7789_DC_GPIO_Port, ST7789_RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ST7789_DC_GPIO_Port, ST7789_RES_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	ST7789_SendCmd(ST7789_COLMOD);
	ST7789_SendDataPortion(ST7789_COLMOD_PARAM_16BIT);

	ST7789_SendCmd(ST7789_INVON);
	ST7789_SendCmd(ST7789_SLPOUT);
	HAL_Delay(5);
	ST7789_SendCmd(ST7789_DISPON);
}

