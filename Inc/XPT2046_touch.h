/*
 * XPT2046_touch.h
 *
 *  Created on: 20 sep. 2019 ð.
 *      Author: Andriy Honcharenko
 */

#ifndef XPT2046_TOUCH_H_
#define XPT2046_TOUCH_H_

#include "main.h"
#include <stdbool.h>

/*** Redefine if necessary ***/

// Warning! Use SPI bus with < 2.5 Mbit speed, better ~650 Kbit to be save.
#define XPT2046_SPI_PORT hspi1
extern SPI_HandleTypeDef XPT2046_SPI_PORT;

#define XPT2046_IRQ_Pin       T_IRQ_Pin
#define XPT2046_IRQ_GPIO_Port T_IRQ_GPIO_Port
#define XPT2046_CS_Pin        T_CS_Pin
#define XPT2046_CS_GPIO_Port  T_CS_GPIO_Port

// change depending on screen orientation
#define XPT2046_SCALE_X 480
#define XPT2046_SCALE_Y 320

// to calibrate uncomment UART_Printf line in ili9341_touch.c
#define XPT2046_MIN_RAW_X 3900
#define XPT2046_MAX_RAW_X 28090
#define XPT2046_MIN_RAW_Y 4500
#define XPT2046_MAX_RAW_Y 27500

// call before initializing any SPI devices
static void XPT2046_Init();
bool XPT2046_TouchPressed();
bool XPT2046_TouchGetCoordinates(uint16_t* x, uint16_t* y);
//compatability with ADA_Fruit libs
uint8_t   TS_IO_DetectToch(void);
uint16_t  TS_IO_GetY(void);
uint16_t  TS_IO_GetX(void);

#endif /* XPT2046_TOUCH_H_ */
