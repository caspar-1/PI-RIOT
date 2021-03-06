/*
 * Copyright (C) 2013 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_native Native Board
 * @ingroup     boards
 * @brief       Board emulation for the native port
 *
 * The native board uses call level hardware simulation
 *
 * @{
 *
 * @file
 * @brief       Basic definitions for the native board
 *
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   LED handlers
 * @{
 */
void _native_LED_GREEN_OFF(void);
void _native_LED_GREEN_ON(void);
void _native_LED_GREEN_TOGGLE(void);
void _native_LED_RED_OFF(void);
void _native_LED_RED_ON(void);
void _native_LED_RED_TOGGLE(void);

#define LED0_ON             (_native_LED_RED_ON())
#define LED0_OFF            (_native_LED_RED_OFF())
#define LED0_TOGGLE         (_native_LED_RED_TOGGLE())
#define LED1_ON             (_native_LED_GREEN_ON())
#define LED1_OFF            (_native_LED_GREEN_OFF())
#define LED1_TOGGLE         (_native_LED_GREEN_TOGGLE())




#define PERIPH_SPI_NEEDS_TRANSFER_BYTE
#define PERIPH_SPI_NEEDS_TRANSFER_REG
#define PERIPH_SPI_NEEDS_TRANSFER_REGS

/** @} */

//set up pins used by MRF24J40 interface
#define MRF24J40_PARAM_RESET        (GPIO_PIN(0, 13))



#define SPI_NUMOF           (1)


#ifdef __cplusplus
}
#endif

/** @} */
#endif /* BOARD_H */
