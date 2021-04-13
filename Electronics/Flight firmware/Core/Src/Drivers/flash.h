/*
 * flash.h. For use with the ws25q128
 * https://datasheet.lcsc.com/szlcsc/1905281008_Winbond-Elec-W25Q128JVSIQ_C113767.pdf
 *  Created on: 18 Mar 2021
 *      Author: elvin
 */

#ifndef SRC_DRIVERS_FLASH_H_
#define SRC_DRIVERS_FLASH_H_


#include "stm32f4xx_hal.h"

typedef struct {
    // add up to 256 bytes (one page)
    uint8_t padding[256];
} flashblock;


typedef struct {
    SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef *csPinBank;
    uint16_t csPin;

    uint8_t txBuf[7];
    uint8_t rxBuf[8];

    flashblock dataBuffer;
} flash;


#endif /* SRC_DRIVERS_FLASH_H_ */
