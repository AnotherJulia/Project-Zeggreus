/*
 * flash.h. For use with the ws25q128
 * https://datasheet.lcsc.com/szlcsc/1905281008_Winbond-Elec-W25Q128JVSIQ_C113767.pdf
 *  Created on: 18 Mar 2021
 *      Author: elvin
 */

#ifndef SRC_DRIVERS_FLASH_H_
#define SRC_DRIVERS_FLASH_H_


#include "stm32f4xx_hal.h"


typedef struct __attribute__((packed)) {
    uint8_t packet_state_bits; // <[]>
    uint8_t pin_states_servo;
    uint32_t systick; // equivalent to 24 bit unsigned integer
    float vbat;
    float orientation_quat[4]; // [w,x,y,z]
    int16_t acc[3];
    int16_t gyro[3];
    float baro;
    float temp; // temperature in celsius
    float altitude;
    uint8_t vertical_velocity;
    uint16_t debug_ranging;
} blackbox_packet;

typedef struct __attribute__((packed)) {
    // add up to 256 bytes (one page)
    blackbox_packet packets[4]; // 64 bytes each
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
