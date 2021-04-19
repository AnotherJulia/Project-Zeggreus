/*
 * LSM6DSO32.h
 *
 *  Created on: 18 Mar 2021
 *      Author: elvin
 */

#ifndef INC_DRIVERS_LSM6DSO32_H_
#define INC_DRIVERS_LSM6DSO32_H_

#include "stm32f4xx_hal.h"
#include "constants.h"

//lsm6dso32
#define LSM_WHO_AM_I    0x0F
#define LSM_CTRL3_C     0x12
#define LSM_CTRL1_XL        0x10
#define LSM_CTRL2_G         0x11
#define LSM_CTRL4_C         0x13
#define LSM_CTRL6_C         0x15
#define LSM_CTRL7_G         0x16
#define LSM_INT1_CTRL       0x0d
#define LSM_COUNTER_BDR_REG1 0x0b
#define LSM_OUTX_L_G        0x22
#define LSM_OUTX_L_A        0x28

typedef struct {
    SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef *csPinBank;
    uint16_t csPin;

    uint8_t txBuf[7];
    uint8_t rxBuf[8];

    int16_t rawGyro[3];
    float gyroDPS[3];
    float gyroRPS[3];
    float gyroDPSOffset[3];

    int16_t rawAcc[3];
    float accGs[3];
    float accMPS[3];

    float gyroConvDPS;
    float accConvG;

    int16_t rawTemp;
    float temp;
} lsm6dso;

uint8_t LSM_ReadRegister(lsm6dso *imu, uint8_t addr, uint8_t *data);
uint8_t LSM_WriteRegister(lsm6dso *imu, uint8_t regAddr, uint8_t data);
uint8_t LSM_init(lsm6dso *imu, SPI_HandleTypeDef *spiHandle,
        GPIO_TypeDef *csPinBank, uint16_t csPin);
uint8_t LSM_pollsensors(lsm6dso *imu);
uint8_t LSM_Convert(lsm6dso *imu);


#endif /* INC_DRIVERS_LSM6DSO32_H_ */
