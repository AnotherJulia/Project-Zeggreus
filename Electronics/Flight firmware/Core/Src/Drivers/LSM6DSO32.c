/*
 * SPL06.c
 *
 *  Created on: 18 Mar 2021
 *      Author: elvin
 */


#include "LSM6DSO32.h"


uint8_t LSM_ReadRegister(lsm6dso *imu, uint8_t addr, uint8_t *data) {
    uint8_t txBuf[2] = { addr | 0x80, 0x00 }; // dummy byte in the middle
    uint8_t rxBuf[2];

    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);

    uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2,
    HAL_MAX_DELAY) == HAL_OK);

    while (HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY)
        ;

    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

    if (status == 1) {
        *data = rxBuf[1];
    }

    return status;

}

uint8_t LSM_WriteRegister(lsm6dso *imu, uint8_t regAddr, uint8_t data) {
    uint8_t txBuf[2] = { regAddr, data };

    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);

    uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY)
            == HAL_OK);

    while (HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

    return status;
}

uint8_t LSM_init(lsm6dso *imu, SPI_HandleTypeDef *spiHandle,
        GPIO_TypeDef *csPinBank, uint16_t csPin) {
    imu->spiHandle = spiHandle;
    imu->csPinBank = csPinBank;
    imu->csPin = csPin;

    // check
    uint8_t data;

    LSM_ReadRegister(imu, LSM_WHO_AM_I, &data);

    if (data != 0x6C) {
        return 1;
    }

    // software reset
    LSM_WriteRegister(imu, LSM_CTRL3_C, 0x01);
    HAL_Delay(40);

    // LSM_WriteRegister(imu, LSM_CTRL1_XL, 0b10101100); // 1010(6.66kHz accelerometer) 11(+/- 16g) 00
    LSM_WriteRegister(imu, LSM_CTRL1_XL, 0b10100100); // 1010(6.66kHz accelerometer) 01(+/- 32g) 00

    HAL_Delay(1);

    LSM_WriteRegister(imu, LSM_CTRL2_G, 0b10101100); // 1010 (6.66kHz gyro) 11(2000dps FS) 00
    HAL_Delay(1);

    // 0.070 dps/LSB if +/- 2000 dps
    // https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
    imu->gyroConvDPS = 0.070;

    // 0.976 mg/LSB if +/- 32 G
    imu->accConvG =  0.000976;



    LSM_WriteRegister(imu, LSM_CTRL4_C, 0b00001010); // 00001(drdy_mask until filter inits) 0 1(gyro LPF enable) 0
    HAL_Delay(1);

    LSM_WriteRegister(imu, LSM_CTRL6_C, 0b00000000); // 00000 <000>(gyroscope LPF1 bandwidth) 010 = 171 Hz
    HAL_Delay(1);

    LSM_WriteRegister(imu, LSM_CTRL7_G, 0b00000000); // <0>(high perf mode) <1>(gyro HPF) <00>(16mHz) 0000
    HAL_Delay(1);

    LSM_WriteRegister(imu, LSM_INT1_CTRL, 0b00000010); // gyro data ready interrupt
    HAL_Delay(1);

    LSM_WriteRegister(imu, LSM_COUNTER_BDR_REG1, 0b10000000); // 1 (pulsed data ready) 0000000
    HAL_Delay(1);



    HAL_Delay(100);

    imu->gyroDPSOffset[0] = 0;
    imu->gyroDPSOffset[1] = 0;
    imu->gyroDPSOffset[2] = 0;

    return 0;
    uint16_t calSamples = 100;

    float calcOffset[3] = {0,0,0};

    for (uint16_t i = 0; i < calSamples; i++) {

        LSM_pollsensors(imu);

        calcOffset[0] += imu->gyroDPS[0];
        calcOffset[1] += imu->gyroDPS[1];
        calcOffset[2] += imu->gyroDPS[2];

        HAL_Delay(2);
    }

    imu->gyroDPSOffset[0] = 0; //calcOffset[0] / calSamples;
    imu->gyroDPSOffset[1] = 0; //calcOffset[0] / calSamples;
    imu->gyroDPSOffset[2] = 0; //0calcOffset[0] / calSamples;


    return 0;
}

uint8_t LSM_pollsensors(lsm6dso *imu) {
    uint8_t txBuf[13] = { LSM_OUTX_L_G | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // dummy bytes
    uint8_t rxBuf[13];

    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);

    uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 13,
    HAL_MAX_DELAY) == HAL_OK);

    while (HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY)
        ;

    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

    if (status == 1) {
        // 0, gxl, gxh, gyl, gyh etc.
        imu->rawGyro[0] = rxBuf[2] << 8 | rxBuf[1];
        imu->rawGyro[1] = rxBuf[4] << 8 | rxBuf[3];
        imu->rawGyro[2] = rxBuf[6] << 8 | rxBuf[5];

        imu->rawAcc[0] = rxBuf[8] << 8 | rxBuf[7];
        imu->rawAcc[1] = rxBuf[10] << 8 | rxBuf[9];
        imu->rawAcc[2] = rxBuf[12] << 8 | rxBuf[11];

        // convert units
        LSM_Convert(imu);

    }

    return status;
}

uint8_t LSM_Convert(lsm6dso *imu) {
    imu->accGs[0] = imu->rawAcc[0] * imu->accConvG;
    imu->accGs[1] = imu->rawAcc[1] * imu->accConvG;
    imu->accGs[2] = imu->rawAcc[2] * imu->accConvG;

    imu->accMPS[0] = imu->accGs[0] * standardGravity;
    imu->accMPS[1] = imu->accGs[1] * standardGravity;
    imu->accMPS[2] = imu->accGs[2] * standardGravity;

    imu->gyroDPS[0] = imu->rawGyro[0] * imu->gyroConvDPS - imu->gyroDPSOffset[0];
    imu->gyroDPS[1] = imu->rawGyro[1] * imu->gyroConvDPS - imu->gyroDPSOffset[1];
    imu->gyroDPS[2] = imu->rawGyro[2] * imu->gyroConvDPS - imu->gyroDPSOffset[2];

    imu->gyroRPS[0] = imu->gyroDPS[0] * PI / 180;
    imu->gyroRPS[1] = imu->gyroDPS[1] * PI / 180;
    imu->gyroRPS[2] = imu->gyroDPS[2] * PI / 180;
}

uint8_t LSM_ReadDMA(lsm6dso *imu) {
    uint8_t txBuf[13] = { LSM_OUTX_L_G | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // dummy bytes
    uint8_t rxBuf[13];

    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, txBuf, imu->rxBuf, 13) == HAL_OK) {
        return 1;
    }
    else {
        HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
        return 0;
    }

}

void LSM_ReadDMA_Complete(lsm6dso *imu) {
    HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

    imu->rawGyro[0] = (uint16_t) (imu->rxBuf[2] << 8 | imu->rxBuf[1]);
    imu->rawGyro[1] = (uint16_t) (imu->rxBuf[4] << 8 | imu->rxBuf[3]);
    imu->rawGyro[2] = (uint16_t) (imu->rxBuf[6] << 8 | imu->rxBuf[5]);

    imu->rawAcc[0] = (uint16_t) (imu->rxBuf[8] << 8 | imu->rxBuf[7]);
    imu->rawAcc[1] = (uint16_t) (imu->rxBuf[10] << 8 | imu->rxBuf[9]);
    imu->rawAcc[2] = (uint16_t) (imu->rxBuf[12] << 8 | imu->rxBuf[11]);
    LSM_Convert(imu);
}
