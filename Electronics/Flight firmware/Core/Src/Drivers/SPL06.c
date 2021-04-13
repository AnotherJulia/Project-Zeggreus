/*
 * SPL06.c
 *
 *  Created on: 2 Mar 2021
 *      Author: elvin
 */


#include <SPL06.h>



/*
 *
 * INITIALISATION
 *
 */
uint8_t SPL06_Init(SPL06 *bar, I2C_HandleTypeDef *i2cHandle, uint8_t i2cAddress) {

    uint8_t status = 0;

    /* Store peripheral data */
    bar->i2cHandle = i2cHandle;
    bar->i2cAddress = i2cAddress << 1;

    /* Clear measurements */
    bar->pressure_Pa   = 0.0f;
    bar->temperature_C = 0.0f;

    uint8_t txBuf[2] = {0x00, 0x00};
    uint8_t rxBuf[2];

    /* Check device ID */
    uint8_t id;

    status += (HAL_I2C_Mem_Read(bar->i2cHandle, bar->i2cAddress, SPL06_ID, I2C_MEMADD_SIZE_8BIT, rxBuf, 1, 500) == HAL_OK);

    id = rxBuf[0];

    /* Make sure device ID matches */
    if (id != 0x10) {

        return 0;

    }
    HAL_Delay(10);

    /* Read calibration coefficients */
    uint8_t calibTxBuf[19];
    calibTxBuf[0] = (SPL06_COEF | 0x80);

    uint8_t calibRxBuf[19];

    //status += (HAL_SPI_TransmitReceive(bar->spiHandle, calibTxBuf, calibRxBuf, 19, HAL_MAX_DELAY) == HAL_OK);
    status += (HAL_I2C_Mem_Read(bar->i2cHandle, bar->i2cAddress, SPL06_COEF, I2C_MEMADD_SIZE_8BIT, calibRxBuf, 19, 500) == HAL_OK);


    /* Convert raw calibration coefficients to signed integers */
    bar->c0 = (uint16_t)calibRxBuf[0] << 4 | (uint16_t)calibRxBuf[1] >> 4;
    bar->c0 = (bar->c0 & 1 << 11) ? (0xF000 | bar->c0) : bar->c0;

    bar->c1 = (uint16_t)(calibRxBuf[1] & 0x0f) << 8 | (uint16_t)calibRxBuf[2];
    bar->c1 = (bar->c1 & 1 << 11) ? (0xF000 | bar->c1) : bar->c1;

    bar->c00 = (uint32_t)calibRxBuf[3] << 12 | (uint32_t)calibRxBuf[4] << 4 | (uint16_t)calibRxBuf[5] >> 4;
    bar->c00 = (bar->c00 & 1 << 19) ? (0xFFF00000 | bar->c00) : bar->c00;

    bar->c10 = (uint32_t)(calibRxBuf[5] & 0x0f) << 16 | (uint32_t)calibRxBuf[6] << 8 | (uint32_t)calibRxBuf[7];
    bar->c10 = (bar->c10 & 1 << 19) ? (0xFFF00000 | bar->c10) : bar->c10;

    bar->c01 = (uint16_t) calibRxBuf[8]  << 8 | calibRxBuf[9];
    bar->c11 = (uint16_t) calibRxBuf[10] << 8 | calibRxBuf[11];
    bar->c20 = (uint16_t) calibRxBuf[12] << 8 | calibRxBuf[13];
    bar->c21 = (uint16_t) calibRxBuf[14] << 8 | calibRxBuf[15];
    bar->c30 = (uint16_t) calibRxBuf[16] << 8 | calibRxBuf[17];
    HAL_Delay(25);

    /* Set pressure configuration */
    txBuf[0] = 0x33;

    status += (HAL_I2C_Mem_Write(bar->i2cHandle, bar->i2cAddress, SPL06_PRS_CFG, I2C_MEMADD_SIZE_8BIT, txBuf, 1, 500) == HAL_OK);


    HAL_Delay(10);

    /* Set temperature configuration */
    txBuf[0] = 0xB3;

    status += (HAL_I2C_Mem_Write(bar->i2cHandle, bar->i2cAddress, SPL06_TMP_CFG, I2C_MEMADD_SIZE_8BIT, txBuf, 1, 500) == HAL_OK);
    //status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);

    HAL_Delay(10);

    /* Set measurement configuration */
    txBuf[0] = 0xFF ;

    status += (HAL_I2C_Mem_Write(bar->i2cHandle, bar->i2cAddress, SPL06_MEAS_CFG, I2C_MEMADD_SIZE_8BIT, txBuf, 1, 500) == HAL_OK);
    //status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);


    return status;

}

/*
 *
 * TEMPERATURE AND PRESSURE READ (POLLING)
 *
 */
void SPL06_Read(SPL06 *bar) {

    uint8_t txBuf[7];
    txBuf[0] = 0x00 | 0x80;

    uint8_t rxBuf[6];

    //HAL_I2C_Mem_Read(bar->i2cHandle, bar->i2cAddress, 0x80, I2C_MEMADD_SIZE_8BIT, rxBuf, 1, 500);
    HAL_I2C_Mem_Read(bar->i2cHandle, bar->i2cAddress, 0x00, I2C_MEMADD_SIZE_8BIT, rxBuf, 6, 500);

    /* Convert raw to uncalibrated pressure and temperature */
    int32_t pres = ((uint32_t) rxBuf[0] << 16) | ((uint32_t) rxBuf[1] << 8) | ((uint32_t) rxBuf[2]);
            pres = (pres & 1 << 23) ? (0xFF000000 | pres) : pres;

    int32_t temp = ((uint32_t) rxBuf[3] << 16) | ((uint32_t) rxBuf[4] << 8) | ((uint32_t) rxBuf[5]);
            temp = (temp & 1 << 23) ? (0xFF000000 | temp) : temp;

    /* Apply calibration */
    float tempRaw = (float) temp / 7864320.0f;
    bar->temperature_C = 0.5f * bar->c0 + bar->c1 * tempRaw;

    float presRaw   = (float) pres / 7864320.0f;
    bar->pressure_Pa = bar->c00 + presRaw * (bar->c10 + presRaw * (bar->c20 + bar->c30 * presRaw))
                    + tempRaw * (bar->c01 + presRaw * (bar->c11 + bar->c21 * presRaw));

}
