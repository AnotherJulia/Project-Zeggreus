/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
//#include "radio.h"
//#include "sx1280.h"
#include "SPL06.h"
#include "LSM6DSO32.h"
#include "servo.h"
#include "Quaternion.h"
#include "orientation.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void playtone(uint16_t freq, uint16_t ms, uint8_t vol);
void changeLed(uint8_t ledR, uint8_t ledG, uint8_t ledB);
void jingleBell();
void rick();
void ksp();

void SetStandbyRC() {
    uint8_t loraRxBuf[2];
    uint8_t loraTxBuf[] = { 0x80, 0x00 }; // Standby RC
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, 2,
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetTxContinuousWave() {
    uint8_t loraRxBuf[1];
    uint8_t loraTxBuf[] = { 0xD1 }; // ContinousWave

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, 1,
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetRfFrequency() {
    // 52e6/(2^18) multiples of
    // 2.46 = 2.46 * 10^9/(52e6/(2^18)) = 12401428 = 0xBD3B14
    // uint32_t rfFreq = 12401428;

    uint8_t loraRxBuf[4];
    uint8_t loraTxBuf[] = { 0x86, 0xBD, 0x3B, 0x14 }; // SetRfFrequency

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, 4,
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetRfFrequency2() {
    // 52e6/(2^18) multiples of
    // 2.46 = 2.46 * 10^9/(52e6/(2^18)) = 12401428 = 0xBD3B14
    // uint32_t rfFreq = 12401428;

    uint8_t loraRxBuf[4];
    uint8_t loraTxBuf[] = { 0x86, 0xBE, 0xC4, 0xEC }; // SetRfFrequency

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, 4,
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void setPacketLora() {
    uint8_t loraRxBuf[2];
    uint8_t loraTxBuf[] = { 0x8A, 0x01 }; // Set packet to lora

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, 2,
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetTxParams(uint8_t power, uint8_t rampTime) {
    uint8_t loraRxBuf[3];
    // Set to -12 dBm = 0.06 mW
    uint8_t loraTxBuf[] = { 0x8E, power, rampTime};

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, sizeof(loraTxBuf),
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    uint8_t loraRxBuf[3];
    uint8_t loraTxBuf[] = { 0x8F, txBaseAddress, rxBaseAddress };

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, sizeof(loraTxBuf),
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetModulationParams(uint8_t modParam1, uint8_t modParam2, uint8_t modParam3) {
    uint8_t loraRxBuf[4];
    uint8_t loraTxBuf[] = { 0x8B, modParam1, modParam2, modParam3};

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    uint8_t loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, sizeof(loraTxBuf),
            1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetPacketParamsLora(uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, uint8_t param5) {
    uint8_t loraTxBuf[] = { 0x8C, param1, param2, param3, param4, param5};
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, loraTxBuf, sizeof(loraTxBuf), 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void WriteBuffer(uint8_t offset, uint8_t *data, uint8_t size) {
    uint8_t loraTxBuf[] = {0x1A, offset};
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, loraTxBuf, sizeof(loraTxBuf), 1000);
    HAL_SPI_Transmit(&hspi3, data, size, 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

}


void ReadBuffer(uint8_t offset, uint8_t size, uint8_t *data) {
    uint8_t loraTxBuf[] = { 0x1B, offset};
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, loraTxBuf, sizeof(loraTxBuf), 1000);
    HAL_SPI_Receive(&hspi3, data, size, 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask ) {
    uint8_t buf[9];
    buf[0] = 0x8D;
    buf[1] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( irqMask & 0x00FF );
    buf[3] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[4] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[5] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[6] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[7] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[8] = ( uint8_t )( dio3Mask & 0x00FF );

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, buf, sizeof(buf), 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void ClrIrqStatus(uint16_t irqMask) {
    uint8_t buf[3];
    buf[0] = 0X97;
    buf[1] = ( uint8_t )( ( ( uint16_t )irqMask >> 8 ) & 0x00FF );
    buf[2] = ( uint8_t )( ( uint16_t )irqMask & 0x00FF );

    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, buf, sizeof(buf), 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetTx(uint8_t periodBase, uint16_t periodBaseCount) {
    uint8_t buf[4];
    buf[0] = 0X83;
    buf[1] = periodBase;
    buf[2] = ( uint8_t )( ( ( uint16_t )periodBaseCount >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( ( uint16_t )periodBaseCount & 0x00FF );
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, buf, sizeof(buf), 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void SetRx(uint8_t periodBase, uint16_t periodBaseCount) {
    uint8_t buf[4];
    buf[0] = 0X82;
    buf[1] = periodBase;
    buf[2] = ( uint8_t )( ( ( uint16_t )periodBaseCount >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( ( uint16_t )periodBaseCount & 0x00FF );
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, buf, sizeof(buf), 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void GetPacketStatus(uint64_t *packetStatus) {
    uint8_t loraTxBuf[] = { 0x1D};
    // Redo this with proper packetstatus type
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, loraTxBuf, sizeof(loraTxBuf), 1000);
    HAL_SPI_Receive(&hspi3, packetStatus, 5, 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}

void GetRxBufferStatus() {

}

void WriteRegisterByte(uint16_t address, uint8_t data) {
    uint8_t loraTxBuf[4];
    loraTxBuf[0] = 0x18;
    loraTxBuf[1] = ( uint8_t )( ( ( uint16_t )address >> 8 ) & 0x00FF );
    loraTxBuf[2] = ( uint8_t )( ( uint16_t )address & 0x00FF );
    loraTxBuf[3] = data;
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, loraTxBuf, sizeof(loraTxBuf), 1000);
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);
}



void loraTesting(uint8_t isTx) {

    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    SetStandbyRC();
    HAL_Delay(3);
    setPacketLora();
    HAL_Delay(2);
    SetRfFrequency2();
    HAL_Delay(2);

    SetBufferBaseAddresses(0,0); // 127
    HAL_Delay(1);
    SetModulationParams(0x90, 0x0A, 0x01); // Spreading factor 9, 1600 BW (0x0A), CR 4/5
    HAL_Delay(1);

    WriteRegisterByte( 0x925, 0x32 ); // must be used for SF9-12. Different for 5-8 (page 112)

    HAL_Delay(1);
    SetPacketParamsLora(12, 0x80, 32, 0x20, 0x40); // 12 symbol preamble, implicit header, 32 byte payload, CRC enabled, Normal IQ
    HAL_Delay(1);
    // testing: set to -12 dBm

    if (isTx) {
        //SetTxParams(0x06, 0xE0); // Power = 13 dBm (0x1F), Pout = -18 + power (dBm) ramptime = 20 us.
        //SetTxParams(0x00, 0xE0); // lowest power -18dBm
        SetTxParams(31, 0xE0); // Highest power. 12.5 dBm
        HAL_Delay(3);

        uint8_t data[] = {0,0,0,0};

        WriteBuffer(0, data, sizeof(data));
        HAL_Delay(1);

        SetDioIrqParams(1,1,0,0); // txdone on gpio1

        //SetTxContinuousWave();
        HAL_Delay(3);

        uint8_t ledon = 0;

        while (1) {
            ledon = !ledon;
            //data[1] = (rand()%5) * 30;
            //data[2] = (rand()%5) * 30;
            //data[3] = (rand()%5) * 30;
            data[2] = ledon * 100;

            changeLed(data[1], data[2], data[3]);

            WriteBuffer(0, data, sizeof(data));
            HAL_Delay(1);
            ClrIrqStatus(1); // clear txdone irq
            HAL_Delay(1);
            SetTx(0x02, 50); // time-out of 1ms * 50 = 50ms
            //SetRfFrequency2();
            //HAL_Delay(5000);
            //SetRfFrequency();
            //changeLed(0, 100, 0);
            HAL_Delay(500);
        }
    }
    else {
        // rx mode
        SetDioIrqParams(1<<1, 1<<1, 0, 0); //rxdone on gpio1
        HAL_Delay(1);


        uint8_t data[32];
        data[0] = 60;
        data[1] = 60;
        data[2] = 60;
        uint8_t rxStartBufferPointer = 1;

        changeLed(0, 100, 0);
        uint8_t counter = 0;

        while (1) {

            //SetRx(0x00, 0xffff); // continous rx
            SetRx(0x00, 0); // No timeout
            //SetRx(0x02, 200); // 200 ms timeout
            HAL_Delay(1);
            // wait for reception:
            while (!HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin)) {}

            //GetPacketStatus(); // TODO
            ClrIrqStatus(1<<1); // clear rxdone Irq
            HAL_Delay(1);
            //GetRxBufferStatus(); // TODO
            ReadBuffer(rxStartBufferPointer, 32, data);
            changeLed(data[1], data[2], data[3]);

            HAL_Delay(10);

        }

    }
}

void loraOrientation(uint8_t isTx) {

    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    SetStandbyRC();
    HAL_Delay(3);
    setPacketLora();
    HAL_Delay(2);
    SetRfFrequency2();
    HAL_Delay(2);

    SetBufferBaseAddresses(0,0); // 127
    HAL_Delay(1);
    SetModulationParams(0x90, 0x18, 0x01); // Spreading factor 9, 800 BW, CR 4/5
    HAL_Delay(1);

    WriteRegisterByte( 0x925, 0x32 ); // must be used for SF9-12. Different for 5-8 (page 112)

    HAL_Delay(1);
    SetPacketParamsLora(0b00010011, 0x80, 32, 0x20, 0x00); // 12 symbol preamble, implicit header, 32 byte payload, CRC enabled, inverted IQ
    HAL_Delay(1);
    // testing: set to -12 dBm

    float data[4];

    char printBuffer[128];

    if (isTx) {
        //SetTxParams(0x06, 0xE0); // Power = 13 dBm (0x1F), Pout = -18 + power (dBm) ramptime = 20 us.
        SetTxParams(0x00, 0xE0); // lowest power -18dBm
        HAL_Delay(3);

        lsm6dso imu;
        uint8_t lsm_init_status = LSM_init(&imu, &hspi2, SPI2_NSS_GPIO_Port,SPI2_NSS_Pin);

        Orientation ori;
        orientation_init(&ori);
        uint32_t counter = 0;

        data[0] = ori.orientationQuat.w;
        data[1] = ori.orientationQuat.v[0];
        data[2] = ori.orientationQuat.v[1];
        data[3] = ori.orientationQuat.v[2];

        WriteBuffer(0, (uint8_t*)data, sizeof(data));
        HAL_Delay(1);

        SetDioIrqParams(1,1,0,0); // txdone on gpio1

        //SetTxContinuousWave();
        HAL_Delay(3);

        uint32_t lasttime = HAL_GetTick();
        uint32_t nowtime = HAL_GetTick();
        float dt = 0;

        while (1) {


            LSM_pollsensors(&imu);
            changeLed(0, 0, 100);
            nowtime = HAL_GetTick();
            dt = (nowtime - lasttime)/ 1000.0;
            lasttime = nowtime;

            orientation_setGyro(&ori, imu.gyroRPS);
            orientation_setAcc(&ori, imu.accMPS);
            orientation_update(&ori, dt);

            counter++;

            if (counter % 20 == 0) {
                data[0] = ori.orientationQuat.w;
                data[1] = ori.orientationQuat.v[0];
                data[2] = ori.orientationQuat.v[1];
                data[3] = ori.orientationQuat.v[2];

                sprintf(printBuffer, "Quaternion: %f, %f, %f, %f\r\n",data[0],data[1],data[2],data[3]);
                //sprintf(printBuffer, "Quaternion: %f, %f, %f, %f\r\n",data[0],ori.orientationQuat.v[0],ori.orientationQuat.v[1],ori.orientationQuat.v[2]);
                CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));

                WriteBuffer(0, (uint8_t*)data, sizeof(data));
                HAL_Delay(1);
                ClrIrqStatus(1); // clear txdone irq
                HAL_Delay(1);
                SetTx(0x02, 50); // time-out of 1ms * 50 = 50ms
            }

            HAL_Delay(1);

        }
    }
    else {
        // rx mode
        SetDioIrqParams(1<<1, 1<<1, 0, 0); //rxdone on gpio1
        HAL_Delay(1);

        uint8_t rxStartBufferPointer = 1;

        changeLed(0, 100, 0);
        uint8_t counter = 0;

        while (1) {

            //SetRx(0x00, 0xffff); // continous rx
            SetRx(0x00, 0); // No timeout
            //SetRx(0x02, 200); // 200 ms timeout
            HAL_Delay(1);
            // wait for reception:
            while (!HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin)) {}

            //GetPacketStatus(); // TODO
            ClrIrqStatus(1<<1); // clear rxdone Irq
            HAL_Delay(1);
            //GetRxBufferStatus(); // TODO


            ReadBuffer(rxStartBufferPointer, sizeof(data), (uint8_t*)data);

            if (counter % 30 == 0) {
                sprintf(printBuffer, "Quaternion: %f, %f, %f, %f\r\n",data[0],data[1],data[2],data[3]);
                CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
            }
            HAL_Delay(1);

        }

    }
}


void servoToggleTest() {
    while (1) {
        htim2.Instance->CCR4 = 1000;
        HAL_Delay(2000);
        htim2.Instance->CCR4 = 2000;
        HAL_Delay(2000);

    };
}

void infiniteKSP() {
    while (1) {
        ksp();
    };
}

void startupMusic() {
    // Startup sound
    changeLed(100, 34, 20);

    changeLed(255, 0, 0);
    playtone(1046, 100, 10);
    HAL_Delay(100);
    changeLed(0, 255, 0);
    playtone(1319, 100, 10);
    return;
    HAL_Delay(100);
    changeLed(0, 0, 255);
    playtone(1175, 100, 20);
    HAL_Delay(100);

    changeLed(255, 0, 255);
    playtone(1568, 200, 30);
    HAL_Delay(300);
    //playtone(988, 200, 30);

    for (int i = 1400; i < 1900; i += 100) {

        uint32_t aar_val = 1e6 / (i);
        htim3.Instance->ARR = aar_val;
        htim3.Instance->CCR2 = aar_val * 10 / (2 * 100);

        changeLed((i - 1400) / 3, (i - 1400) / 3, (i - 1400) / 3);
        HAL_Delay(100);

        // reset to defaults
        //htim3.Instance->CCR2 = 0;
        //htim3.Instance->ARR = 256 - 1;
    }
}



void SDTesting() {
    // LSM6dso setup
    lsm6dso imu;
    uint8_t lsm_init_status = LSM_init(&imu, &hspi2, SPI2_NSS_GPIO_Port,
    SPI2_NSS_Pin);
    uint8_t sd_mounted = !HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin);

    if (!sd_mounted) {
        while (1) {
            changeLed(0, 0, 100);
            HAL_Delay(1000);
        }
    }

    changeLed(0, 0, 0);

    HAL_Delay(5000);

    changeLed(100, 100, 100);
    char printBuffer[128];
    sprintf(printBuffer, "SD CARD TEST!\r\n");

    CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));

    FATFS FatFs;  //Fatfs handle
    FIL fil;      //File handle
    FRESULT fres; //Result after operations

    //Open the file system
    fres = f_mount(&FatFs, "", 1); //1=mount now
    if (fres != FR_OK) {
        sprintf(printBuffer, "f_mount error (%i)\r\n", fres);
        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
        while (1) {
            CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
            HAL_Delay(100);
        }
    }

    //Let's get some statistics from the SD card
    DWORD free_clusters, free_sectors, total_sectors;

    FATFS *getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
        sprintf(printBuffer, "f_getfree error (%i)\r\n", fres);

        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
        while (1)
            ;
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    sprintf(printBuffer,
            "SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
            total_sectors / 2, free_sectors / 2);

    CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));

    //Now let's try to open file "test.txt"
    fres = f_open(&fil, "hello.txt", FA_READ);
    if (fres != FR_OK) {
        sprintf(printBuffer, "f_open error\r\n");

        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
        while (1)
            ;
    }

    sprintf(printBuffer, "I was able to open 'hello.txt' for reading!\r\n");
    CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));

    //Read 30 bytes from "test.txt" on the SD card
    BYTE readBuf[128];

    //We can either use f_read OR f_gets to get data out of files
    //f_gets is a wrapper on f_read that does some string formatting for us
    TCHAR *rres = f_gets((TCHAR*) readBuf, 30, &fil);
    if (rres != 0) {
        sprintf(printBuffer, "Read string from 'test.txt' contents: %s\r\n",
                readBuf);
        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
    } else {
        sprintf(printBuffer, "f_gets error (%i)\r\n", fres);
        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);

    //Now let's try and write a file "write.txt"
    fres = f_open(&fil, "write.txt",
    FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
    if (fres == FR_OK) {
        sprintf(printBuffer, "I was able to open 'write.txt' for writing\r\n");
        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
    } else {
        sprintf(printBuffer, "f_open error (%i)\r\n", fres);
        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
    }

    //Copy in a string
    strncpy((char*) readBuf, "a new file is made!", 19);
    UINT bytesWrote;
    fres = f_write(&fil, readBuf, 19, &bytesWrote);
    if (fres == FR_OK) {
        sprintf(printBuffer, "Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
    } else {
        sprintf(printBuffer, "f_write error (%i)\r\n");
        CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
    }

    //Be a tidy kiwi - don't forget to close your file!
    f_close(&fil);

    changeLed(0, 200, 200);

    f_open(&fil, "gyro.csv", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

    sprintf(printBuffer, "t,gx,gy,gz\r\n");
    strncpy((char*) readBuf, printBuffer, strlen(printBuffer));
    fres = f_write(&fil, readBuf, strlen(printBuffer), &bytesWrote);

    for (uint16_t i = 0; i < 15 * 1000; i++) { // 15 second test
        HAL_Delay(1);
        LSM_pollsensors(&imu);

        sprintf(printBuffer, "%d,%d,%d,%d\r\n", HAL_GetTick(), imu.rawGyro[1],
                (-1) * imu.rawGyro[0], imu.rawGyro[2]);
        strncpy((char*) readBuf, printBuffer, strlen(printBuffer));
        fres = f_write(&fil, readBuf, strlen(printBuffer), &bytesWrote);
    }
    f_close(&fil);

    //We're done, so de-mount the drive
    f_mount(NULL, "", 0);

    while (1) {
        changeLed(0, 200, 0);
        HAL_Delay(40);

    }

}

void BWtest() {

    enum state {
        INITIALIZATION, PADIDLE, ARMED, LAUNCHED, DEPLOYED
    };

    enum state currentState;
    currentState = ARMED;

    uint32_t launchTime = HAL_GetTick();
    uint32_t currentTime = HAL_GetTick();
    uint32_t deployDelay = 5000; // ms

    // reset servo
    htim2.Instance->CCR4 = 1000;

    Servo deployServo;
    servo_init(&deployServo, &htim2, &htim2.Instance->CCR4);
    servo_writeangle(&deployServo, 0);

    while (1) {
        HAL_Delay(1);

        currentTime = HAL_GetTick();

        switch (currentState) {
        case ARMED:

            if (HAL_GPIO_ReadPin(DEBUG_GPIO_Port, DEBUG_Pin)) {
                changeLed(100, 0, 0);
                currentState = LAUNCHED;
                launchTime = HAL_GetTick();
            } else {
                changeLed(0, 100, 0);
            }

            break;
        case LAUNCHED:

            playtone(2000, 100, 30);
            HAL_Delay(100);

            if ((currentTime - launchTime) > deployDelay) {
                //htim2.Instance->CCR4 = 2000;
                servo_writeangle(&deployServo, 180);
                changeLed(100, 100, 100);
                currentState = DEPLOYED;
            }
            break;

        case DEPLOYED:
            ksp();
            break;

        default:
            break;
        }

    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    startupMusic();

    changeLed(90, 0, 0);

    htim3.Instance->CCR2 = 0;
    htim3.Instance->ARR = 256 - 1;

    char printBuffer[128];
    HAL_Delay(500);


    //BWtest();
    uint8_t is_tx = 0;
    loraTesting(is_tx);
    //loraOrientation(is_tx);

    // LSM6dso setup
    lsm6dso imu;
    uint8_t lsm_init_status = LSM_init(&imu, &hspi2, SPI2_NSS_GPIO_Port,SPI2_NSS_Pin);

    SPL06 baro;
    uint8_t barostatus = SPL06_Init(&baro, &hi2c3, 0x77);

    if (barostatus != 5) {
        while (1) {

            HAL_Delay(100);
            changeLed(100, 0, 0);
            HAL_Delay(100);
            changeLed(0, 0, 0);

        }
    }

    //SDTesting();

    float yrot = 0;
    uint32_t lasttime = HAL_GetTick();
    uint32_t nowtime = HAL_GetTick();
    float dt = 0;

    uint16_t rawadc;

    Orientation ori;
    orientation_init(&ori);
    uint32_t counter = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        //SPL06_Read(&baro);
        LSM_pollsensors(&imu);

        //HAL_ADC_Start(&hadc1);
        //HAL_ADC_PollForConversion(&hadc1, 100);
        //rawadc = HAL_ADC_GetValue(&hadc1);

        //sprintf(printBuffer, "Pres:%f,temp:%f,%f\r\n", baro.pressure_Pa, baro.temperature_C);
        //sprintf(printBuffer, "%f\r\n", baro.pressure_Pa);
        //sprintf(printBuffer, "gx: %d, gy: %d, gz: %d\r\n", imu.rawGyro[0], imu.rawGyro[1], imu.rawGyro[2]);
        //sprintf(printBuffer, "gx: %d, gy: %d, gz: %d\r\n", imu.rawAcc[0], imu.rawAcc[1], imu.rawAcc[2]);
        //sprintf(printBuffer, "gx:%f,gy:%f,gz:%f\r\n", imu.accMPS[0], imu.accMPS[1], imu.accMPS[2]);
        //sprintf(printBuffer, "gx: %f, gy: %f, gz: %f\r\n", imu.gyroRPS[0], imu.gyroRPS[1], imu.gyroRPS[2]);
        //sprintf(printBuffer, "y:%f,o:%f,g:%f,V:%d\r\n", yrot, imu.gyroDPSOffset[1],
        //        imu.gyroDPS[1], rawadc);
        //sprintf(printBuffer, "T:%f\r\n", (float) (25 + (((rawadc - 943) * 3.3 / 4096.0)) / 0.0025));

        changeLed(0, 0, 100);
        nowtime = HAL_GetTick();
        dt = (nowtime - lasttime)/ 1000.0;
        lasttime = nowtime;

        orientation_setGyro(&ori, imu.gyroRPS);
        orientation_setAcc(&ori, imu.accMPS);
        orientation_update(&ori, dt);

        //sprintf(printBuffer, "z:%f,y:%f,x:%f\r\n", ori.eulerZYX[0], ori.eulerZYX[1], ori.eulerZYX[2]);
        if (counter % 30 == 0) {
            sprintf(printBuffer, "Quaternion: %f, %f, %f, %f\r\n",ori.orientationQuat.w,ori.orientationQuat.v[0],ori.orientationQuat.v[1],ori.orientationQuat.v[2]);
            //sprintf(printBuffer, "Quaternion: %f, %f, %f, %f\r\n",ori.horQuat.w,ori.horQuat.v[0],ori.horQuat.v[1],ori.horQuat.v[2]);
            //sprintf(printBuffer, "Counter: %d\r\n",counter);
            //sprintf(printBuffer, "gx: %d, gy: %d, gz: %d\r\n", imu.rawGyro[0], imu.rawGyro[1], imu.rawGyro[2]);
            CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
        }
        counter++;



        HAL_Delay(1);

        //HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);
        //loraRet = HAL_SPI_TransmitReceive(&hspi3, loraTxBuf, loraRxBuf, 2, 1000);
        //HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

        /*
         for (int i = 0; i < 31 * 2; i++) {
         changeLed((uint8_t) ((cos(((double) i) / 75) + 1) * 128),
         (uint8_t) ((sin(((double) i) / 50) + 1) * 128),
         (uint8_t) ((sin(((double) i) / 100) + 1) * 128));
         HAL_Delay(2);
         }
         */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 90-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 256-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_NSS_GPIO_Port, SD_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_NSS_Pin */
  GPIO_InitStruct.Pin = SD_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_Pin */
  GPIO_InitStruct.Pin = DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DEBUG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS_Pin LORA_RESET_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin|LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO1_Pin */
  GPIO_InitStruct.Pin = LORA_DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_BUSY_Pin */
  GPIO_InitStruct.Pin = LORA_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void playtone(uint16_t freq, uint16_t ms, uint8_t vol) {

    // 90MHz / (90 * ARR) = freq
    // 90MHz / (90 * freq) = ARR
    // 1MHz/(freq) = AAR

    // save LED's
    uint16_t ledR = htim3.Instance->CCR3;
    uint16_t ledG = htim3.Instance->CCR1;
    uint16_t ledB = htim3.Instance->CCR4;

    uint32_t aar_val = 1e6 / (freq);
    htim3.Instance->CNT = 0;
    htim3.Instance->ARR = aar_val;
    htim3.Instance->CCR2 = aar_val * vol / (2 * 100);
    // same LED brightness
    htim3.Instance->CCR3 = (aar_val * ledR) / 256;
    htim3.Instance->CCR1 = (aar_val * ledG) / 256;
    htim3.Instance->CCR4 = (aar_val * ledB) / 256;

    HAL_Delay(ms);

    // reset to defaults
    htim3.Instance->CCR2 = 0;
    htim3.Instance->ARR = 256 - 1;

    // back to normal LED
    htim3.Instance->CCR3 = ledR;
    htim3.Instance->CCR1 = ledG;
    htim3.Instance->CCR4 = ledB;
}

void changeLed(uint8_t ledR, uint8_t ledG, uint8_t ledB) {
    htim3.Instance->CCR3 = ledR;
    htim3.Instance->CCR1 = ledG;
    htim3.Instance->CCR4 = ledB;

}

#define G0 784
#define A0 880
#define B0 988
#define C1 1047
#define D1 1175
#define E1 1319
#define F1 1397
#define G1 1568
#define A1 1760

void jingleBell() {
    uint16_t vol = 10;
    uint16_t wait1 = 125;
    uint16_t wait2 = 255;

    uint16_t tones[] = { E1, E1, E1, E1, E1, E1,
    E1, G1, C1, D1, E1,
    F1, F1, F1, F1, F1, E1, E1,
    E1, E1, D1, D1, E1, D1, G1,

    E1, E1, E1, E1, E1, E1,
    E1, G1, C1, D1, E1,
    F1, F1, F1, F1, F1, E1, E1,
    E1, G1, G1, F1, D1, C1 };

    uint16_t tonelength[] = { wait1, wait1, wait2, wait1, wait1, wait2, wait1,
            wait1, wait1, wait1, wait2, wait1, wait1, wait1, wait1, wait1,
            wait1, wait1, wait1, wait1, wait1, wait1, wait1, wait2, wait2,

            wait1, wait1, wait2, wait1, wait1, wait2, wait1, wait1, wait1,
            wait1, wait2, wait1, wait1, wait1, wait1, wait1, wait1, wait1,
            wait1, wait1, wait1, wait1, wait1, wait2 };

    uint16_t delays[] = { wait1, wait1, wait2, wait1, wait1, wait2, wait1,
            wait1, wait1, wait1, wait1 * 3, wait1, wait1, wait1, wait1, wait1,
            wait1, wait1, wait1, wait1, wait1, wait1, wait1, wait2, wait2,

            wait1, wait1, wait2, wait1, wait1, wait2, wait1, wait1, wait1,
            wait1, wait1 * 3, wait1, wait1, wait1, wait1, wait1, wait1, wait1,
            wait1, wait1, wait1, wait1, wait1, wait2 };

    uint8_t num_notes = sizeof(tonelength) / sizeof(uint16_t);

    for (uint8_t i = 0; i < num_notes; i++) {
        playtone(tones[i], tonelength[i], vol);
        HAL_Delay(delays[i]);
    }

}

#define  a3f    208     // 208 Hz
#define  b3f    233     // 233 Hz
#define  b3     247     // 247 Hz
#define  c4     261     // 261 Hz MIDDLE C
#define  c4s    277     // 277 Hz
#define  e4f    311     // 311 Hz
#define  f4     349     // 349 Hz
#define  a4f    415     // 415 Hz
#define  b4f    466     // 466 Hz
#define  b4     493     // 493 Hz
#define  c5     523     // 523 Hz
#define  c5s    554     // 554 Hz
#define  e5f    622     // 622 Hz
#define  f5     698     // 698 Hz
#define  f5s    740     // 740 Hz
#define  a5f    831     // 831 Hz

#define rest    -1

void rick() {
    uint16_t vol = 10;
    uint16_t beatlength = 50; // determines tempo
    float beatseparationconstant = 0.3;

    uint16_t song1_intro_melody[] = { c5s, e5f, e5f, f5, a5f, f5s, f5, e5f, c5s,
    e5f, rest, a4f, a4f };

    uint16_t song1_intro_rhythmn[] =
            { 6, 10, 6, 6, 1, 1, 1, 1, 6, 10, 4, 2, 10 };

    uint16_t song1_verse1_melody[] = { rest, c4s, c4s, c4s, c4s, e4f, rest, c4,
    b3f, a3f,
    rest, b3f, b3f, c4, c4s, a3f, a4f, a4f, e4f,
    rest, b3f, b3f, c4, c4s, b3f, c4s, e4f, rest, c4, b3f, b3f, a3f,
    rest, b3f, b3f, c4, c4s, a3f, a3f, e4f, e4f, e4f, f4, e4f,
    c4s, e4f, f4, c4s, e4f, e4f, e4f, f4, e4f, a3f,
    rest, b3f, c4, c4s, a3f, rest, e4f, f4, e4f };

    uint16_t song1_verse1_rhythmn[] = { 2, 1, 1, 1, 1, 2, 1, 1, 1, 5, 1, 1, 1,
            1, 3, 1, 2, 1, 5, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 3, 1, 1, 1, 1,
            2, 1, 1, 1, 1, 1, 1, 4, 5, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 3,
            1, 1, 1, 3 };

    uint16_t song1_chorus_melody[] = { b4f, b4f, a4f, a4f,
    f5, f5, e5f, b4f, b4f, a4f, a4f, e5f, e5f, c5s, c5, b4f,
    c5s, c5s, c5s, c5s,
    c5s, e5f, c5, b4f, a4f, a4f, a4f, e5f, c5s,
    b4f, b4f, a4f, a4f,
    f5, f5, e5f, b4f, b4f, a4f, a4f, a5f, c5, c5s, c5, b4f,
    c5s, c5s, c5s, c5s,
    c5s, e5f, c5, b4f, a4f, rest, a4f, e5f, c5s, rest };

    uint16_t song1_chorus_rhythmn[] =
            { 1, 1, 1, 1, 3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2, 1, 1, 1, 1, 3, 3,
                    3, 1, 2, 2, 2, 4, 8, 1, 1, 1, 1, 3, 3, 6, 1, 1, 1, 1, 3, 3,
                    3, 1, 2, 1, 1, 1, 1, 3, 3, 3, 1, 2, 2, 2, 4, 8, 4 };

    int a = 4; // part index
    int b = 0; // song index
    int c; // lyric index

    while (1) {

        uint16_t notelength;
        if (a == 1 || a == 2) {
            // intro
            notelength = beatlength * song1_intro_rhythmn[b];
            if (song1_intro_melody[b] > 0) {
                playtone(song1_intro_melody[b], notelength, vol);
            }
            b++;
            if (b >= sizeof(song1_intro_melody) / sizeof(uint16_t)) {
                a++;
                b = 0;
                c = 0;
            }
        } else if (a == 3 || a == 5) {
            // verse
            notelength = beatlength * 2 * song1_verse1_rhythmn[b];
            if (song1_verse1_melody[b] > 0) {
                playtone(song1_verse1_melody[b], notelength, vol);
                c++;
            }
            b++;
            if (b >= sizeof(song1_verse1_melody) / sizeof(uint16_t)) {
                a++;
                b = 0;
                c = 0;
            }
        } else if (a == 4 || a == 6) {
            // chorus
            notelength = beatlength * song1_chorus_rhythmn[b];
            if (song1_chorus_melody[b] > 0) {
                playtone(song1_chorus_melody[b], notelength, vol);
                c++;
            }
            b++;
            if (b >= sizeof(song1_chorus_melody) / sizeof(uint16_t)) {
                a++;
                b = 0;
                c = 0;
            }
        }

        HAL_Delay(notelength);
        //noTone(piezo);

        HAL_Delay(notelength * beatseparationconstant);
        if (a == 7) { // loop back around to beginning of song
            a = 1;
        }
    }

}

void ksp() {
    int vol = 20;
    playtone(659, 1000, vol); // EHigh
    playtone(523, 1000, vol); //
    playtone(783, 1000, vol);
    playtone(523, 333, vol);
    playtone(659, 333, vol);
    playtone(784, 333, vol);
    playtone(932, 1000, vol);
    playtone(880, 1000, vol);
    playtone(784, 1000, vol);
    playtone(523, 333, vol);
    playtone(659, 333, vol);
    playtone(784, 333, vol);
    playtone(932, 1000, vol);
    playtone(880, 1000, vol);
    playtone(784, 1000, vol);
    playtone(523, 1000, vol);
    playtone(587, 1000, vol);
    playtone(698, 2000, vol);
    playtone(587, 500, vol);
    playtone(523, 500, vol);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {

    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/