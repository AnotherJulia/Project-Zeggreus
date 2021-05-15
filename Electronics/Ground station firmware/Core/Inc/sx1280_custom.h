#include "stm32f4xx_hal.h"


typedef struct {
    SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef *csPinBank;
    uint16_t csPin;

    GPIO_TypeDef *Dio1PinBank;
    uint16_t Dio1Pin;


    uint8_t txBuf[8];
    uint8_t rxBuf[8];

    uint8_t spi_return;

    uint8_t rssiSync;
    int8_t rawSnr;

    float rssi;
    float snr;

} sx1280_custom;

void sxInit(sx1280_custom *radio,SPI_HandleTypeDef *spiHandle,
        GPIO_TypeDef *csPinBank, uint16_t csPin);

void sxSpiTransmit(sx1280_custom *radio, uint8_t *txBuf, uint8_t size);
void sxSpiTransmitReceive(sx1280_custom *radio, uint8_t *txBuf, uint8_t *rxBuf, uint8_t size);

void SetStandbyRC(sx1280_custom *radio);
void SetTxContinuousWave(sx1280_custom *radio);
void SetRfFrequency(sx1280_custom *radio);
void SetRfFrequency2(sx1280_custom *radio);
void setPacketLora(sx1280_custom *radio);
void SetTxParams(sx1280_custom *radio, uint8_t power, uint8_t rampTime);
void SetBufferBaseAddresses(sx1280_custom *radio, uint8_t txBaseAddress, uint8_t rxBaseAddress);
void SetModulationParams(sx1280_custom *radio, uint8_t modParam1, uint8_t modParam2,
        uint8_t modParam3);
void SetPacketParamsLora(sx1280_custom *radio, uint8_t param1, uint8_t param2, uint8_t param3,
        uint8_t param4, uint8_t param5);
void WriteBuffer(sx1280_custom *radio, uint8_t offset, uint8_t *data, uint8_t size);
void ReadBuffer(sx1280_custom *radio, uint8_t offset, uint8_t size, uint8_t *data);
void SetDioIrqParams(sx1280_custom *radio, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask,
        uint16_t dio3Mask);

void ClrIrqStatus(sx1280_custom *radio, uint16_t irqMask);

void SetTx(sx1280_custom *radio, uint8_t periodBase, uint16_t periodBaseCount);
void SetRx(sx1280_custom *radio, uint8_t periodBase, uint16_t periodBaseCount);

void GetPacketStatusLora(sx1280_custom *radio);

void GetRxBufferStatus(sx1280_custom *radio);

void WriteRegisterByte(sx1280_custom *radio, uint16_t address, uint8_t data);
void sxSetDio1Pin(sx1280_custom *radio, GPIO_TypeDef *Dio1PinBank, uint16_t Dio1Pin);
