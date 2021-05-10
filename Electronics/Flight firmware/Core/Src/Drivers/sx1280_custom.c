#include "main.h"
#include "sx1280_custom.h"


void sxSpiTransmit(sx1280_custom *radio, uint8_t *txBuf, uint8_t size) {
    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_RESET);
    radio->spi_return = HAL_SPI_Transmit(radio->spiHandle, txBuf, size, 1000);
    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_SET);
}
void sxSpiTransmitReceive(sx1280_custom *radio, uint8_t *txBuf, uint8_t *rxBuf, uint8_t size) {
    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_RESET);
    radio->spi_return = HAL_SPI_TransmitReceive(radio->spiHandle,txBuf,rxBuf,size,1000);
    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_SET);
}


void sxInit(sx1280_custom *radio, SPI_HandleTypeDef *spiHandle,
        GPIO_TypeDef *csPinBank, uint16_t csPin) {

    radio->spiHandle = spiHandle;
    radio->csPinBank = csPinBank;
    radio->csPin = csPin;


    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    SetStandbyRC(radio);
    HAL_Delay(3);
    setPacketLora(radio);
    HAL_Delay(2);
    SetRfFrequency2(radio);
    HAL_Delay(2);

    SetBufferBaseAddresses(radio, 0, 0); // 127
    HAL_Delay(1);
    SetModulationParams(radio, 0x90, 0x0A, 0x01); // Spreading factor 9, 1600 BW (0x0A), CR 4/5
    HAL_Delay(1);

    WriteRegisterByte(radio, 0x925, 0x32); // must be used for SF9-12. Different for 5-8 (page 112)

    HAL_Delay(1);
    SetPacketParamsLora(radio, 12, 0x80, 32, 0x20, 0x40); // 12 symbol preamble, implicit header, 32 byte payload, CRC enabled, Normal IQ
    HAL_Delay(1);
}

void sxSetDio1Pin(sx1280_custom *radio, GPIO_TypeDef *Dio1PinBank, uint16_t Dio1Pin) {
    radio->Dio1PinBank = Dio1PinBank;
    radio->Dio1Pin = Dio1Pin;
}

void SetStandbyRC(sx1280_custom *radio) {
    uint8_t loraRxBuf[2];
    uint8_t loraTxBuf[] = { 0x80, 0x00 }; // Standby RC

    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, 2);
}

void SetTxContinuousWave(sx1280_custom *radio) {
    uint8_t loraRxBuf[1];
    uint8_t loraTxBuf[] = { 0xD1 }; // ContinousWave
    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, 1);
}

void SetRfFrequency(sx1280_custom *radio) {
    // 52e6/(2^18) multiples of
    // 2.46 = 2.46 * 10^9/(52e6/(2^18)) = 12401428 = 0xBD3B14
    // uint32_t rfFreq = 12401428;

    uint8_t loraRxBuf[4];
    uint8_t loraTxBuf[] = { 0x86, 0xBD, 0x3B, 0x14 }; // SetRfFrequency
    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, 4);
}

void SetRfFrequency2(sx1280_custom *radio) {
    // 52e6/(2^18) multiples of
    // 2.46 = 2.46 * 10^9/(52e6/(2^18)) = 12401428 = 0xBD3B14
    // uint32_t rfFreq = 12401428;

    uint8_t loraRxBuf[4];
    uint8_t loraTxBuf[] = { 0x86, 0xBE, 0xC4, 0xEC }; // SetRfFrequency
    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, 4);
}

void setPacketLora(sx1280_custom *radio) {
    uint8_t loraRxBuf[2];
    uint8_t loraTxBuf[] = { 0x8A, 0x01 }; // Set packet to lora
    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, 2);
}

void SetTxParams(sx1280_custom *radio, uint8_t power, uint8_t rampTime) {
    uint8_t loraRxBuf[3];
    // Set to -12 dBm = 0.06 mW
    uint8_t loraTxBuf[] = { 0x8E, power, rampTime };
    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, 3);
}

void SetBufferBaseAddresses(sx1280_custom *radio, uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    uint8_t loraRxBuf[3];
    uint8_t loraTxBuf[] = { 0x8F, txBaseAddress, rxBaseAddress };

    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, sizeof(loraTxBuf));
}

void SetModulationParams(sx1280_custom *radio, uint8_t modParam1, uint8_t modParam2,
        uint8_t modParam3) {
    uint8_t loraRxBuf[4];
    uint8_t loraTxBuf[] = { 0x8B, modParam1, modParam2, modParam3 };

    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, sizeof(loraTxBuf));
}

void SetPacketParamsLora(sx1280_custom *radio, uint8_t param1, uint8_t param2, uint8_t param3,
        uint8_t param4, uint8_t param5) {
    uint8_t loraTxBuf[] = { 0x8C, param1, param2, param3, param4, param5 };
    sxSpiTransmit(radio, loraTxBuf, sizeof(loraTxBuf));
}

void WriteBuffer(sx1280_custom *radio, uint8_t offset, uint8_t *data, uint8_t size) {
    uint8_t loraTxBuf[] = { 0x1A, offset };

    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->spiHandle, loraTxBuf, sizeof(loraTxBuf), 1000);
    HAL_SPI_Transmit(radio->spiHandle, data, size, 1000);
    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_SET);

}

void ReadBuffer(sx1280_custom *radio, uint8_t offset, uint8_t size, uint8_t *data) {
    uint8_t loraTxBuf[] = { 0x1B, offset };
    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(radio->spiHandle, loraTxBuf, sizeof(loraTxBuf), 1000);
    HAL_SPI_Receive(radio->spiHandle, data, size, 1000);
    HAL_GPIO_WritePin(radio->csPinBank, radio->csPin, GPIO_PIN_SET);
}

void SetDioIrqParams(sx1280_custom *radio, uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask,
        uint16_t dio3Mask) {
    uint8_t loraTxBuf[9];
    loraTxBuf[0] = 0x8D;
    loraTxBuf[1] = (uint8_t) ((irqMask >> 8) & 0x00FF);
    loraTxBuf[2] = (uint8_t) (irqMask & 0x00FF);
    loraTxBuf[3] = (uint8_t) ((dio1Mask >> 8) & 0x00FF);
    loraTxBuf[4] = (uint8_t) (dio1Mask & 0x00FF);
    loraTxBuf[5] = (uint8_t) ((dio2Mask >> 8) & 0x00FF);
    loraTxBuf[6] = (uint8_t) (dio2Mask & 0x00FF);
    loraTxBuf[7] = (uint8_t) ((dio3Mask >> 8) & 0x00FF);
    loraTxBuf[8] = (uint8_t) (dio3Mask & 0x00FF);
    sxSpiTransmit(radio, loraTxBuf, sizeof(loraTxBuf));
}

void ClrIrqStatus(sx1280_custom *radio, uint16_t irqMask) {
    uint8_t buf[3];
    buf[0] = 0X97;
    buf[1] = (uint8_t) (((uint16_t) irqMask >> 8) & 0x00FF);
    buf[2] = (uint8_t) ((uint16_t) irqMask & 0x00FF);
    sxSpiTransmit(radio, buf, sizeof(buf));
}

void SetTx(sx1280_custom *radio, uint8_t periodBase, uint16_t periodBaseCount) {
    uint8_t buf[4];
    buf[0] = 0X83;
    buf[1] = periodBase;
    buf[2] = (uint8_t) (((uint16_t) periodBaseCount >> 8) & 0x00FF);
    buf[3] = (uint8_t) ((uint16_t) periodBaseCount & 0x00FF);
    sxSpiTransmit(radio, buf, sizeof(buf));
}

void SetRx(sx1280_custom *radio, uint8_t periodBase, uint16_t periodBaseCount) {
    uint8_t buf[4];
    buf[0] = 0X82;
    buf[1] = periodBase;
    buf[2] = (uint8_t) (((uint16_t) periodBaseCount >> 8) & 0x00FF);
    buf[3] = (uint8_t) ((uint16_t) periodBaseCount & 0x00FF);
    sxSpiTransmit(radio, buf, sizeof(buf));
}

void GetPacketStatusLora(sx1280_custom *radio) {
    uint8_t loraRxBuf[4];
    uint8_t loraTxBuf[] = { 0x1D, 0x00, 0x00, 0x00};

    sxSpiTransmitReceive(radio, loraTxBuf, loraRxBuf, sizeof(loraTxBuf));

    radio->rssiSync = loraRxBuf[2];
    radio->rawSnr = loraRxBuf[3];

    radio->rssi = -((float) radio->rssiSync)/2;
    radio->snr = ((float) radio->rawSnr)/4;
}

void GetRxBufferStatus(sx1280_custom *radio) {

}

void WriteRegisterByte(sx1280_custom *radio, uint16_t address, uint8_t data) {
    uint8_t loraTxBuf[4];
    loraTxBuf[0] = 0x18;
    loraTxBuf[1] = (uint8_t) (((uint16_t) address >> 8) & 0x00FF);
    loraTxBuf[2] = (uint8_t) ((uint16_t) address & 0x00FF);
    loraTxBuf[3] = data;
    sxSpiTransmit(radio, loraTxBuf, sizeof(loraTxBuf));
}
