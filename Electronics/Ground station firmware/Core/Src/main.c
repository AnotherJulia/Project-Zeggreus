/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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

    char printBuffer[128];

    if (isTx) {
        //SetTxParams(0x06, 0xE0); // Power = 13 dBm (0x1F), Pout = -18 + power (dBm) ramptime = 20 us.
        SetTxParams(0x00, 0xE0); // lowest power -18dBm
        //SetTxParams(31, 0xE0); // Highest power. 12.5 dBm
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

            //changeLed(data[1], data[2], data[3]);

            WriteBuffer(0, data, sizeof(data));
            HAL_Delay(1);
            ClrIrqStatus(1); // clear txdone irq
            HAL_Delay(1);
            SetTx(0x02, 50); // time-out of 1ms * 50 = 50ms
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ledon);
            //SetRfFrequency2();
            //HAL_Delay(5000);
            //SetRfFrequency();
            //changeLed(0, 100, 0);
            sprintf(printBuffer, "Sending data...\r\n");
            CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
            HAL_Delay(500);
        }
    }
    else {
        // rx mode, reception
        SetDioIrqParams(1<<1, 1<<1, 0, 0); //rxdone on gpio1
        HAL_Delay(1);


        float data[5];
        data[0] = 60;
        data[1] = 60;
        data[2] = 60;
        uint8_t rxStartBufferPointer = 1;
        char printBuffer[128];

        uint8_t counter = 0;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,GPIO_PIN_SET);

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


            counter++;

            if (counter % 20 == 0) {

                sprintf(printBuffer, "Quaternion: %f, %f, %f, %f\r\n", data[0],
                        data[1], data[2], data[3]);
                //sprintf(printBuffer, "Quaternion: %f, %f, %f, %f\r\n",data[0],ori.orientationQuat.v[0],ori.orientationQuat.v[1],ori.orientationQuat.v[2]);
                CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));

                WriteBuffer(0, (uint8_t*) data, sizeof(data));
                HAL_Delay(1);
                ClrIrqStatus(1); // clear txdone irq
                HAL_Delay(1);
                SetTx(0x02, 50); // time-out of 1ms * 50 = 50ms
            }

//

            HAL_Delay(10);


        }

    }
}


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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //loraTesting(0);

  HAL_Delay(200);
  /* USER CODE END 2 */
  char printBuffer[128];
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  sprintf(printBuffer,"/*Project Zeggreus,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10*/");
	  CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.Channel = ADC_CHANNEL_10;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|RXEN_Pin|TXEN_Pin|LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SWANT_GPIO_Port, SWANT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin MODE_Pin RXEN_Pin TXEN_Pin
                           LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|MODE_Pin|RXEN_Pin|TXEN_Pin
                          |LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SWANT_Pin */
  GPIO_InitStruct.Pin = SWANT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SWANT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO3_Pin */
  GPIO_InitStruct.Pin = DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO2_Pin LORA_DIO1_Pin LORA_BUSY_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin|LORA_DIO1_Pin|LORA_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RESET_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
  while (1)
  {
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
