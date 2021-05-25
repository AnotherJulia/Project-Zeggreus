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
#include "sx1280_custom.h"
#include "telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct Quaternion {
    float w;       /**< Scalar part */
    float v[3];    /**< Vector part */
} Quaternion;

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t is_ant1 = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void switchAntenna() {
    is_ant1 = !is_ant1;

    HAL_GPIO_WritePin(SWANT_GPIO_Port, SWANT_Pin, is_ant1);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, is_ant1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void loraTesting(uint8_t isTx) {

    sx1280_custom radio;
    sxInit(&radio, &hspi3, LORA_NSS_GPIO_Port, LORA_NSS_Pin);

    if (isTx) {
        //SetTxParams(0x06, 0xE0); // Power = 13 dBm (0x1F), Pout = -18 + power (dBm) ramptime = 20 us.
        //SetTxParams(0x00, 0xE0); // lowest power -18dBm
        SetTxParams(&radio, 31, 0xE0); // Highest power. 12.5 dBm
        HAL_Delay(3);

        uint8_t data[] = { 0, 0, 0, 0 };

        WriteBuffer(&radio, 0, data, sizeof(data));
        HAL_Delay(1);

        SetDioIrqParams(&radio, 1, 1, 0, 0); // txdone on gpio1

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

            WriteBuffer(&radio, 0, data, sizeof(data));
            HAL_Delay(1);
            ClrIrqStatus(&radio, 1); // clear txdone irq
            HAL_Delay(1);
            SetTx(&radio, 0x02, 50); // time-out of 1ms * 50 = 50ms
            //SetRfFrequency2();
            //HAL_Delay(5000);
            //SetRfFrequency();
            //changeLed(0, 100, 0);
            HAL_Delay(500);
        }
    } else {
        // rx mode
        SetDioIrqParams(&radio, 1 << 1, 1 << 1, 0, 0); //rxdone on gpio1
        HAL_Delay(1);

        uint8_t data[32];
        data[0] = 60;
        data[1] = 60;
        data[2] = 60;
        uint8_t rxStartBufferPointer = 1;

        uint8_t counter = 0;

        while (1) {

            //SetRx(0x00, 0xffff); // continous rx
            SetRx(&radio, 0x00, 0); // No timeout
            //SetRx(0x02, 200); // 200 ms timeout
            HAL_Delay(1);
            // wait for reception:
            while (!HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin)) {
            }

            //GetPacketStatus(); // TODO
            ClrIrqStatus(&radio, 1 << 1); // clear rxdone Irq
            HAL_Delay(1);
            //GetRxBufferStatus(); // TODO
            ReadBuffer(&radio, rxStartBufferPointer, 32, data);
            changeLed(data[1], data[2], data[3]);
            if (data[2]) {
                //htim2.Instance->CCR4 = 1000;
            } else {
                //htim2.Instance->CCR4 = 2000;
            }

            HAL_Delay(10);

        }

    }
}

void loraOrientation(uint8_t isTx) {

    sx1280_custom radio;

    sxInit(&radio, &hspi3, LORA_NSS_GPIO_Port, LORA_NSS_Pin);
    sxSetDio1Pin(&radio, LORA_DIO1_GPIO_Port, LORA_DIO1_Pin);

    float data[4];

    char printBuffer[128];

    if (isTx) {
    } else {
        // rx mode
        SetDioIrqParams(&radio, 1 << 1, 1 << 1, 0, 0); //rxdone on gpio1
        HAL_Delay(1);

        uint8_t rxStartBufferPointer = 1;

        //changeLed(0, 100, 0);
        while (1) {

            //SetRx(0x00, 0xffff); // continous rx
            SetRx(&radio, 0x00, 0); // No timeout
            //SetRx(0x02, 200); // 200 ms timeout
            HAL_Delay(1);
            // wait for reception:
            while (!HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin)) {
            }

            GetPacketStatusLora(&radio);
            ClrIrqStatus(&radio, 1 << 1); // clear rxdone Irq
            HAL_Delay(1);
            //GetRxBufferStatus(); // TODO

            ReadBuffer(&radio, rxStartBufferPointer, sizeof(data),
                    (uint8_t*) data);
            snprintf(printBuffer, 128,
                    "Quaternion: %f, %f, %f, %f, RSSI: %f, SNR: %f\r\n",
                    data[0], data[1], data[2], data[3], radio.rssi, radio.snr);
            CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
            HAL_Delay(1);

        }

    }
}

void loraTelemetry() {
    TLM_decoded TLM_dec;
    TLM_encoded TLM_enc;

    sx1280_custom radio;

    sxInit(&radio, &hspi3, LORA_NSS_GPIO_Port, LORA_NSS_Pin);
    sxSetDio1Pin(&radio, LORA_DIO1_GPIO_Port, LORA_DIO1_Pin);

    char printBuffer[256];

    // rx mode
    SetDioIrqParams(&radio, (1 << 1) | (1 << 6), 1 << 1, 0, 0); //rxdone on gpio1, crcerror on as well
    HAL_Delay(1);

    uint8_t rxStartBufferPointer = 0;
    /*
    TLM_dec.packet_type = 1;
    TLM_dec.flight_state = 23;
    TLM_dec.is_playing_music = 0;
    TLM_dec.is_data_logging = 0;
    TLM_dec.pin_states = 0b00011011;
    TLM_dec.servo_state = 3;
    TLM_dec.vbat = 7.283;
    TLM_dec.systick = 1232432;
    TLM_dec.orientation_quat[0] = 0.143123;
    TLM_dec.acc[2] = -12343;
    TLM_dec.gyro[2] = -21;
    TLM_dec.baro = 90001.623;
    TLM_dec.temp = 63.4;
    TLM_dec.vertical_velocity = 180;
    TLM_dec.altitude = 1321;
    TLM_dec.debug = 1337;
    TLM_dec.ranging = 15212;

     */

    float latitude = 52.394821;
    float longitude = 5.922696;

    float acc_conversion = 0.0095712904;
    float gyro_conversion = 0.00122173047; //0.070;

    uint32_t pkt_count = 0;

    //changeLed(0, 100, 0);
    uint8_t data[4];
    uint32_t lasttime = HAL_GetTick();
    uint32_t nowtime = HAL_GetTick();
    uint32_t delay = 0;

    uint8_t is_soft_enabled;
    uint8_t is_armed ;
    uint8_t is_breakwire_connected;
    uint8_t is_camera_on;

    uint8_t button_pressed = 0;
    while (1) {

        //SetRx(0x00, 0xffff); // continous rx
        SetRx(&radio, 0x00, 0); // No timeout
        //SetRx(0x02, 200); // 200 ms timeout
        HAL_Delay(1);
        // wait for reception:
        for (int i = 0; i < 35; i++) { // 35 ms timeout
            if (HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin)) {
                nowtime = HAL_GetTick();
                delay = nowtime - lasttime  ;
                lasttime = nowtime;
                break;
            }
            HAL_Delay(1);
        }

        if (HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin)) {

            pkt_count++;

            GetPacketStatusLora(&radio);
            GetIrqStatus(&radio);

            ClrIrqStatus(&radio, (1 << 1) | (1 << 6)); // clear rxdone Irq and crcerror
            HAL_Delay(1);
            //GetRxBufferStatus(); // TODO

            ReadBuffer(&radio, rxStartBufferPointer, sizeof(TLM_enc),
                    (uint8_t*) &TLM_enc);
            //ReadBuffer(&radio, rxStartBufferPointer, sizeof(data), (uint8_t*) data);
            decode_TLM(&TLM_enc, &TLM_dec);
            //snprintf(printBuffer, 128, "%d,%d,%d,%d,%d,%d,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\r\n", TLM_dec.packet_type,TLM_dec.flight_state,TLM_dec.is_playing_music,TLM_dec.is_data_logging,
            //        TLM_dec.pin_states,TLM_dec.servo_state, TLM_dec.vbat, TLM_dec.systick, TLM_dec.orientation_quat[0], TLM_dec.acc[2],TLM_dec.gyro[2],TLM_dec.baro, TLM_dec.temp, TLM_dec.vertical_velocity,
            //        TLM_dec.altitude, TLM_dec.ranging);

            is_soft_enabled = (TLM_dec.pin_states & 1);
            is_armed = (TLM_dec.pin_states & (1 << 1)) >> 1;
            is_breakwire_connected = (TLM_dec.pin_states & (1 << 2)) >> 2;
            is_camera_on = (TLM_dec.pin_states & (1 << 3)) >> 3;

            snprintf(printBuffer, 256,
                    "/*Project Zeggreus,%ld,%ld,%f,%f,%f,%f,%f,%ld,%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d*/\r\n",
                    TLM_dec.systick, pkt_count, TLM_dec.vbat, TLM_dec.temp,
                    TLM_dec.altitude, TLM_dec.baro / 1000, TLM_dec.temp, delay,
                    TLM_dec.systick, longitude, latitude, TLM_dec.altitude, 0.0,
                    TLM_dec.acc[0] * acc_conversion,
                    TLM_dec.acc[1] * acc_conversion,
                    TLM_dec.acc[2] * acc_conversion,
                    TLM_dec.gyro[0] * gyro_conversion,
                    TLM_dec.gyro[1] * gyro_conversion,
                    TLM_dec.gyro[2] * gyro_conversion, radio.rssi,
                    radio.crcError, is_ant1 ? 1 : 2, is_soft_enabled, is_armed,
                    is_breakwire_connected, is_camera_on,TLM_dec.flight_state);

            //snprintf(printBuffer, 128, "Quaternion:%f, %f, %f, %f\r\n", TLM_dec.orientation_quat[0], TLM_dec.orientation_quat[1], TLM_dec.orientation_quat[2], TLM_dec.orientation_quat[3]);
            //snprintf(printBuffer, 128,
            //       "Quaternion: %d, %d, %d, %d, RSSI: %f, SNR: %f\r\n",
            //       data[0], data[1], data[2], data[3], radio.rssi, radio.snr);
            CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

        } else {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
            // try other antenna
            switchAntenna();
        }

        if (!HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)) {
            if (!button_pressed) {
                switchAntenna();
                button_pressed = 1;
            }
        }
        else {
            button_pressed = 0;
        }

        HAL_Delay(1);

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
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  loraTelemetry(0);

  HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //sprintf(printBuffer,"/*Project Zeggreus,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10*/");
	  //CDC_Transmit_FS((uint8_t*) printBuffer, strlen(printBuffer));
	  //HAL_Delay(500);
	  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);


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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|RXEN_Pin|TXEN_Pin
                          |LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SWANT_GPIO_Port, SWANT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin MODE_Pin RXEN_Pin
                           TXEN_Pin LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|MODE_Pin|RXEN_Pin
                          |TXEN_Pin|LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_Pin */
  GPIO_InitStruct.Pin = DEBUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DEBUG_GPIO_Port, &GPIO_InitStruct);

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
