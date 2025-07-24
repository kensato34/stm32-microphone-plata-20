/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SDI_PIN          GPIO_PIN_12   // Serial Data Input
#define SCK_PIN          GPIO_PIN_10   // Serial Clock
#define LATCH_PIN        GPIO_PIN_11   // Latch
#define SHIFT_PORT       GPIOC         // Порт подключения


#define EEPROM_CS_GPIO_Port GPIOB
#define EEPROM_CS_Pin       GPIO_PIN_7

#define EEPROM_SIZE 8192

#define EEPROM_CMD_WREN     0x06
#define EEPROM_CMD_WRDI     0x04
#define EEPROM_CMD_RDSR     0x05
#define EEPROM_CMD_READ     0x03
#define EEPROM_CMD_WRITE    0x02
//#define EEPROM_CMD_ERASE    0x20
//#define EEPROM_CMD_CHIP_ERASE 0xC7


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint8_t pressed_key = 0;
volatile uint8_t uart_ready = 1;
uint16_t start_address = 0;
uint16_t test_length = 0;
char uart_buf[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void shift_register_init(void);
void shift_register_send_byte(uint8_t data);
void indicate_key_with_leds(char key);

void EEPROM_WriteByte(uint16_t address, uint8_t data);
uint8_t EEPROM_ReadByte(uint16_t address);
void EEPROM_Test(void);
void EEPROM_TestArray(uint16_t start_address, uint8_t test_length);
void EEPROM_EraseChip(void);
void EEPROM_WritePageData(uint16_t address, uint8_t* data, uint16_t length);
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Инициализация строк клавиатуры
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);


  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);


 //EEPROM_Test();
  sprintf(uart_buf, "Enter start address:\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
  HAL_UART_Receive(&huart1, (uint8_t*)uart_buf, sizeof(uart_buf), HAL_MAX_DELAY);
  sscanf(uart_buf, "%hx", &start_address);

  if (sscanf(uart_buf, "%hx", &start_address) != 1) {
      sprintf(uart_buf, "Error: invalid address.\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
      return;
  }

  if (start_address >= 0x2000) {
      sprintf(uart_buf, "Error: address too large.\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
      return;
  }



  // Запрос длины
  sprintf(uart_buf, "Enter test length:\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
 HAL_UART_Receive(&huart1, (uint8_t*)uart_buf, sizeof(uart_buf), HAL_MAX_DELAY);
  sscanf(uart_buf, "%hu", &test_length);



  // Запуск теста
  EEPROM_TestArray(start_address, test_length);

 // EEPROM_TestArray(0xFF34, 9999);




  HAL_UART_Transmit_IT(&huart1, (uint8_t*)"START\r\n", 7);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (pressed_key != 0) {
      indicate_key_with_leds(pressed_key);
      pressed_key = 0;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_0
                          |GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE0
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_0
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE5 PE6 PE7 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_ready = 1;
    }
}

//
void shift_register_send_8_bytes(uint8_t data[8]) {
    HAL_GPIO_WritePin(SHIFT_PORT, LATCH_PIN, GPIO_PIN_RESET);

    // 8 байт последовательно
    for (int byte_idx = 7; byte_idx >= 0; byte_idx--) {
        for (int bit_idx = 7; bit_idx >= 0; bit_idx--) {
            HAL_GPIO_WritePin(SHIFT_PORT, SDI_PIN,
                (data[byte_idx] & (1 << bit_idx)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SHIFT_PORT, SCK_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SHIFT_PORT, SCK_PIN, GPIO_PIN_RESET);
        }
    }

    HAL_GPIO_WritePin(SHIFT_PORT, LATCH_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SHIFT_PORT, LATCH_PIN, GPIO_PIN_RESET);
}





void indicate_key_with_leds(char key) {
    uint8_t led_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    switch(key) {

        case '1': led_data[1] |= 0b00001100; break;
        case '2': led_data[1] |= 0b00000011; break;
        case '3': led_data[2] |= 0b00001100; break;
        case '4': led_data[2] |= 0b00000011; break;
        case '5': led_data[3] |= 0b00001100; break;
        case '6': led_data[3] |= 0b00000011; break;
        case '7': led_data[4] |= 0b00001100; break;
        case '8': led_data[4] |= 0b00000011; break;
        case '9': led_data[5] |= 0b00001100; break;
        case '0': led_data[5] |= 0b00000011; break;

        case 'A': led_data[1] |= 0b00110000; break;
        case 'B': led_data[1] |= 0b11000000; break;
        case 'C': led_data[2] |= 0b00110000; break;
        case 'D': led_data[2] |= 0b11000000; break;
        case 'Z': led_data[3] |= 0b00110000; break;
        case 'O': led_data[3] |= 0b11000000; break;
        case 'V': led_data[4] |= 0b00110000; break;
        case '*': led_data[4] |= 0b11000000; break;
        case '#': led_data[5] |= 0b00110000; break;
        case '%': led_data[5] |= 0b11000000; break;




        default: break;
    }

    shift_register_send_8_bytes(led_data);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    currentMillis = HAL_GetTick();
    if (currentMillis - previousMillis < 50) return;
    previousMillis = currentMillis;

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);

    uint16_t col_pins[4] = {GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8};
    uint16_t row_pins[5] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};

    char keymap[5][4] = {
        {'1', '6', 'A', 'O'},
        {'2', '7', 'B', 'V'},
        {'3', '8', 'C', '*'},
        {'4', '9', 'D', '#'},
        {'5', '0', 'Z', '%'}
    };

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // Сканирование клавиатуры
    for (int row = 0; row < 5; row++) {

        for (int i = 0; i < 5; i++) {
            HAL_GPIO_WritePin(GPIOE, row_pins[i], GPIO_PIN_SET);
        }


        HAL_GPIO_WritePin(GPIOE, row_pins[row], GPIO_PIN_RESET);


        for (int col = 0; col < 4; col++) {
            if (HAL_GPIO_ReadPin(GPIOE, col_pins[col]) == GPIO_PIN_RESET) {
                pressed_key = keymap[row][col];

                char debug_message[50];
                sprintf(debug_message, "Key: '%c' (Row:%d, Col:%d)\r\n", pressed_key, row, col);
                HAL_UART_Transmit(&huart1, (uint8_t*)debug_message, strlen(debug_message), 100);

                goto cleanup;
            }
        }
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)"NONE\r\n", 6, 100);

cleanup:

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
}
// EEPROMOMOMOMOM


static void EEPROM_CS_LOW(void) {
    HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_RESET);
}

static void EEPROM_CS_HIGH(void) {
    HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_SET);
}

static void EEPROM_WaitReady(void) {
    uint8_t cmd = EEPROM_CMD_RDSR;
    uint8_t status;

    do {
        EEPROM_CS_LOW();
        HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
        HAL_SPI_Receive(&hspi1, &status, 1, 100);
        EEPROM_CS_HIGH();
        HAL_Delay(10);
    } while (status & 0x01);
}

void EEPROM_WriteByte(uint16_t address, uint8_t data) {
    uint8_t cmd_buf[4] = {
        EEPROM_CMD_WRITE,
        (address >> 8) & 0xFF,
        address & 0xFF,
        data
    };

    uint8_t wren_cmd = EEPROM_CMD_WREN;
    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &wren_cmd, 1, 100);
    EEPROM_CS_HIGH();

    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd_buf, 4, 100);
    EEPROM_CS_HIGH();

    EEPROM_WaitReady();
}


void EEPROM_WriteArrayPaged(uint16_t start_address, uint8_t* data, uint16_t length) {
    uint16_t PAGE_SIZE = 32;
    uint16_t bytes_written = 0;



    while (bytes_written < length) {
        uint16_t current_address = start_address + bytes_written;
        uint16_t page_offset = current_address % PAGE_SIZE;
        uint16_t bytes_to_write = PAGE_SIZE - page_offset;

        if (bytes_to_write > (length - bytes_written)) {
            bytes_to_write = length - bytes_written;
        }

        EEPROM_WriteArray(current_address, &data[bytes_written], bytes_to_write);
        bytes_written += bytes_to_write;
    }
}


void EEPROM_WriteArray(uint16_t address, uint8_t* data, uint16_t length) {

    uint8_t cmd_buf[3] = {
        EEPROM_CMD_WRITE,
        (address >> 8) & 0xFF,
        address & 0xFF
    };

    uint8_t wren_cmd = EEPROM_CMD_WREN;
    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &wren_cmd, 1, 100);
    EEPROM_CS_HIGH();



    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd_buf, 3, 100);
    HAL_SPI_Transmit(&hspi1, data, length, 100);
    EEPROM_CS_HIGH();


    EEPROM_WaitReady();
}


void EEPROM_ReadArray(uint16_t start_address, uint8_t* data, uint16_t length) {
    uint8_t cmd_buf[3] = {
        EEPROM_CMD_READ,
        (uint8_t)(start_address >> 8),
        (uint8_t)(start_address & 0xFF)
    };

    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd_buf, 3, 100);
    HAL_SPI_Receive(&hspi1, data, length, 1000);
    EEPROM_CS_HIGH();
}


void EEPROM_TestArray(uint16_t start_address , uint8_t test_length) {
    uint8_t* test_array = (uint8_t*)malloc(test_length);
    uint8_t* read_array = (uint8_t*)malloc(test_length);
    char msg[64];

    HAL_UART_Transmit(&huart1, (uint8_t*)"Start TestArray\r\n", 17, 100);

    for (int i = 0; i < test_length; i++) {
        test_array[i] = i & 0xFF;
    }

    EEPROM_WriteArrayPaged(start_address, test_array, test_length);

    EEPROM_ReadArray(start_address, read_array, test_length);

    uint8_t errors = 0;
    for (int i = 0; i < test_length; i++) {
        if (test_array[i] != read_array[i]) {
            errors++;
            sprintf(msg, "Error: wrote 0x%02X, read 0x%02X\r\n", test_array[i], read_array[i]);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        }
    }

    if (errors == 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"Array test is OK!\r\n", 20, 100);
    } else {
        sprintf(msg, "%d errors found\r\n", errors);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }

    free(test_array);
    free(read_array);
}





uint8_t EEPROM_ReadByte(uint16_t address) {
    uint8_t cmd_buf[3] = {
        EEPROM_CMD_READ,
        (address >> 8) & 0xFF,
        address & 0xFF
    };
    uint8_t data;

    EEPROM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, cmd_buf, 3, 100);
    HAL_SPI_Receive(&hspi1, &data, 1, 100);
    EEPROM_CS_HIGH();

    return data;
}





void EEPROM_Test(void) {
    EEPROM_CS_HIGH();
    HAL_Delay(10);

    uint16_t test_addr = 0x0010;
    uint8_t test_data = 0x5A;
    char msg[64];

    // Читаем что было до записи
    uint8_t before_data = EEPROM_ReadByte(test_addr);
    sprintf(msg, "Before write: 0x%02X\r\n", before_data);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

   EEPROM_WriteByte(test_addr, test_data);

    // Читаем что получилось после записи
    uint8_t after_data = EEPROM_ReadByte(test_addr);
    sprintf(msg, "After write: 0x%02X\r\n", after_data);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

    if (after_data == test_data) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"EEPROM OK\r\n", 11, 100);
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"EEPROM ERROR\r\n", 14, 100);
    }
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
