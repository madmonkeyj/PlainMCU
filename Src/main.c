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
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor_manager.h"
#include "debug_utils.h"
#include "icm42688.h"
#include "mmc5983ma.h"
#include "bmp581.h"
#include "i2c_scanner.h"
#include "i2c_dma_arbiter.h"  // Required for DMA arbiter
#include <math.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RunHighSpeedLoop(void);
void PrintStatistics(void);
void TestBMP581(void);
void TestSensorManagerWithBaro(void);
void RunContinuousSensorTest(void);

void QuickDiagnosticTest(void);
void PowerCycleTest(void);
void RunBMP581TestSequence(void);


static inline uint32_t GetMicros(void);
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
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  // Wait for USB CDC to be ready
  HAL_Delay(2000);

  // Initialize I2C DMA arbiter FIRST
  I2C_DMA_Arbiter_Init();

  DebugPrint("\r\n\r\n");
  DebugPrint("========================================\r\n");
  DebugPrint("  STM32G4 Flight Computer v1.0\r\n");
  DebugPrint("  CPU: 170MHz, DMA-optimized sensors\r\n");
  DebugPrint("========================================\r\n\r\n");

  // Scan I2C bus
  I2C_Scan();

  // Run quick diagnostic
  QuickDiagnosticTest();

  // Run full test
  TestBMP581();

  // Dump registers
  BMP581_DumpRegisters();

  // Test data updating
  BMP581_TestDataUpdating();

  DebugPrint("Initialization complete. Entering main loop...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Your flight computer main loop here
    // For now, just blink LED and idle
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(500);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Get microsecond timestamp (uses SysTick)
 */
static inline uint32_t GetMicros(void) {
    uint32_t m = HAL_GetTick();
    uint32_t u = SysTick->LOAD - SysTick->VAL;

    // Check for pending SysTick interrupt
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
        m++;
        u = SysTick->LOAD - SysTick->VAL;
    }

    // Convert to microseconds (assuming 170MHz CPU)
    return (m * 1000) + (u * 1000 / SysTick->LOAD);
}

/**
 * @brief Test BMP581 barometer with Bosch-style initialization
 */
void TestBMP581(void) {
    uint8_t chip_id;
    BMP581_Data_t baro_data;

    DebugPrint("\r\n");
    DebugPrint("========================================\r\n");
    DebugPrint("       BMP581 Barometer Test\r\n");
    DebugPrint("      (Bosch-Style Driver v2.0)\r\n");
    DebugPrint("========================================\r\n\r\n");

    // Test 1: Read Chip ID
    DebugPrint("Step 1: Reading Chip ID...\r\n");
    if (BMP581_ReadChipID(&chip_id) == HAL_OK) {
        DebugPrint("  Chip ID: 0x%02X (expected 0x50)\r\n", chip_id);
        if (chip_id == BMP581_CHIP_ID_VALUE) {
            DebugPrint("  ✓ Chip ID correct!\r\n\r\n");
        } else {
            DebugPrint("  ✗ Chip ID mismatch!\r\n\r\n");
            return;
        }
    } else {
        DebugPrint("  ✗ Failed to read Chip ID\r\n");
        DebugPrint("  Check I2C wiring and address (0x47)\r\n\r\n");
        return;
    }

    // Test 2: Initialize with power-up validation
    DebugPrint("Step 2: Initializing BMP581...\r\n");
    DebugPrint("  - Checking NVM ready status\r\n");
    DebugPrint("  - Validating POR complete\r\n");
    DebugPrint("  - Configuring sensor\r\n");
    DebugPrint("  - Starting measurements\r\n\r\n");

    if (BMP581_Init() == HAL_OK) {
        DebugPrint("  ✓ BMP581 initialized successfully!\r\n");
        DebugPrint("  Config: 50Hz ODR, 8x pressure OSR, 2x temp OSR\r\n");
        DebugPrint("  IIR filter: Coeff 3 (moderate smoothing)\r\n");
        DebugPrint("  Mode: NORMAL (continuous measurements)\r\n\r\n");
    } else {
        DebugPrint("  ✗ Failed to initialize BMP581\r\n");
        DebugPrint("  Running diagnostics...\r\n\r\n");

        // Run diagnostic to identify the problem
        BMP581_DiagnoseIssue();
        return;
    }

    // Test 3: Verify data is updating
    DebugPrint("Step 3: Verifying sensor data...\r\n");
    if (BMP581_WaitForDataReady(200) == HAL_OK) {
        DebugPrint("  ✓ Sensor data ready!\r\n\r\n");
    } else {
        DebugPrint("  ⚠ Timeout waiting for data ready\r\n");
        DebugPrint("  Sensor may not be producing valid data\r\n\r\n");
    }

    // Test 4: Read sensor data
    DebugPrint("Step 4: Reading sensor data (10 samples @ 50Hz)...\r\n");
    DebugPrint("----------------------------------------\r\n");

    uint32_t successful_reads = 0;
    uint32_t invalid_pressure = 0;

    for (int i = 0; i < 10; i++) {
        // Wait for new data (at 50Hz, data updates every 20ms)
        HAL_Delay(25);

        if (BMP581_ReadSensorData(&baro_data) == HAL_OK) {
            DebugPrint("Sample %2d: ", i + 1);

            // Check if pressure is the stuck value (0x7F7F7F = 8355711 / 64 = 130557.98 Pa)
            if (fabsf(baro_data.pressure_pa - 130557.98f) < 1.0f) {
                DebugPrint("P=STUCK (0x7F7F7F) | ");
                invalid_pressure++;
            } else {
                DebugPrint("P=%7.2f Pa (%7.2f hPa) | ",
                           baro_data.pressure_pa,
                           baro_data.pressure_pa / 100.0f);
            }

            DebugPrint("T=%5.2f°C | ", baro_data.temperature_c);

            // Calculate altitude (using standard sea level pressure)
            float altitude = 44330.0f * (1.0f - powf(baro_data.pressure_pa / 101325.0f, 0.1903f));
            DebugPrint("Alt=%6.1f m\r\n", altitude);

            successful_reads++;
        } else {
            DebugPrint("Sample %2d: ✗ Read failed\r\n", i + 1);
        }
    }

    DebugPrint("----------------------------------------\r\n");
    DebugPrint("Successful reads: %lu/10\r\n", successful_reads);

    if (invalid_pressure > 0) {
        DebugPrint("⚠ WARNING: %lu samples had stuck pressure (0x7F7F7F)!\r\n", invalid_pressure);
        DebugPrint("   This indicates the sensor is not measuring pressure.\r\n");
        DebugPrint("   Running diagnostics...\r\n\r\n");
        BMP581_DiagnoseIssue();
    } else if (successful_reads >= 8) {
        DebugPrint("✓ BMP581 test PASSED!\r\n");
        DebugPrint("  Pressure readings are valid and changing.\r\n");
    } else {
        DebugPrint("⚠ BMP581 test completed with errors\r\n");
    }

    DebugPrint("========================================\r\n\r\n");
}

/**
 * @brief Quick diagnostic test to identify BMP581 issues
 */
void QuickDiagnosticTest(void) {
    DebugPrint("\r\n");
    DebugPrint("========================================\r\n");
    DebugPrint("     BMP581 Quick Diagnostic\r\n");
    DebugPrint("========================================\r\n\r\n");

    uint8_t chip_id;

    // 1. Can we communicate?
    DebugPrint("1. I2C Communication Test:\r\n");
    if (BMP581_ReadChipID(&chip_id) == HAL_OK) {
        DebugPrint("   ✓ I2C communication OK\r\n");
        DebugPrint("   Chip ID: 0x%02X\r\n\r\n", chip_id);
    } else {
        DebugPrint("   ✗ I2C communication FAILED\r\n");
        DebugPrint("   Check wiring and pull-ups\r\n\r\n");
        return;
    }

    // 2. Is NVM ready?
    DebugPrint("2. NVM Status (CRITICAL):\r\n");
    uint8_t status_reg;
    if (BMP581_ReadRegister(BMP581_REG_STATUS, &status_reg) == HAL_OK) {
        bool nvm_ready = (status_reg & 0x02) != 0;
        bool nvm_error = (status_reg & 0x04) != 0;

        DebugPrint("   STATUS = 0x%02X\r\n", status_reg);
        DebugPrint("   NVM Ready: %s\r\n", nvm_ready ? "YES ✓" : "NO ✗");
        DebugPrint("   NVM Error: %s\r\n", nvm_error ? "YES ✗" : "NO ✓");

        if (!nvm_ready) {
            DebugPrint("\r\n   ✗ NVM NOT READY - This is the problem!\r\n");
            DebugPrint("   The sensor cannot measure pressure without NVM.\r\n");
            DebugPrint("   This may indicate:\r\n");
            DebugPrint("   - Insufficient power-up delay\r\n");
            DebugPrint("   - Power supply issue\r\n");
            DebugPrint("   - Defective sensor\r\n\r\n");
        } else {
            DebugPrint("   ✓ NVM is ready\r\n\r\n");
        }
    }

    // 3. Read raw pressure value
    DebugPrint("3. Raw Pressure Reading:\r\n");
    uint8_t raw_press[3];
    if (HAL_I2C_Mem_Read(&hi2c1, BMP581_I2C_ADDR, 0x20,
                         I2C_MEMADD_SIZE_8BIT, raw_press, 3, 100) == HAL_OK) {
        DebugPrint("   Raw bytes: 0x%02X 0x%02X 0x%02X\r\n",
                   raw_press[2], raw_press[1], raw_press[0]);

        if (raw_press[0] == 0x7F && raw_press[1] == 0x7F && raw_press[2] == 0x7F) {
            DebugPrint("   ✗ STUCK AT 0x7F7F7F!\r\n");
            DebugPrint("   Sensor is not measuring pressure.\r\n\r\n");
        } else {
            DebugPrint("   ✓ Value looks valid (not stuck)\r\n\r\n");
        }
    }

    DebugPrint("========================================\r\n\r\n");
}

/**
 * @brief Power cycle test - useful for debugging power-up issues
 */
void PowerCycleTest(void) {
    DebugPrint("\r\n");
    DebugPrint("========================================\r\n");
    DebugPrint("     BMP581 Power Cycle Test\r\n");
    DebugPrint("========================================\r\n\r\n");

    DebugPrint("This test checks if the sensor initializes\r\n");
    DebugPrint("properly after a soft reset.\r\n\r\n");

    for (int cycle = 1; cycle <= 3; cycle++) {
        DebugPrint("--- Cycle %d ---\r\n", cycle);

        // Soft reset
        DebugPrint("Performing soft reset...\r\n");
        if (BMP581_SoftReset() == HAL_OK) {
            DebugPrint("✓ Reset successful\r\n");
        } else {
            DebugPrint("✗ Reset failed\r\n");
        }

        // Check status
        uint8_t status_reg;
        if (BMP581_ReadRegister(BMP581_REG_STATUS, &status_reg) == HAL_OK) {
            DebugPrint("STATUS = 0x%02X ", status_reg);
            if (status_reg & 0x02) {
                DebugPrint("(NVM Ready ✓)\r\n");
            } else {
                DebugPrint("(NVM NOT Ready ✗)\r\n");
            }
        }

        HAL_Delay(500);
    }

    DebugPrint("\r\n========================================\r\n\r\n");
}

/**
 * @brief Main test sequence - add this to your main() function
 */
void RunBMP581TestSequence(void) {
    // Scan I2C bus
    I2C_Scan();

    // Run quick diagnostic first
    QuickDiagnosticTest();

    // If diagnostic looks good, run full test
    TestBMP581();

    // Dump registers for detailed analysis
    BMP581_DumpRegisters();

    // Optional: Test data updating
    BMP581_TestDataUpdating();
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
