/**
  ******************************************************************************
  * @file    bmp581.c
  * @brief   BMP581 barometer - Based on Bosch BMP5 reference implementation
  ******************************************************************************
  */

#include "bmp581.h"
#include "i2c_dma_arbiter.h"
#include <math.h>

/* Private variables */
static volatile uint32_t baro_dma_callback_count = 0;
static uint8_t __attribute__((aligned(4))) baro_rx_buffer[8];
static volatile bool baro_dma_busy = false;

/* Private function prototypes */
static HAL_StatusTypeDef BMP581_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef BMP581_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);
static HAL_StatusTypeDef BMP581_PowerUpCheck(void);
static HAL_StatusTypeDef BMP581_SetPowerModeInternal(BMP581_PowerMode_t powermode);
static float BMP581_ConvertPressure(int32_t raw_press);
static float BMP581_ConvertTemperature(int32_t raw_temp);
static float BMP581_CalculateAltitude(float pressure_pa, float sea_level_pa);

/* Private functions */
static HAL_StatusTypeDef BMP581_WriteRegister(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, BMP581_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, BMP581_I2C_TIMEOUT);
}

HAL_StatusTypeDef BMP581_ReadRegister(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, BMP581_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, BMP581_I2C_TIMEOUT);
}

static HAL_StatusTypeDef BMP581_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
    HAL_StatusTypeDef status;
    uint32_t wait_start;

    /* Wait if our previous DMA is still busy */
    wait_start = HAL_GetTick();
    while (baro_dma_busy) {
        if (HAL_GetTick() - wait_start > BMP581_I2C_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

    /* Mark our device as busy */
    baro_dma_busy = true;

    /* Request DMA transfer through arbiter */
    status = I2C_DMA_Arbiter_RequestTransfer(
        &hi2c1,
        I2C_DMA_DEVICE_BARO,
        BMP581_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        baro_rx_buffer,
        len,
        BMP581_DMA_Complete_Callback
    );

    if (status != HAL_OK) {
        baro_dma_busy = false;
        return status;
    }

    /* Wait for completion */
    uint32_t timeout = HAL_GetTick() + BMP581_I2C_TIMEOUT;
    while (baro_dma_busy && HAL_GetTick() < timeout) {
        __NOP();
    }

    if (baro_dma_busy) {
        HAL_I2C_Master_Abort_IT(&hi2c1, BMP581_I2C_ADDR);
        baro_dma_busy = false;
        return HAL_TIMEOUT;
    }

    /* Copy from DMA buffer */
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = baro_rx_buffer[i];
    }

    return HAL_OK;
}

/**
 * @brief Power-up validation check (based on Bosch post-power-up procedure)
 * This checks:
 * 1. NVM ready status = 1 and NVM error status = 0 (from STATUS register)
 * 2. POR/soft-reset complete status = 1 (from INT_STATUS register)
 */
static HAL_StatusTypeDef BMP581_PowerUpCheck(void) {
    HAL_StatusTypeDef status;
    uint8_t status_reg;
    uint8_t int_status_reg;
    uint32_t timeout = HAL_GetTick() + 100; // 100ms timeout

    while (HAL_GetTick() < timeout) {
        status = BMP581_ReadRegister(BMP581_REG_STATUS, &status_reg);
        if (status != HAL_OK) return status;

        // Check if nvm_rdy (bit 1) is set and nvm_err (bit 2) is clear
        if ((status_reg & BMP581_STATUS_NVM_RDY) && !(status_reg & BMP581_STATUS_NVM_ERR)) {
            // NVM is OK, now check for POR complete
            status = BMP581_ReadRegister(BMP581_REG_INT_STATUS, &int_status_reg);
            if (status != HAL_OK) return status;

            // Check if por_complete (bit 4) is set
            if (int_status_reg & BMP581_INT_STATUS_POR) {
                return HAL_OK; // Both checks passed
            }
        }
        HAL_Delay(5); // Wait before retrying
    }

    return HAL_ERROR; // Timeout reached, check failed
}

/**
 * @brief Internal power mode setting
 */
static HAL_StatusTypeDef BMP581_SetPowerModeInternal(BMP581_PowerMode_t powermode) {
    HAL_StatusTypeDef status;
    uint8_t reg_data;

    status = BMP581_ReadRegister(BMP581_REG_ODR_CONFIG, &reg_data);
    if (status != HAL_OK) {
        return status;
    }

    // --- FIX: Correctly mask and set the power mode bits [1:0] ---
    reg_data = (reg_data & ~BMP581_MODE_MASK) | (powermode & BMP581_MODE_MASK);

    return BMP581_WriteRegister(BMP581_REG_ODR_CONFIG, reg_data);
}

static float BMP581_ConvertPressure(int32_t raw_press) {
    return (float)raw_press * BMP581_PRESS_SCALE;
}

static float BMP581_ConvertTemperature(int32_t raw_temp) {
    return (float)raw_temp * BMP581_TEMP_SCALE;
}

static float BMP581_CalculateAltitude(float pressure_pa, float sea_level_pa) {
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

/* Public functions */
void BMP581_DMA_Complete_Callback(void) {
    baro_dma_busy = false;
    baro_dma_callback_count++;
}

uint32_t BMP581_GetDMACallbackCount(void) {
    return baro_dma_callback_count;
}

/**
 * @brief Initialize BMP581 sensor with a more robust startup sequence
 */
HAL_StatusTypeDef BMP581_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t chip_id;
    uint8_t reg_data;

    // --- FIX: Implement a robust startup sequence ---

    // 1. Wait for the sensor to be physically ready on the I2C bus.
    // This is more reliable than a fixed delay. We'll wait up to 100ms.
    status = HAL_I2C_IsDeviceReady(&hi2c1, BMP581_I2C_ADDR, 10, 100);
    if (status != HAL_OK) {
        return status; // Sensor is not even acknowledging its address
    }

    // 2. Perform a dummy read of the Chip ID register.
    // Some Bosch sensors require this to fully enable the I2C interface.
    (void)BMP581_ReadChipID(&chip_id);
    HAL_Delay(2); // Small delay after first communication

    // 3. Now that communication is established, verify the Chip ID properly.
    status = BMP581_ReadChipID(&chip_id);
    if (status != HAL_OK || chip_id != BMP581_CHIP_ID_VALUE) {
        return HAL_ERROR;
    }

    // 4. Perform a soft reset to put the sensor into a known state.
    status = BMP581_WriteRegister(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    if (status != HAL_OK) {
        return status;
    }
    // Wait the required time for the soft reset to complete (t_soft_res = 2ms).
    HAL_Delay(5); // Use 5ms for a safe margin

    // 5. After reset, the sensor needs time to load NVM. We must now perform the critical power-up check.
    // This function waits for NVM_READY and POR_COMPLETE flags.
    status = BMP581_PowerUpCheck();
    if (status != HAL_OK) {
        return HAL_ERROR; // This is where it was failing before.
    }

    // --- Configuration can now proceed ---

    // 6. Configure OSR: 8x pressure, 2x temperature, and enable pressure measurement.
    reg_data = (BMP581_OSR_T_2X << BMP581_OSR_T_SHIFT) | BMP581_OSR_P_8X | (1 << BMP581_PRESS_EN_POS);
    status = BMP581_WriteRegister(BMP581_REG_OSR_CONFIG, reg_data);
    if (status != HAL_OK) return status;

    // 7. Configure IIR filter: coefficient 3.
    uint8_t iir_config = BMP581_IIR_COEFF_3 | (BMP581_IIR_COEFF_3 << 4);
    status = BMP581_WriteRegister(BMP581_REG_DSP_IIR, iir_config);
    if (status != HAL_OK) return status;

    // --- FINAL FIX: Enable the Data Ready interrupt source ---
    // This allows the DRDY bit in the INT_STATUS register to be set.
    status = BMP581_WriteRegister(BMP581_REG_INT_SOURCE, 0x01);
    if (status != HAL_OK) return status;

    // 8. Set ODR to 50 Hz and switch to NORMAL mode for continuous measurements.
    uint8_t odr_config = (BMP581_DEEP_DISABLED << BMP581_DEEP_DISABLE_POS) | (BMP581_ODR_50_HZ << 2) | BMP581_MODE_NORMAL;
    status = BMP581_WriteRegister(BMP581_REG_ODR_CONFIG, odr_config);
    if (status != HAL_OK) return status;

    // 9. Wait for the first measurement to be ready.
    return BMP581_WaitForDataReady(100);
}

HAL_StatusTypeDef BMP581_ReadChipID(uint8_t *chip_id) {
    return BMP581_ReadRegister(BMP581_REG_CHIP_ID, chip_id);
}

HAL_StatusTypeDef BMP581_ReadSensorData(BMP581_Data_t *data) {
    HAL_StatusTypeDef status;
    uint8_t raw_data[6];

    if (data == NULL) {
        return HAL_ERROR;
    }

    /* Read all 6 bytes: TEMP_XLSB through PRESS_MSB */
    status = BMP581_ReadRegisters(BMP581_REG_TEMP_DATA_XLSB, raw_data, 6);

    if (status == HAL_OK) {
        /* Combine 24-bit temperature */
        data->temperature_raw = (int32_t)(((uint32_t)raw_data[2] << 16) | ((uint32_t)raw_data[1] << 8) | raw_data[0]);
        /* Sign extend from 24-bit to 32-bit */
        if (data->temperature_raw & 0x00800000) {
            data->temperature_raw |= 0xFF000000;
        }

        /* Combine 24-bit pressure */
        data->pressure_raw = (int32_t)(((uint32_t)raw_data[5] << 16) | ((uint32_t)raw_data[4] << 8) | raw_data[3]);
        /* Sign extend from 24-bit to 32-bit */
        if (data->pressure_raw & 0x00800000) {
            data->pressure_raw |= 0xFF000000;
        }

        /* Convert to engineering units */
        data->temperature_c = BMP581_ConvertTemperature(data->temperature_raw);
        data->pressure_pa = BMP581_ConvertPressure(data->pressure_raw);
    }

    return status;
}

HAL_StatusTypeDef BMP581_GetPressure(float *pressure_pa) {
    BMP581_Data_t data;
    if (pressure_pa == NULL) return HAL_ERROR;
    HAL_StatusTypeDef status = BMP581_ReadSensorData(&data);
    if (status == HAL_OK) *pressure_pa = data.pressure_pa;
    return status;
}

HAL_StatusTypeDef BMP581_GetTemperature(float *temperature_c) {
    BMP581_Data_t data;
    if (temperature_c == NULL) return HAL_ERROR;
    HAL_StatusTypeDef status = BMP581_ReadSensorData(&data);
    if (status == HAL_OK) *temperature_c = data.temperature_c;
    return status;
}

HAL_StatusTypeDef BMP581_GetAltitude(float *altitude_m, float sea_level_pa) {
    float pressure_pa;
    if (altitude_m == NULL) return HAL_ERROR;
    HAL_StatusTypeDef status = BMP581_GetPressure(&pressure_pa);
    if (status == HAL_OK) *altitude_m = BMP581_CalculateAltitude(pressure_pa, sea_level_pa);
    return status;
}

HAL_StatusTypeDef BMP581_CheckDataReady(bool *ready) {
    HAL_StatusTypeDef status;
    uint8_t int_status_reg;
    if (ready == NULL) {
        return HAL_ERROR;
    }

    // --- FIX: Check the correct register (INT_STATUS) and bit (DRDY) as per datasheet ---
    status = BMP581_ReadRegister(BMP581_REG_INT_STATUS, &int_status_reg);
    if (status == HAL_OK) {
        *ready = (int_status_reg & BMP581_INT_STATUS_DRDY);
    } else {
        *ready = false;
    }

    return status;
}

HAL_StatusTypeDef BMP581_WaitForDataReady(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    bool ready = false;
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (BMP581_CheckDataReady(&ready) == HAL_OK && ready) {
            return HAL_OK;
        }
        HAL_Delay(5);
    }
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef BMP581_GetPowerMode(BMP581_PowerMode_t *powermode) {
    HAL_StatusTypeDef status;
    uint8_t reg_data;
    if (powermode == NULL) {
        return HAL_ERROR;
    }

    status = BMP581_ReadRegister(BMP581_REG_ODR_CONFIG, &reg_data);
    if (status != HAL_OK) {
        return status;
    }

    // --- FIX: Correctly get power mode from bits [1:0] and check deep standby flag ---
    uint8_t pwrmode = reg_data & BMP581_MODE_MASK;
    uint8_t deep_dis = (reg_data >> BMP581_DEEP_DISABLE_POS) & 0x01;

    if (pwrmode == BMP581_POWERMODE_STANDBY && deep_dis == BMP581_DEEP_ENABLED) {
        *powermode = BMP581_POWERMODE_DEEP_STANDBY;
    } else {
        *powermode = (BMP581_PowerMode_t)pwrmode;
    }
    return HAL_OK;
}

HAL_StatusTypeDef BMP581_SetPowerMode(BMP581_PowerMode_t powermode) {
    HAL_StatusTypeDef status;
    BMP581_PowerMode_t current_mode;

    status = BMP581_GetPowerMode(&current_mode);
    if (status != HAL_OK) return status;

    if (current_mode != BMP581_POWERMODE_STANDBY) {
        status = BMP581_SetPowerModeInternal(BMP581_POWERMODE_STANDBY);
        if (status != HAL_OK) return status;
        HAL_Delay(BMP581_DELAY_US_STANDBY / 1000);
    }

    if (powermode != BMP581_POWERMODE_STANDBY) {
        status = BMP581_SetPowerModeInternal(powermode);
    }
    return status;
}

HAL_StatusTypeDef BMP581_SoftReset(void) {
    HAL_StatusTypeDef status = BMP581_WriteRegister(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    if (status == HAL_OK) {
        HAL_Delay(BMP581_DELAY_US_SOFT_RESET / 1000);
        return BMP581_PowerUpCheck();
    }
    return status;
}


/* ===== DIAGNOSTIC FUNCTIONS ===== */

void BMP581_DiagnoseIssue(void) {
    extern void DebugPrint(const char *format, ...);
    uint8_t reg_value;

    DebugPrint("\r\n========================================\r\n");
    DebugPrint("  BMP581 Comprehensive Diagnostic\r\n");
    DebugPrint("========================================\r\n\r\n");

    if (BMP581_ReadRegister(BMP581_REG_CHIP_ID, &reg_value) == HAL_OK) {
        DebugPrint("1. Chip ID: 0x%02X (Expected: 0x50) -> %s\r\n", reg_value, (reg_value == 0x50) ? "PASS" : "FAIL");
    } else {
        DebugPrint("1. Chip ID: FAILED TO READ\r\n");
        return;
    }

    if (BMP581_ReadRegister(BMP581_REG_STATUS, &reg_value) == HAL_OK) {
        DebugPrint("2. NVM Status (STATUS=0x%02X):\r\n", reg_value);
        DebugPrint("   - NVM Ready (bit 1): %s\r\n", (reg_value & BMP581_STATUS_NVM_RDY) ? "YES" : "NO");
        DebugPrint("   - NVM Error (bit 2): %s\r\n", (reg_value & BMP581_STATUS_NVM_ERR) ? "YES (ERROR)" : "NO");
    }

    if (BMP581_ReadRegister(BMP581_REG_INT_STATUS, &reg_value) == HAL_OK) {
        DebugPrint("3. Reset/Data Status (INT_STATUS=0x%02X):\r\n", reg_value);
        DebugPrint("   - POR Complete (bit 4): %s\r\n", (reg_value & BMP581_INT_STATUS_POR) ? "YES" : "NO");
        DebugPrint("   - Data Ready (bit 0):   %s\r\n", (reg_value & BMP581_INT_STATUS_DRDY) ? "YES" : "NO");
    }

    if (BMP581_ReadRegister(BMP581_REG_OSR_CONFIG, &reg_value) == HAL_OK) {
        DebugPrint("4. Pressure Enable (OSR_CONFIG=0x%02X): %s\r\n", reg_value, (reg_value & (1 << BMP581_PRESS_EN_POS)) ? "YES" : "NO");
    }

    DebugPrint("5. Raw Pressure Data:\r\n");
    uint8_t raw[3];
    if (HAL_I2C_Mem_Read(&hi2c1, BMP581_I2C_ADDR, BMP581_REG_PRESS_DATA_XLSB, I2C_MEMADD_SIZE_8BIT, raw, 3, 100) == HAL_OK) {
        DebugPrint("   - Raw Bytes: 0x%02X 0x%02X 0x%02X", raw[2], raw[1], raw[0]);
        if (raw[0] == 0x7F && raw[1] == 0x7F && raw[2] == 0x7F) {
            DebugPrint(" -> STUCK AT 0x7F7F7F!\r\n");
        } else if (raw[0] == 0x00 && raw[1] == 0x00 && raw[2] == 0x80) {
            DebugPrint(" -> STUCK AT 0x800000!\r\n");
        }
        else {
            DebugPrint(" -> Looks valid\r\n");
        }
    }
    DebugPrint("========================================\r\n\r\n");
}


void BMP581_DumpRegisters(void) {
    extern void DebugPrint(const char *format, ...);
    uint8_t reg_value;

    DebugPrint("\r\n========================================\r\n");
    DebugPrint("    BMP581 Register Dump\r\n");
    DebugPrint("========================================\r\n\r\n");

    DebugPrint("--- Device ID ---\r\n");
    if (BMP581_ReadRegister(BMP581_REG_CHIP_ID, &reg_value) == HAL_OK) {
        DebugPrint("CHIP_ID (0x01):       0x%02X\r\n", reg_value);
    }
    if (BMP581_ReadRegister(BMP581_REG_REV_ID, &reg_value) == HAL_OK) {
        DebugPrint("REV_ID (0x02):        0x%02X\r\n", reg_value);
    }

    DebugPrint("\r\n--- Status Registers ---\r\n");
    if (BMP581_ReadRegister(BMP581_REG_STATUS, &reg_value) == HAL_OK) {
        DebugPrint("STATUS (0x28):        0x%02X\r\n", reg_value);
        DebugPrint("  - NVM ready:        %s\r\n", (reg_value & BMP581_STATUS_NVM_RDY) ? "YES" : "NO");
        DebugPrint("  - NVM error:        %s\r\n", (reg_value & BMP581_STATUS_NVM_ERR) ? "YES" : "NO");
    }
    if (BMP581_ReadRegister(BMP581_REG_INT_STATUS, &reg_value) == HAL_OK) {
        DebugPrint("INT_STATUS (0x27):    0x%02X\r\n", reg_value);
        DebugPrint("  - Data ready:       %s\r\n", (reg_value & BMP581_INT_STATUS_DRDY) ? "YES" : "NO");
        DebugPrint("  - POR complete:     %s\r\n", (reg_value & BMP581_INT_STATUS_POR) ? "YES" : "NO");
    }

    DebugPrint("\r\n--- Configuration ---\r\n");
    if (BMP581_ReadRegister(BMP581_REG_ODR_CONFIG, &reg_value) == HAL_OK) {
        DebugPrint("ODR_CONFIG (0x37):    0x%02X\r\n", reg_value);
    }
    if (BMP581_ReadRegister(BMP581_REG_OSR_CONFIG, &reg_value) == HAL_OK) {
        DebugPrint("OSR_CONFIG (0x36):    0x%02X\r\n", reg_value);
    }
    if (BMP581_ReadRegister(BMP581_REG_DSP_IIR, &reg_value) == HAL_OK) {
        DebugPrint("DSP_IIR (0x31):       0x%02X\r\n", reg_value);
    }

    DebugPrint("\r\n========================================\r\n\r\n");
}


void BMP581_TestDataUpdating(void) {
    extern void DebugPrint(const char *format, ...);

    DebugPrint("\r\n========================================\r\n");
    DebugPrint("  BMP581 Data Update Test\r\n");
    DebugPrint("========================================\r\n\r\n");

    uint8_t raw_data[6];
    int32_t prev_press_raw = 0;
    bool first_sample = true;

    for (int i = 0; i < 5; i++) {
        HAL_Delay(50); // Wait for a new sample at ~20Hz

        if (BMP581_ReadRegisters(BMP581_REG_TEMP_DATA_XLSB, raw_data, 6) == HAL_OK) {
            int32_t press_raw = ((int32_t)raw_data[5] << 16) |
                                ((int32_t)raw_data[4] << 8) |
                                ((int32_t)raw_data[3]);

            if (press_raw & 0x00800000) press_raw |= 0xFF000000;

            if (first_sample) {
                DebugPrint("Sample %d: P_raw=%ld (baseline)\r\n", i + 1, press_raw);
                first_sample = false;
            } else {
                DebugPrint("Sample %d: P_raw=%ld (delta=%ld)\r\n",
                           i + 1, press_raw, press_raw - prev_press_raw);
            }
            prev_press_raw = press_raw;
        } else {
             DebugPrint("Sample %d: Failed to read sensor data.\r\n", i + 1);
        }
    }

    DebugPrint("\r\n========================================\r\n\r\n");
}
