/**
  ******************************************************************************
  * @file    i2c_dma_arbiter.c
  * @brief   I2C DMA arbiter implementation
  ******************************************************************************
  */

#include "i2c_dma_arbiter.h"

/* Arbiter state for each I2C peripheral */
typedef struct {
    volatile bool busy;
    volatile I2C_DMA_Device_t active_device;
    I2C_DMA_Callback_t callback;
} I2C_DMA_Arbiter_State_t;

/* State for I2C1 (only one we're using) */
static I2C_DMA_Arbiter_State_t i2c1_arbiter = {
    .busy = false,
    .active_device = I2C_DMA_DEVICE_NONE,
    .callback = NULL
};

/**
 * @brief Initialize the I2C DMA arbiter
 */
void I2C_DMA_Arbiter_Init(void) {
    i2c1_arbiter.busy = false;
    i2c1_arbiter.active_device = I2C_DMA_DEVICE_NONE;
    i2c1_arbiter.callback = NULL;
}

/**
 * @brief Request a DMA transfer (thread-safe via busy flag)
 * @param hi2c: I2C handle
 * @param device: Device requesting transfer
 * @param dev_addr: I2C device address
 * @param mem_addr: Memory address to read from
 * @param mem_addr_size: Memory address size (I2C_MEMADD_SIZE_8BIT or 16BIT)
 * @param buffer: DMA buffer to receive data
 * @param size: Number of bytes to read
 * @param callback: Function to call when transfer completes
 * @retval HAL_OK if transfer started, HAL_BUSY if arbiter busy, HAL_ERROR on failure
 */
HAL_StatusTypeDef I2C_DMA_Arbiter_RequestTransfer(
    I2C_HandleTypeDef *hi2c,
    I2C_DMA_Device_t device,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_addr_size,
    uint8_t *buffer,
    uint16_t size,
    I2C_DMA_Callback_t callback
) {
    HAL_StatusTypeDef status;

    /* Only support I2C1 for now */
    if (hi2c != &hi2c1) {
        return HAL_ERROR;
    }

    /* Check if arbiter is busy */
    if (i2c1_arbiter.busy) {
        return HAL_BUSY;
    }

    /* Mark arbiter as busy BEFORE starting DMA */
    i2c1_arbiter.busy = true;
    i2c1_arbiter.active_device = device;
    i2c1_arbiter.callback = callback;

    /* Start DMA transfer */
    status = HAL_I2C_Mem_Read_DMA(hi2c, dev_addr, mem_addr,
                                   mem_addr_size, buffer, size);

    if (status != HAL_OK) {
        /* Transfer failed to start - clear arbiter */
        i2c1_arbiter.busy = false;
        i2c1_arbiter.active_device = I2C_DMA_DEVICE_NONE;
        i2c1_arbiter.callback = NULL;
        return status;
    }

    return HAL_OK;
}

/**
 * @brief Called from HAL I2C DMA complete callback
 * @note This should be called from HAL_I2C_MemRxCpltCallback
 */
void I2C_DMA_Arbiter_TransferComplete(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) {
        return;
    }

    /* Call the device-specific callback */
    if (i2c1_arbiter.callback != NULL) {
        i2c1_arbiter.callback();
    }

    /* Clear arbiter state */
    i2c1_arbiter.busy = false;
    i2c1_arbiter.active_device = I2C_DMA_DEVICE_NONE;
    i2c1_arbiter.callback = NULL;
}

/**
 * @brief Called from HAL I2C error callback
 * @note This should be called from HAL_I2C_ErrorCallback
 */
void I2C_DMA_Arbiter_TransferError(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) {
        return;
    }

    /* Call the device-specific callback (so it can clear its busy flag) */
    if (i2c1_arbiter.callback != NULL) {
        i2c1_arbiter.callback();
    }

    /* Clear arbiter state */
    i2c1_arbiter.busy = false;
    i2c1_arbiter.active_device = I2C_DMA_DEVICE_NONE;
    i2c1_arbiter.callback = NULL;
}

/**
 * @brief Check if arbiter is busy
 * @retval true if DMA transfer in progress, false otherwise
 */
bool I2C_DMA_Arbiter_IsBusy(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c1) {
        return false;
    }

    return i2c1_arbiter.busy;
}
