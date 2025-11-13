/**
  ******************************************************************************
  * @file    i2c_dma_arbiter.h
  * @brief   I2C DMA arbiter for multiple devices on same bus
  * @note    Ensures only one DMA transfer is active at a time
  ******************************************************************************
  */

#ifndef I2C_DMA_ARBITER_H_
#define I2C_DMA_ARBITER_H_

#include "main.h"
#include "i2c.h"
#include <stdbool.h>

/* Device IDs for tracking */
typedef enum {
    I2C_DMA_DEVICE_NONE = 0,
    I2C_DMA_DEVICE_MAG,
    I2C_DMA_DEVICE_BARO
} I2C_DMA_Device_t;

/* Completion callback type */
typedef void (*I2C_DMA_Callback_t)(void);

/* Function prototypes */
void I2C_DMA_Arbiter_Init(void);
HAL_StatusTypeDef I2C_DMA_Arbiter_RequestTransfer(
    I2C_HandleTypeDef *hi2c,
    I2C_DMA_Device_t device,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_addr_size,
    uint8_t *buffer,
    uint16_t size,
    I2C_DMA_Callback_t callback
);
void I2C_DMA_Arbiter_TransferComplete(I2C_HandleTypeDef *hi2c);
void I2C_DMA_Arbiter_TransferError(I2C_HandleTypeDef *hi2c);
bool I2C_DMA_Arbiter_IsBusy(I2C_HandleTypeDef *hi2c);

#endif /* I2C_DMA_ARBITER_H_ */
