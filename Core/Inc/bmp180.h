
#define BMP180_H

#include "stm32f1xx_hal.h"

typedef struct {
    int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2, MB, MC, MD;
} BMP180_CalibData;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    BMP180_CalibData calib;
    int32_t B5;
} BMP180_HandleTypedef;

// Endereço padrão
#define BMP180_ADDR (0x77 << 1)

// API
HAL_StatusTypeDef BMP180_Init(BMP180_HandleTypedef *bmp, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMP180_ReadTemperature_DMA(BMP180_HandleTypedef *bmp, float *temperature);
