#include "bmp180.h"

/* ========================= DEFINIÇÕES DO SENSOR ========================= */
// Endereços internos do BMP180 (datasheet)
#define BMP180_REG_CALIB   0xAA   // Início dos dados de calibração
#define BMP180_REG_CONTROL 0xF4   // Registrador de controle
#define BMP180_REG_RESULT  0xF6   // Registrador de resultado
#define BMP180_CMD_TEMP    0x2E   // Comando para iniciar medição de temperatura

/* ========================= BUFFERS ========================= */
// Buffer para leitura dos dados de calibração (22 bytes)
static uint8_t rx_buffer[22];

// Buffer para envio de comandos via I2C
static uint8_t tx_buffer[2];

// Buffer para armazenar temperatura bruta (2 bytes)
static uint8_t raw_temp[2];

// Flag usada para indicar fim da transferência via DMA
static volatile uint8_t dma_done = 0;


/* ========================= CALLBACK DO DMA ========================= */
// Essa função é chamada automaticamente quando uma leitura I2C via DMA termina
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Indica que a transferência terminou
    dma_done = 1;
}


/* ========================= LEITURA DOS DADOS DE CALIBRAÇÃO ========================= */
// Essa função lê os coeficientes internos do sensor BMP180
// Esses valores são usados para corrigir a temperatura medida
static HAL_StatusTypeDef BMP180_ReadCalib(BMP180_HandleTypedef *bmp)
{
    dma_done = 0; // Zera flag antes de iniciar DMA

    // Inicia leitura via DMA dos 22 bytes de calibração
    if (HAL_I2C_Mem_Read_DMA(bmp->hi2c, BMP180_ADDR, BMP180_REG_CALIB, 1, rx_buffer, 22) != HAL_OK)
        return HAL_ERROR;

    // Aguarda até o DMA terminar (callback seta dma_done = 1)
    while (!dma_done);

    // Monta os valores de calibração a partir dos bytes recebidos
    bmp->calib.AC1 = (rx_buffer[0] << 8) | rx_buffer[1];
    bmp->calib.AC2 = (rx_buffer[2] << 8) | rx_buffer[3];
    bmp->calib.AC3 = (rx_buffer[4] << 8) | rx_buffer[5];
    bmp->calib.AC4 = (rx_buffer[6] << 8) | rx_buffer[7];
    bmp->calib.AC5 = (rx_buffer[8] << 8) | rx_buffer[9];
    bmp->calib.AC6 = (rx_buffer[10] << 8) | rx_buffer[11];
    bmp->calib.B1  = (rx_buffer[12] << 8) | rx_buffer[13];
    bmp->calib.B2  = (rx_buffer[14] << 8) | rx_buffer[15];
    bmp->calib.MB  = (rx_buffer[16] << 8) | rx_buffer[17];
    bmp->calib.MC  = (rx_buffer[18] << 8) | rx_buffer[19];
    bmp->calib.MD  = (rx_buffer[20] << 8) | rx_buffer[21];

    return HAL_OK;
}


/* ========================= INICIALIZAÇÃO DO SENSOR ========================= */
// Associa o BMP180 ao barramento I2C e lê os dados de calibração
HAL_StatusTypeDef BMP180_Init(BMP180_HandleTypedef *bmp, I2C_HandleTypeDef *hi2c)
{
    bmp->hi2c = hi2c; // Salva ponteiro do I2C

    // Lê coeficientes de calibração
    return BMP180_ReadCalib(bmp);
}


/* ========================= LEITURA DE TEMPERATURA COM DMA ========================= */
// Realiza leitura da temperatura utilizando DMA
HAL_StatusTypeDef BMP180_ReadTemperature_DMA(BMP180_HandleTypedef *bmp, float *temperature)
{
    /* -------- 1. ENVIAR COMANDO DE MEDIÇÃO -------- */
    tx_buffer[0] = BMP180_REG_CONTROL; // Registrador de controle
    tx_buffer[1] = BMP180_CMD_TEMP;    // Comando de leitura de temperatura

    // Envia comando via I2C usando DMA
    if (HAL_I2C_Master_Transmit_DMA(bmp->hi2c, BMP180_ADDR, tx_buffer, 2) != HAL_OK)
        return HAL_ERROR;

    // Aguarda I2C ficar livre
    while (HAL_I2C_GetState(bmp->hi2c) != HAL_I2C_STATE_READY);

    // Tempo necessário para conversão interna do sensor
    HAL_Delay(5);


    /* -------- 2. LER TEMPERATURA BRUTA -------- */
    dma_done = 0; // Zera flag

    // Lê 2 bytes do registrador de resultado
    if (HAL_I2C_Mem_Read_DMA(bmp->hi2c, BMP180_ADDR, BMP180_REG_RESULT, 1, raw_temp, 2) != HAL_OK)
        return HAL_ERROR;

    // Aguarda finalização da leitura via DMA
    while (!dma_done);

    // Junta os 2 bytes em um valor inteiro (UT = Uncompensated Temperature)
    int32_t UT = (raw_temp[0] << 8) | raw_temp[1];


    /* -------- 3. CÁLCULO DA TEMPERATURA -------- */
    // Fórmulas retiradas do datasheet do BMP180

    int32_t X1 = ((UT - bmp->calib.AC6) * bmp->calib.AC5) >> 15;
    int32_t X2 = (bmp->calib.MC << 11) / (X1 + bmp->calib.MD);

    bmp->B5 = X1 + X2;

    int32_t T = (bmp->B5 + 8) >> 4;

    // Converte para float em graus Celsius
    *temperature = T / 10.0;


    return HAL_OK;
}