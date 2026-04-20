#include "bmp180.h"

/* ========================= DEFINIÇÕES DO SENSOR ========================= */
// Endereços internos do BMP180 (definidos no datasheet)

// Endereço inicial dos coeficientes de calibração (armazenados na EEPROM interna)
#define BMP180_REG_CALIB   0xAA  

// Registrador usado para enviar comandos ao sensor
#define BMP180_REG_CONTROL 0xF4  

// Registrador onde o resultado da medição é armazenado
#define BMP180_REG_RESULT  0xF6  

// Comando específico para iniciar medição de temperatura
#define BMP180_CMD_TEMP    0x2E  


/* ========================= BUFFERS ========================= */

// Buffer para armazenar os 22 bytes de calibração lidos do sensor
static uint8_t rx_buffer[22];

// Buffer para envio de comandos via I2C (registrador + comando)
static uint8_t tx_buffer[2];

// Buffer para armazenar temperatura bruta (2 bytes vindos do sensor)
static uint8_t raw_temp[2];

// Flag que indica quando a transferência via DMA terminou
// "volatile" porque é alterada dentro de uma interrupção (callback)
static volatile uint8_t dma_done = 0;


/* ========================= CALLBACK DO DMA ========================= */

// Essa função é chamada automaticamente pela HAL quando uma leitura
// via I2C com DMA termina (interrupção)
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Sinaliza que a transferência foi concluída
    dma_done = 1;
}


/* ========================= LEITURA DOS DADOS DE CALIBRAÇÃO ========================= */

// Essa função lê os coeficientes internos do BMP180
// Esses coeficientes são usados nas fórmulas de compensação
static HAL_StatusTypeDef BMP180_ReadCalib(BMP180_HandleTypedef *bmp)
{
    dma_done = 0; // Zera a flag antes de iniciar nova operação

    // Inicia leitura via DMA:
    // - hi2c → barramento I2C
    // - BMP180_ADDR → endereço do sensor
    // - BMP180_REG_CALIB → registrador inicial
    // - 1 → tamanho do endereço (8 bits)
    // - rx_buffer → onde salvar os dados
    // - 22 → quantidade de bytes
    if (HAL_I2C_Mem_Read_DMA(bmp->hi2c, BMP180_ADDR, BMP180_REG_CALIB, 1, rx_buffer, 22) != HAL_OK)
        return HAL_ERROR;

    // Espera até o DMA terminar (callback seta dma_done = 1)
    while (!dma_done);

    // Converte os bytes recebidos em valores de 16 bits (big-endian)
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

// Inicializa o sensor BMP180
// Associa o barramento I2C e lê os dados de calibração
HAL_StatusTypeDef BMP180_Init(BMP180_HandleTypedef *bmp, I2C_HandleTypeDef *hi2c)
{
    // Salva ponteiro do I2C dentro da estrutura do sensor
    bmp->hi2c = hi2c;

    // Lê coeficientes de calibração (obrigatório antes de qualquer leitura)
    return BMP180_ReadCalib(bmp);
}


/* ========================= LEITURA DE TEMPERATURA COM DMA ========================= */

// Função principal para leitura de temperatura
HAL_StatusTypeDef BMP180_ReadTemperature_DMA(BMP180_HandleTypedef *bmp, float *temperature)
{
    /* -------- 1. ENVIAR COMANDO DE MEDIÇÃO -------- */

    // Define registrador e comando
    tx_buffer[0] = BMP180_REG_CONTROL;
    tx_buffer[1] = BMP180_CMD_TEMP;

    // Envia comando via I2C usando DMA
    if (HAL_I2C_Master_Transmit_DMA(bmp->hi2c, BMP180_ADDR, tx_buffer, 2) != HAL_OK)
        return HAL_ERROR;

    // Aguarda o barramento I2C ficar livre
    // (garante que a transmissão terminou)
    while (HAL_I2C_GetState(bmp->hi2c) != HAL_I2C_STATE_READY);

    // Tempo mínimo para o sensor realizar a conversão (datasheet: ~4.5 ms)
    HAL_Delay(5);


    /* -------- 2. LER TEMPERATURA BRUTA -------- */

    dma_done = 0; // Zera flag antes da leitura

    // Lê 2 bytes do registrador de resultado (temperatura bruta)
    if (HAL_I2C_Mem_Read_DMA(bmp->hi2c, BMP180_ADDR, BMP180_REG_RESULT, 1, raw_temp, 2) != HAL_OK)
        return HAL_ERROR;

    // Espera a leitura terminar (via callback)
    while (!dma_done);

    // Converte os dois bytes em um inteiro (UT = Uncompensated Temperature)
    int32_t UT = (raw_temp[0] << 8) | raw_temp[1];


    /* -------- 3. CÁLCULO DA TEMPERATURA -------- */

    // Fórmulas oficiais do datasheet do BMP180
    // Usam os coeficientes de calibração

    int32_t X1 = ((UT - bmp->calib.AC6) * bmp->calib.AC5) >> 15;

    int32_t X2 = (bmp->calib.MC << 11) / (X1 + bmp->calib.MD);

    // B5 é usado também em cálculo de pressão
    bmp->B5 = X1 + X2;

    // Temperatura em décimos de grau (ex: 253 = 25.3°C)
    int32_t T = (bmp->B5 + 8) >> 4;

    // Converte para float em graus Celsius
    *temperature = T / 10.0;


    return HAL_OK;
}