/* TCP Testing with W5500
To Test go to web browser and enter http://192.168.1.100
or Go to CMD and Type ping 192.168.1.100
 */

#include "stm32l0xx_hal.h"

// W5500 Essential registers
#define W5500_SHAR      0x0009
#define W5500_SIPR      0x000F
#define W5500_S0_MR     0x0000
#define W5500_S0_CR     0x0001
#define W5500_S0_SR     0x0003
#define W5500_S0_PORT   0x0004
#define W5500_S0_TX_WR  0x0024
#define W5500_S0_RX_RSR 0x0026
#define W5500_S0_RX_RD  0x0028

#define CB_COMMON   0x00
#define CB_SOCK0    0x08
#define CB_SOCK0_TX 0x10
#define CB_SOCK0_RX 0x18

#define CMD_OPEN   0x01
#define CMD_LISTEN 0x02
#define CMD_SEND   0x20
#define CMD_RECV   0x40

#define ST_LISTEN  0x14
#define ST_ESTAB   0x17

// Hardware handles
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

// Network config
const uint8_t mac_addr[6] = {0x02, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
const uint8_t ip_addr[4] = {192, 168, 1, 100};
const uint8_t subnet_mask[4] = {255, 255, 255, 0};
const uint8_t gateway_addr[4] = {192, 168, 1, 1};

uint8_t rx_buffer[32];

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_USART2_UART_Init(void);
void DebugPrint(const char* msg);
void W5500_Reset(void);
void W5500_SetNetwork(void);
void W5500_WriteReg(uint16_t addr, uint8_t cb, uint8_t data);
uint8_t W5500_ReadReg(uint16_t addr, uint8_t cb);
void W5500_WriteMulti(uint16_t addr, uint8_t cb, const uint8_t* data, uint8_t len);
void W5500_ReadMulti(uint16_t addr, uint8_t cb, uint8_t* data, uint8_t len);
uint16_t W5500_ReceiveData(void);
void W5500_SendData(const char* msg);
void W5500_InitSocket(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();

    DebugPrint("=== W5500 Test ===\r\n");

    // Initialize W5500
    W5500_Reset();
    HAL_Delay(100);
    W5500_SetNetwork();

    // Test W5500 communication
    uint8_t version = W5500_ReadReg(0x0039, CB_COMMON);
    DebugPrint("W5500 Version: 0x");
    char hex[3] = {'0'+(version>>4), '0'+(version&0xF), 0};
    if(hex[0] > '9') hex[0] = 'A' + (hex[0]-'0'-10);
    if(hex[1] > '9') hex[1] = 'A' + (hex[1]-'0'-10);
    DebugPrint(hex);
    DebugPrint("\r\n");

    if(version != 0x04) {
        DebugPrint("ERROR: W5500 not detected!\r\n");
        while(1) { HAL_Delay(1000); }
    }

    DebugPrint("W5500 detected successfully\r\n");
    DebugPrint("IP: 192.168.1.100\r\n");
    DebugPrint("Configure PC: 192.168.1.101\r\n");

    // Initialize TCP socket
    W5500_InitSocket();

    DebugPrint("Web server ready\r\n");
    DebugPrint("Test: http://192.168.1.100\r\n");

    while(1) {
        uint8_t status = W5500_ReadReg(W5500_S0_SR, CB_SOCK0);

        if(status == ST_ESTAB) {
            uint16_t len = W5500_ReceiveData();
            if(len > 0) {
                DebugPrint("HTTP Request received\r\n");
                W5500_SendData("HTTP/1.0 200 OK\r\n\r\n<h1>STM32 Works!</h1><p>W5500 Ethernet Test</p>");
                DebugPrint("Response sent\r\n");
            }
        }

        HAL_Delay(10);
    }
}

void W5500_Reset(void)
{
    DebugPrint("Resetting W5500...\r\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(100);
}

void W5500_SetNetwork(void)
{
    DebugPrint("Setting network config...\r\n");
    W5500_WriteMulti(W5500_SHAR, CB_COMMON, mac_addr, 6);
    W5500_WriteMulti(0x0001, CB_COMMON, gateway_addr, 4);  // Gateway
    W5500_WriteMulti(0x0005, CB_COMMON, subnet_mask, 4);   // Subnet
    W5500_WriteMulti(W5500_SIPR, CB_COMMON, ip_addr, 4);   // IP
    HAL_Delay(50);
    DebugPrint("Network config complete\r\n");
}

void W5500_InitSocket(void)
{
    DebugPrint("Initializing socket...\r\n");

    // Set TCP mode
    W5500_WriteReg(W5500_S0_MR, CB_SOCK0, 0x01);

    // Set port 80
    W5500_WriteReg(W5500_S0_PORT, CB_SOCK0, 0x00);
    W5500_WriteReg(W5500_S0_PORT+1, CB_SOCK0, 80);

    // Open socket
    W5500_WriteReg(W5500_S0_CR, CB_SOCK0, CMD_OPEN);
    HAL_Delay(10);

    // Start listening
    W5500_WriteReg(W5500_S0_CR, CB_SOCK0, CMD_LISTEN);
    HAL_Delay(10);

    uint8_t status = W5500_ReadReg(W5500_S0_SR, CB_SOCK0);
    DebugPrint("Socket status: 0x");
    char hex[3] = {'0'+(status>>4), '0'+(status&0xF), 0};
    if(hex[0] > '9') hex[0] = 'A' + (hex[0]-'0'-10);
    if(hex[1] > '9') hex[1] = 'A' + (hex[1]-'0'-10);
    DebugPrint(hex);
    if(status == 0x14) DebugPrint(" (LISTENING)");
    DebugPrint("\r\n");
}

uint16_t W5500_ReceiveData(void)
{
    uint16_t len = W5500_ReadReg(W5500_S0_RX_RSR, CB_SOCK0) << 8;
    len |= W5500_ReadReg(W5500_S0_RX_RSR+1, CB_SOCK0);

    if(len == 0 || len > 31) return 0;

    uint16_t ptr = W5500_ReadReg(W5500_S0_RX_RD, CB_SOCK0) << 8;
    ptr |= W5500_ReadReg(W5500_S0_RX_RD+1, CB_SOCK0);

    W5500_ReadMulti(ptr, CB_SOCK0_RX, rx_buffer, len);

    ptr += len;
    W5500_WriteReg(W5500_S0_RX_RD, CB_SOCK0, ptr >> 8);
    W5500_WriteReg(W5500_S0_RX_RD+1, CB_SOCK0, ptr & 0xFF);
    W5500_WriteReg(W5500_S0_CR, CB_SOCK0, CMD_RECV);

    return len;
}

void W5500_SendData(const char* msg)
{
    uint8_t len = 0;
    while(msg[len] && len < 100) len++;

    uint16_t ptr = W5500_ReadReg(W5500_S0_TX_WR, CB_SOCK0) << 8;
    ptr |= W5500_ReadReg(W5500_S0_TX_WR+1, CB_SOCK0);

    W5500_WriteMulti(ptr, CB_SOCK0_TX, (const uint8_t*)msg, len);

    ptr += len;
    W5500_WriteReg(W5500_S0_TX_WR, CB_SOCK0, ptr >> 8);
    W5500_WriteReg(W5500_S0_TX_WR+1, CB_SOCK0, ptr & 0xFF);
    W5500_WriteReg(W5500_S0_CR, CB_SOCK0, CMD_SEND);
    HAL_Delay(5);
}

void W5500_WriteReg(uint16_t addr, uint8_t cb, uint8_t data)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    uint8_t cmd[4] = {addr>>8, addr&0xFF, cb|0x04, data};
    HAL_SPI_Transmit(&hspi1, cmd, 4, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t W5500_ReadReg(uint16_t addr, uint8_t cb)
{
    uint8_t data;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    uint8_t cmd[3] = {addr>>8, addr&0xFF, cb};
    HAL_SPI_Transmit(&hspi1, cmd, 3, 100);
    HAL_SPI_Receive(&hspi1, &data, 1, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return data;
}

void W5500_WriteMulti(uint16_t addr, uint8_t cb, const uint8_t* data, uint8_t len)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    uint8_t cmd[3] = {addr>>8, addr&0xFF, cb|0x04};
    HAL_SPI_Transmit(&hspi1, cmd, 3, 100);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void W5500_ReadMulti(uint16_t addr, uint8_t cb, uint8_t* data, uint8_t len)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    uint8_t cmd[3] = {addr>>8, addr&0xFF, cb};
    HAL_SPI_Transmit(&hspi1, cmd, 3, 100);
    HAL_SPI_Receive(&hspi1, data, len, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void DebugPrint(const char* msg)
{
    uint16_t len = 0;
    while(msg[len]) len++;
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 1000);
}

void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;

    HAL_SPI_Init(&hspi1);
}

void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_UART_Init(&huart2);
}

void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // SPI1 pins
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // USART2 TX
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // W5500 control pins
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_SET);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}
