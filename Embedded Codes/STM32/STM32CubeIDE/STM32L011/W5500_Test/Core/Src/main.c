/* W5500 Hardware Test - SPI Communication Verified
 * Now testing W5500 module specifically
 */

#include "stm32l0xx_hal.h"

SPI_HandleTypeDef spi;
UART_HandleTypeDef uart;

void SystemClock_Config(void);
void GPIO_Init(void);
void SPI_Init(void);
void UART_Init(void);
void Print(const char* msg);
void W5500_Write(uint16_t addr, uint8_t cb, uint8_t data);
uint8_t W5500_Read(uint16_t addr, uint8_t cb);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    SPI_Init();
    UART_Init();

    Print("=== W5500 Hardware Test ===\r\n");
    HAL_Delay(2000);

    // Test W5500 reset pin
    Print("Testing Reset Pin (PA8)...\r\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);  // High
    Print("Reset HIGH\r\n");
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);  // Low
    Print("Reset LOW\r\n");
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);  // High
    Print("Reset HIGH (W5500 should boot now)\r\n");
    HAL_Delay(200);

    // Test multiple registers to see what we get
    Print("\r\nReading W5500 registers:\r\n");

    struct {
        uint16_t addr;
        uint8_t cb;
        const char* name;
    } test_regs[] = {
        {0x0039, 0x00, "Version"},
        {0x0000, 0x00, "Mode"},
        {0x0009, 0x00, "MAC[0]"},
        {0x000A, 0x00, "MAC[1]"},
        {0x000F, 0x00, "IP[0]"},
        {0x0010, 0x00, "IP[1]"}
    };

    int num_regs = sizeof(test_regs) / sizeof(test_regs[0]);

    for(int i = 0; i < num_regs; i++) {
        uint8_t value = W5500_Read(test_regs[i].addr, test_regs[i].cb);

        Print(test_regs[i].name);
        Print(": 0x");
        char hex[3] = {'0'+(value>>4), '0'+(value&0xF), 0};
        if(hex[0] > '9') hex[0] = 'A' + (hex[0]-'0'-10);
        if(hex[1] > '9') hex[1] = 'A' + (hex[1]-'0'-10);
        Print(hex);
        Print("\r\n");
    }

    // Try to read version register multiple times
    Print("\r\nVersion register stability test:\r\n");
    for(int i = 0; i < 5; i++) {
        uint8_t ver = W5500_Read(0x0039, 0x00);
        Print("Read ");
        Print('0' + i);
        Print(": 0x");
        char hex[3] = {'0'+(ver>>4), '0'+(ver&0xF), 0};
        if(hex[0] > '9') hex[0] = 'A' + (hex[0]-'0'-10);
        if(hex[1] > '9') hex[1] = 'A' + (hex[1]-'0'-10);
        Print(hex);
        Print("\r\n");
        HAL_Delay(100);
    }

    // Test if we can write to a register
    Print("\r\nTesting write capability...\r\n");

    // Try writing to mode register
    uint8_t original = W5500_Read(0x0000, 0x00);
    Print("Original Mode: 0x");
    char hex_orig[3] = {'0'+(original>>4), '0'+(original&0xF), 0};
    if(hex_orig[0] > '9') hex_orig[0] = 'A' + (hex_orig[0]-'0'-10);
    if(hex_orig[1] > '9') hex_orig[1] = 'A' + (hex_orig[1]-'0'-10);
    Print(hex_orig);
    Print("\r\n");

    // Write test pattern
    W5500_Write(0x0000, 0x00, 0x55);
    HAL_Delay(10);
    uint8_t read_back = W5500_Read(0x0000, 0x00);
    Print("After write 0x55, read: 0x");
    char hex_rb[3] = {'0'+(read_back>>4), '0'+(read_back&0xF), 0};
    if(hex_rb[0] > '9') hex_rb[0] = 'A' + (hex_rb[0]-'0'-10);
    if(hex_rb[1] > '9') hex_rb[1] = 'A' + (hex_rb[1]-'0'-10);
    Print(hex_rb);
    Print("\r\n");

    // Restore original
    W5500_Write(0x0000, 0x00, original);

    Print("\r\n=== Analysis ===\r\n");
    Print("Expected Version: 0x04\r\n");
    Print("If all reads are 0xFF: No W5500 response (power/connection)\r\n");
    Print("If all reads are 0x00: W5500 not responding (reset/power)\r\n");
    Print("If Version = 0x04: W5500 communication OK!\r\n");

    while(1) {
        HAL_Delay(1000);
    }
}

void W5500_Write(uint16_t addr, uint8_t cb, uint8_t data)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);  // CS Low

    uint8_t cmd[4] = {addr>>8, addr&0xFF, cb|0x04, data};
    HAL_SPI_Transmit(&spi, cmd, 4, 1000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);  // CS High
}

uint8_t W5500_Read(uint16_t addr, uint8_t cb)
{
    uint8_t data = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);  // CS Low

    uint8_t cmd[3] = {addr>>8, addr&0xFF, cb};
    HAL_SPI_Transmit(&spi, cmd, 3, 1000);
    HAL_SPI_Receive(&spi, &data, 1, 1000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);  // CS High
    return data;
}

void Print(const char* msg)
{
    int len = 0;
    while(msg[len]) len++;
    HAL_UART_Transmit(&uart, (uint8_t*)msg, len, 1000);
}

void SPI_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    spi.Instance = SPI1;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 7;

    HAL_SPI_Init(&spi);
}

void UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    uart.Instance = USART2;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_UART_Init(&uart);
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    // SPI pins: PA5=SCK, PA6=MISO, PA7=MOSI
    g.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    // UART TX: PA2
    g.Pin = GPIO_PIN_2;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    // W5500 Control pins: PA4=CS, PA8=Reset
    g.Pin = GPIO_PIN_4|GPIO_PIN_8;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    // Set initial states
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);   // CS High
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);   // Reset High
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState = RCC_MSI_ON;
    osc.MSICalibrationValue = 0;
    osc.MSIClockRange = RCC_MSIRANGE_5;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}
