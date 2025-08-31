/* SPI Loopback Test for NUCLEO-L011K4
 * Connect MISO (PA6) to MOSI (PA7) with a jumper wire
 * This will test if SPI is working properly
 */

#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>

SPI_HandleTypeDef spi;
UART_HandleTypeDef uart;

void SystemClock_Config(void);
void GPIO_Init(void);
void SPI_Init(void);
void UART_Init(void);
void Print(const char* msg);
void PrintChar(char c);
void PrintNumber(int num);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    SPI_Init();
    UART_Init();

    Print("=== SPI Loopback Test ===\r\n");
    Print("Connect PA6 (MISO) to PA7 (MOSI) with jumper wire\r\n");
    Print("Starting test...\r\n\r\n");

    // Test data patterns
    uint8_t test_patterns[] = {0x00, 0xFF, 0x55, 0xAA, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    int num_tests = sizeof(test_patterns);
    int passed = 0;
    int failed = 0;

    for(int i = 0; i < num_tests; i++) {
        uint8_t tx_data = test_patterns[i];
        uint8_t rx_data = 0;

        // Pull CS low (even though we're not using it for loopback)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_Delay(1);

        // Send and receive data simultaneously
        HAL_SPI_TransmitReceive(&spi, &tx_data, &rx_data, 1, 1000);

        // Pull CS high
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
        HAL_Delay(1);

        // Check result
        Print("Test ");
        if(i < 10) PrintChar('0');
        PrintNumber(i);
        Print(": TX=0x");

        // Print TX value in hex
        char hex_tx[3];
        hex_tx[0] = (tx_data >> 4) < 10 ? '0' + (tx_data >> 4) : 'A' + (tx_data >> 4) - 10;
        hex_tx[1] = (tx_data & 0xF) < 10 ? '0' + (tx_data & 0xF) : 'A' + (tx_data & 0xF) - 10;
        hex_tx[2] = '\0';
        Print(hex_tx);

        Print(" RX=0x");

        // Print RX value in hex
        char hex_rx[3];
        hex_rx[0] = (rx_data >> 4) < 10 ? '0' + (rx_data >> 4) : 'A' + (rx_data >> 4) - 10;
        hex_rx[1] = (rx_data & 0xF) < 10 ? '0' + (rx_data & 0xF) : 'A' + (rx_data & 0xF) - 10;
        hex_rx[2] = '\0';
        Print(hex_rx);

        if(tx_data == rx_data) {
            Print(" - PASS\r\n");
            passed++;
        } else {
            Print(" - FAIL\r\n");
            failed++;
        }

        HAL_Delay(100);
    }

    Print("\r\n=== Test Summary ===\r\n");
    Print("Passed: ");
    PrintNumber(passed);
    Print("\r\nFailed: ");
    PrintNumber(failed);
    Print("\r\n");

    if(failed == 0) {
        Print("SUCCESS: SPI is working correctly!\r\n");
    } else {
        Print("ERROR: SPI communication failed!\r\n");
        Print("Check your SPI pin connections.\r\n");
    }

    // Continuous pattern test
    Print("\r\nStarting continuous test (Ctrl+C to stop)...\r\n");
    uint8_t counter = 0;

    while(1) {
        uint8_t tx = counter;
        uint8_t rx = 0;

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_SPI_TransmitReceive(&spi, &tx, &rx, 1, 1000);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);

        if(tx != rx) {
            Print("ERROR at count ");
            PrintNumber(counter);
            Print("\r\n");
        } else if(counter % 50 == 0) {
            Print(".");  // Progress indicator
        }

        counter++;
        HAL_Delay(10);
    }
}

void Print(const char* msg)
{
    int len = 0;
    while(msg[len]) len++;
    HAL_UART_Transmit(&uart, (uint8_t*)msg, len, 1000);
}

void PrintChar(char c)
{
    HAL_UART_Transmit(&uart, (uint8_t*)&c, 1, 1000);
}

void PrintNumber(int num)
{
    if(num == 0) {
        PrintChar('0');
        return;
    }

    char buffer[10];
    int i = 0;

    while(num > 0) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }

    // Print digits in reverse order
    while(i > 0) {
        PrintChar(buffer[--i]);
    }
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
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // Slower for testing
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 7;

    if(HAL_SPI_Init(&spi) != HAL_OK) {
        Print("SPI Init Error!\r\n");
    }
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

    if(HAL_UART_Init(&uart) != HAL_OK) {
        // UART init error - can't even report it!
    }
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

    // UART TX pin: PA2
    g.Pin = GPIO_PIN_2;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    // CS pin: PA4 (for testing)
    g.Pin = GPIO_PIN_4;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    // Set CS high initially
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState = RCC_MSI_ON;
    osc.MSICalibrationValue = 0;
    osc.MSIClockRange = RCC_MSIRANGE_5;  // 2.097 MHz
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}
