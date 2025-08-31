/*Set Computer IPv4 to 192.168.1.101
 TCP Port 502
 Set Subnet Mask to 255.255.255.0

NUCLEO-L011K4    →    W5500 Module
3.3V             →    VCC (NOT 5V!)
GND              →    GND
PA5 (SPI1_SCK)   →    SCK
PA6 (SPI1_MISO)  →    MISO
PA7 (SPI1_MOSI)  →    MOSI
PA4              →    CS
PA8              →    RST

POTENTIOMETER
Center LEAD      →    PA0
Left   LEAD      →    3.3V
Right  LEAD      →    GND

To Check
Use Modbus TCP client to connect to 192.168.1.100 port 502
Register 1: ADC value (0-4095)
Register 2: Part name "W5500" as 16-bit values
*/

#include "stm32l0xx_hal.h"
#include <stdio.h>
#include <string.h>

// W5500 Registers
#define W5500_MR        0x0000  // Mode Register
#define W5500_SHAR      0x0009  // MAC Address
#define W5500_SUBR      0x0005  // Subnet Mask
#define W5500_GAR       0x0001  // Gateway
#define W5500_SIPR      0x000F  // IP Address
#define W5500_S0_MR     0x0000  // Socket Mode
#define W5500_S0_CR     0x0001  // Socket Command
#define W5500_S0_IR     0x0002  // Socket Interrupt
#define W5500_S0_SR     0x0003  // Socket Status
#define W5500_S0_PORT   0x0004  // Socket Port
#define W5500_S0_TX_FSR 0x0020  // TX Free Size
#define W5500_S0_TX_RD  0x0022  // TX Read Pointer
#define W5500_S0_TX_WR  0x0024  // TX Write Pointer
#define W5500_S0_RX_RSR 0x0026  // RX Received Size
#define W5500_S0_RX_RD  0x0028  // RX Read Pointer

// Control Bytes
#define CB_COMMON       0x00
#define CB_SOCK0        0x08
#define CB_SOCK0_TX     0x10
#define CB_SOCK0_RX     0x18

// Commands
#define CMD_OPEN        0x01
#define CMD_LISTEN      0x02
#define CMD_CLOSE       0x10
#define CMD_SEND        0x20
#define CMD_RECV        0x40

// Socket Status
#define SOCK_CLOSED     0x00
#define SOCK_INIT       0x13
#define SOCK_LISTEN     0x14
#define SOCK_ESTABLISHED 0x17
#define SOCK_CLOSE_WAIT 0x1C

// Modbus Function Codes
#define MODBUS_READ_HOLDING_REGS  0x03
#define MODBUS_EXCEPTION_RESPONSE 0x80

// Modbus Exception Codes
#define MODBUS_EX_ILLEGAL_FUNCTION    0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDR   0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE  0x03

// Hardware
SPI_HandleTypeDef spi;
UART_HandleTypeDef uart;
ADC_HandleTypeDef hadc;

// Network Configuration
const uint8_t mac[6] = {0x02, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
const uint8_t ip[4] = {192, 168, 1, 100};
const uint8_t subnet[4] = {255, 255, 255, 0};
const uint8_t gateway[4] = {192, 168, 1, 1};

// Buffer
uint8_t buf[128];

// Modbus registers
uint16_t modbus_regs[10] = {0}; // Register 0-9, we'll use 1 and 2

// Function prototypes
void SystemClock_Config(void);
void GPIO_Init(void);
void SPI_Init(void);
void UART_Init(void);
void Print(const char* msg);
void W5500_Init(void);
void W5500_Write(uint16_t addr, uint8_t cb, uint8_t data);
uint8_t W5500_Read(uint16_t addr, uint8_t cb);
void W5500_WriteN(uint16_t addr, uint8_t cb, const uint8_t* data, uint8_t len);
void W5500_ReadN(uint16_t addr, uint8_t cb, uint8_t* data, uint8_t len);
uint8_t W5500_SocketInit(void);
uint16_t W5500_Recv(void);
void W5500_Send(const uint8_t* data, uint16_t len);
uint16_t Read_ADC_PA0(void);
void MX_ADC_Init(void);
void Process_Modbus_Request(void);
uint16_t Calculate_CRC16(uint8_t* data, uint16_t length);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    SPI_Init();
    UART_Init();
    MX_ADC_Init();

    Print("=== W5500 Modbus TCP Server ===\r\n");

    // Initialize part name "W5500" in registers 2-4 (as 16-bit values)
    modbus_regs[1] = ('W' << 8) | '5';  // "W5"
    modbus_regs[2] = ('5' << 8) | '0';  // "50"
    modbus_regs[3] = ('0' << 8) | 0;    // "0" + null terminator

    // Initialize W5500
    W5500_Init();

    // Initialize socket
    if(W5500_SocketInit()) {
    } else {
        while(1);
    }

    uint8_t last_status = 0xFF;
    uint32_t status_count = 0;

    while(1) {
        // Read ADC value and store in register 1
        uint16_t adc_raw = Read_ADC_PA0();
        modbus_regs[0] = adc_raw;

        // Convert to millivolts for display
        uint16_t voltage_mv = (adc_raw * 3300) / 4095;

        char adc_str[5];
        adc_str[0] = '0' + (adc_raw / 1000);
        adc_str[1] = '0' + ((adc_raw / 100) % 10);
        adc_str[2] = '0' + ((adc_raw / 10) % 10);
        adc_str[3] = '0' + (adc_raw % 10);
        adc_str[4] = 0;

        char volt_str[6];
        volt_str[0] = '0' + (voltage_mv / 1000);
        volt_str[1] = '.';
        volt_str[2] = '0' + ((voltage_mv / 100) % 10);
        volt_str[3] = '0' + ((voltage_mv / 10) % 10);
        volt_str[4] = '0' + (voltage_mv % 10);
        volt_str[5] = 0;

        uint8_t status = W5500_Read(W5500_S0_SR, CB_SOCK0);

        // Print status changes AND periodic updates
        if(status != last_status || (status_count % 1000 == 0)) {
            switch(status) {
                case SOCK_CLOSED:
                    HAL_Delay(100);
                    W5500_SocketInit();  // Restart
                    break;
                case SOCK_INIT:
                    break;
                case SOCK_LISTEN:
                    Print("Modbus TCP Server Listening on port 502\r\n");
                    break;
                case SOCK_ESTABLISHED:
                    Print("Modbus Client Connected\r\n");
                    break;
                case SOCK_CLOSE_WAIT:
                    break;
                default:
                    Print("STATUS 0x");
                    char hex[3] = {'0'+(status>>4), '0'+(status&0xF), 0};
                    if(hex[0] > '9') hex[0] = 'A' + (hex[0]-'0'-10);
                    if(hex[1] > '9') hex[1] = 'A' + (hex[1]-'0'-10);
                    Print(hex);
                    Print("\r\n");
                    break;
            }
            last_status = status;
        }
        status_count++;

        // Check for incoming connections
        if(status == SOCK_LISTEN) {
            uint8_t ir = W5500_Read(W5500_S0_IR, CB_SOCK0);
            if(ir != 0) {
                W5500_Write(W5500_S0_IR, CB_SOCK0, ir);
            }
        }

        if(status == SOCK_ESTABLISHED) {
            uint16_t recv_len = W5500_Recv();
            if(recv_len > 0) {
                Print("Received Modbus request (");
                char len_str[4];
                len_str[0] = '0' + (recv_len / 100);
                len_str[1] = '0' + ((recv_len / 10) % 10);
                len_str[2] = '0' + (recv_len % 10);
                len_str[3] = 0;
                Print(len_str);
                Print(" bytes)\r\n");

                Process_Modbus_Request();

                // Wait for send to complete
                HAL_Delay(100);
            }
        } else if(status == SOCK_CLOSE_WAIT) {
            W5500_Write(W5500_S0_CR, CB_SOCK0, CMD_CLOSE);
            HAL_Delay(50);
        }

        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_Delay(50);
    }
}

void Process_Modbus_Request(void)
{
    // Parse Modbus TCP header (MBAP)
    uint16_t transaction_id = (buf[0] << 8) | buf[1];
    uint16_t protocol_id = (buf[2] << 8) | buf[3];
    uint16_t length = (buf[4] << 8) | buf[5];
    uint8_t unit_id = buf[6];

    // Parse PDU
    uint8_t function_code = buf[7];

    uint8_t response[32];
    uint16_t response_len = 0;

    // Build MBAP header for response
    response[0] = transaction_id >> 8;
    response[1] = transaction_id & 0xFF;
    response[2] = 0; // Protocol ID = 0 for Modbus
    response[3] = 0;
    response[4] = 0; // Length - will be filled later
    response[5] = 0;
    response[6] = unit_id;

    response_len = 7; // MBAP header length

    if(function_code == MODBUS_READ_HOLDING_REGS) {
        uint16_t start_addr = (buf[8] << 8) | buf[9];
        uint16_t reg_count = (buf[10] << 8) | buf[11];

        // Check if address and count are valid
        if(start_addr >= 10 || (start_addr + reg_count) > 10 || reg_count == 0 || reg_count > 125) {
            // Exception response - Illegal Data Address
            response[7] = MODBUS_READ_HOLDING_REGS | MODBUS_EXCEPTION_RESPONSE;
            response[8] = MODBUS_EX_ILLEGAL_DATA_ADDR;
            response_len = 9;
        } else {
            // Normal response
            response[7] = MODBUS_READ_HOLDING_REGS;
            response[8] = reg_count * 2; // Byte count
            response_len = 9;

            // Add register data (big-endian)
            for(uint16_t i = 0; i < reg_count; i++) {
                uint16_t reg_value = modbus_regs[start_addr + i];
                response[response_len++] = reg_value >> 8;
                response[response_len++] = reg_value & 0xFF;
            }
        }
    } else {
        // Exception response - Illegal Function
        response[7] = function_code | MODBUS_EXCEPTION_RESPONSE;
        response[8] = MODBUS_EX_ILLEGAL_FUNCTION;
        response_len = 9;
    }

    // Set length in MBAP header (PDU length)
    uint16_t pdu_length = response_len - 6;
    response[4] = pdu_length >> 8;
    response[5] = pdu_length & 0xFF;

    // Send response
    W5500_Send(response, response_len);

    Print("Sent Modbus response\r\n");
}

void W5500_Init(void)
{
    // Hardware reset
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
    HAL_Delay(100);

    // Verify communication
    uint8_t version = W5500_Read(0x0039, CB_COMMON);
    if(version != 0x04) {
        while(1);
    }

    // Configure network settings
    W5500_WriteN(W5500_SHAR, CB_COMMON, mac, 6);      // MAC address
    W5500_WriteN(W5500_GAR, CB_COMMON, gateway, 4);   // Gateway
    W5500_WriteN(W5500_SUBR, CB_COMMON, subnet, 4);   // Subnet mask
    W5500_WriteN(W5500_SIPR, CB_COMMON, ip, 4);       // IP address

    // Configure socket buffer sizes (2KB each)
    W5500_Write(0x001E, CB_SOCK0, 2);  // RX buffer = 2KB
    W5500_Write(0x001F, CB_SOCK0, 2);  // TX buffer = 2KB

    HAL_Delay(100);
}

uint8_t W5500_SocketInit(void)
{
    // Close socket first
    W5500_Write(W5500_S0_CR, CB_SOCK0, CMD_CLOSE);
    HAL_Delay(10);

    // Wait for socket to close
    while(W5500_Read(W5500_S0_SR, CB_SOCK0) != SOCK_CLOSED) {
        HAL_Delay(1);
    }

    // Set TCP mode
    W5500_Write(W5500_S0_MR, CB_SOCK0, 0x01);

    // Set port 502 (Modbus TCP standard port)
    W5500_Write(W5500_S0_PORT, CB_SOCK0, 0x01);
    W5500_Write(W5500_S0_PORT+1, CB_SOCK0, 0xF6); // 502 = 0x01F6

    // Open socket
    W5500_Write(W5500_S0_CR, CB_SOCK0, CMD_OPEN);
    HAL_Delay(10);

    // Wait for socket to initialize
    uint32_t timeout = HAL_GetTick() + 1000;
    while(W5500_Read(W5500_S0_SR, CB_SOCK0) != SOCK_INIT) {
        if(HAL_GetTick() > timeout) {
            return 0;
        }
        HAL_Delay(1);
    }

    // Start listening
    W5500_Write(W5500_S0_CR, CB_SOCK0, CMD_LISTEN);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    // Wait for listen state
    timeout = HAL_GetTick() + 1000;
    while(W5500_Read(W5500_S0_SR, CB_SOCK0) != SOCK_LISTEN) {
        if(HAL_GetTick() > timeout) {
            return 0;
        }
        HAL_Delay(1);
    }
    return 1;
}

uint16_t W5500_Recv(void)
{
    // Get received data size
    uint16_t len = W5500_Read(W5500_S0_RX_RSR, CB_SOCK0) << 8;
    len |= W5500_Read(W5500_S0_RX_RSR+1, CB_SOCK0);

    if(len == 0) return 0;
    if(len > sizeof(buf)-1) len = sizeof(buf)-1;

    // Get read pointer
    uint16_t ptr = W5500_Read(W5500_S0_RX_RD, CB_SOCK0) << 8;
    ptr |= W5500_Read(W5500_S0_RX_RD+1, CB_SOCK0);

    // Read data
    W5500_ReadN(ptr, CB_SOCK0_RX, buf, len);

    // Update read pointer
    ptr += len;
    W5500_Write(W5500_S0_RX_RD, CB_SOCK0, ptr >> 8);
    W5500_Write(W5500_S0_RX_RD+1, CB_SOCK0, ptr & 0xFF);

    // Receive command
    W5500_Write(W5500_S0_CR, CB_SOCK0, CMD_RECV);

    return len;
}

void W5500_Send(const uint8_t* data, uint16_t len)
{
    // Get write pointer
    uint16_t ptr = W5500_Read(W5500_S0_TX_WR, CB_SOCK0) << 8;
    ptr |= W5500_Read(W5500_S0_TX_WR+1, CB_SOCK0);

    // Write data to TX buffer
    W5500_WriteN(ptr, CB_SOCK0_TX, data, len);

    // Update write pointer
    ptr += len;
    W5500_Write(W5500_S0_TX_WR, CB_SOCK0, ptr >> 8);
    W5500_Write(W5500_S0_TX_WR+1, CB_SOCK0, ptr & 0xFF);

    // Send command
    W5500_Write(W5500_S0_CR, CB_SOCK0, CMD_SEND);
}

void W5500_Write(uint16_t addr, uint8_t cb, uint8_t data)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
    uint8_t cmd[4] = {addr>>8, addr&0xFF, cb|0x04, data};
    HAL_SPI_Transmit(&spi, cmd, 4, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

uint8_t W5500_Read(uint16_t addr, uint8_t cb)
{
    uint8_t data;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
    uint8_t cmd[3] = {addr>>8, addr&0xFF, cb};
    HAL_SPI_Transmit(&spi, cmd, 3, 1000);
    HAL_SPI_Receive(&spi, &data, 1, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
    return data;
}

void W5500_WriteN(uint16_t addr, uint8_t cb, const uint8_t* data, uint8_t len)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
    uint8_t cmd[3] = {addr>>8, addr&0xFF, cb|0x04};
    HAL_SPI_Transmit(&spi, cmd, 3, 1000);
    HAL_SPI_Transmit(&spi, (uint8_t*)data, len, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

void W5500_ReadN(uint16_t addr, uint8_t cb, uint8_t* data, uint8_t len)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
    uint8_t cmd[3] = {addr>>8, addr&0xFF, cb};
    HAL_SPI_Transmit(&spi, cmd, 3, 1000);
    HAL_SPI_Receive(&spi, data, len, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

void Print(const char* msg)
{
    int len = 0;
    while(msg[len]) len++;
    HAL_UART_Transmit(&uart, (uint8_t*)msg, len, 1000);
}

uint16_t Read_ADC_PA0(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = ADC_CHANNEL_0;  // PA0
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADC_Start(&hadc);

    if(HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
        uint16_t result = HAL_ADC_GetValue(&hadc);
        HAL_ADC_Stop(&hadc);
        return result;
    }

    HAL_ADC_Stop(&hadc);
    return 0;
}

void MX_ADC_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc.Init.OversamplingMode = DISABLE;

    HAL_ADC_Init(&hadc);
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
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
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
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
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    // PA0 as analog input for ADC
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    // SPI pins
    g.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    // UART TX
    g.Pin = GPIO_PIN_2;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    // W5500 control pins
    g.Pin = GPIO_PIN_3;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 , GPIO_PIN_RESET);

    // W5500 control pins
    g.Pin = GPIO_PIN_4|GPIO_PIN_8;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, 1);
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
    HAL_Delay(5000);

    clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}
