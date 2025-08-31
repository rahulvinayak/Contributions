/* Simple PA0 ADC Debug for NUCLEO-L011K4
 * Reads PA0 analog voltage and displays in debug terminal
 */

#include "stm32l0xx_hal.h"

// Hardware handles
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC_Init(void);
void DebugPrint(const char* msg);
uint16_t Read_ADC_PA0(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_ADC_Init();

    DebugPrint("=== PA0 ADC Monitor ===\r\n");
    DebugPrint("Connect 0-3.3V to PA0\r\n");
    DebugPrint("ADC readings every 1 second:\r\n\r\n");

    while(1)
    {
        // Read ADC value
        uint16_t adc_raw = Read_ADC_PA0();

        // Convert to millivolts (3.3V reference, 12-bit ADC)
        uint16_t voltage_mv = (adc_raw * 3300) / 4095;

        // Display raw ADC value
        DebugPrint("ADC Raw: ");
        char adc_str[5];
        adc_str[0] = '0' + (adc_raw / 1000);
        adc_str[1] = '0' + ((adc_raw / 100) % 10);
        adc_str[2] = '0' + ((adc_raw / 10) % 10);
        adc_str[3] = '0' + (adc_raw % 10);
        adc_str[4] = 0;
        DebugPrint(adc_str);

        DebugPrint(" = ");

        // Display voltage in X.XXX format
        char volt_str[6];
        volt_str[0] = '0' + (voltage_mv / 1000);
        volt_str[1] = '.';
        volt_str[2] = '0' + ((voltage_mv / 100) % 10);
        volt_str[3] = '0' + ((voltage_mv / 10) % 10);
        volt_str[4] = '0' + (voltage_mv % 10);
        volt_str[5] = 0;
        DebugPrint(volt_str);

        DebugPrint("V\r\n");

        // Wait 1 second
        HAL_Delay(1000);
    }
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

    DebugPrint("ADC initialized for PA0\r\n");
}

void DebugPrint(const char* msg)
{
    uint16_t len = 0;
    while(msg[len]) len++;
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 1000);
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

    // PA0 as analog input for ADC
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // USART2 TX (PA2)
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
