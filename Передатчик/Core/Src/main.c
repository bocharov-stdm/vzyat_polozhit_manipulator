/* main.c — ПЕРЕДАТЧИК: 6 потенциометров -> UART1 с фильтрацией от шумов
 *
 * Потенциометры (TX):
 *  CH0: PA0 (ADC1_IN0) -> Servo1 (RX: PA7 / TIM3_CH2)
 *  CH1: PA1 (ADC1_IN1) -> Servo2 (RX: PB0 / TIM3_CH3)
 *  CH2: PA2 (ADC1_IN2) -> Servo3 (RX: PB1 / TIM3_CH4)
 *  CH3: PA3 (ADC1_IN3) -> Servo4 (RX: PB6 / TIM4_CH1)
 *  CH4: PA4 (ADC1_IN4) -> Servo5 (RX: PB7 / TIM4_CH2)
 *  CH5: PA5 (ADC1_IN5) -> Servo6 (RX: PB9 / TIM4_CH4)
 */

#include "main.h"

/* Глобальные хэндлы */
ADC_HandleTypeDef   hadc1;
UART_HandleTypeDef  huart1;

/* Прототипы локальных функций */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

/* --------- Конфиг протокола --------- */

#define NUM_CHANNELS      6U
#define FRAME_SIZE        (3U + NUM_CHANNELS*2U + 1U)   // 3 байта заголовок + данные + CRC

/* Каналы АЦП для 6 потенциометров: PA0..PA5 */
static const uint32_t adc_channels[NUM_CHANNELS] = {
    ADC_CHANNEL_0, // PA0 -> Servo1
    ADC_CHANNEL_1, // PA1 -> Servo2
    ADC_CHANNEL_2, // PA2 -> Servo3
    ADC_CHANNEL_3, // PA3 -> Servo4
    ADC_CHANNEL_4, // PA4 -> Servo5
    ADC_CHANNEL_5  // PA5 -> Servo6
};

/* Фильтрованные значения АЦП (0..4095), 32 бита для точности */
static uint32_t adc_filtered[NUM_CHANNELS] = {
    2048U, 2048U, 2048U, 2048U, 2048U, 2048U
};

/* Чтение одного канала АЦП (блокирующее) */
static uint16_t Read_ADC_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        Error_Handler();

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
        Error_Handler();

    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
    return val;        // 0..4095
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();

    uint8_t  frame[FRAME_SIZE];

    while (1)
    {
        /* 1. Считываем и фильтруем 6 потенциометров */
        for (uint8_t i = 0; i < NUM_CHANNELS; i++)
        {
            uint16_t raw = Read_ADC_Channel(adc_channels[i]);

            // Простейший IIR-фильтр: новое = 7/8 старого + 1/8 нового
            adc_filtered[i] = (adc_filtered[i] * 7U + (uint32_t)raw) / 8U;

            // Лёгкая квантовка, чтобы убрать дрожание на 1–2 единицы
            uint16_t v = (uint16_t)adc_filtered[i];
            v = (uint16_t)((uint32_t)v & ~0x3U); // шаг 4

            adc_filtered[i] = v;
        }

        /* 2. Формируем кадр
         * [0] = 0xAA
         * [1] = 0x55
         * [2] = NUM_CHANNELS (6)
         * дальше 6 значений по 2 байта (MSB, LSB)
         * последний байт = XOR всех предыдущих (CRC)
         */
        frame[0] = 0xAA;
        frame[1] = 0x55;
        frame[2] = NUM_CHANNELS;

        for (uint8_t i = 0; i < NUM_CHANNELS; i++)
        {
            uint16_t v = (uint16_t)adc_filtered[i];
            frame[3 + 2U*i]     = (uint8_t)(v >> 8);   // старший байт
            frame[3 + 2U*i + 1] = (uint8_t)(v & 0xFF); // младший
        }

        /* 3. Контрольная сумма XOR */
        uint8_t crc = 0;
        for (uint8_t i = 0; i < FRAME_SIZE - 1U; i++)
            crc ^= frame[i];

        frame[FRAME_SIZE - 1U] = crc;

        /* 4. Отправляем */
        HAL_UART_Transmit(&huart1, frame, FRAME_SIZE, HAL_MAX_DELAY);

        /* Можно мигать диодом на PA6:
         * HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
         */

        HAL_Delay(20); // период отправки ~20 мс (50 Гц)
    }
}

/* ---------------------- System Clock ---------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;  // 84 МГц
    RCC_OscInitStruct.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  |
                                  RCC_CLOCKTYPE_SYSCLK|
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* ---------------------- ADC1 init ---------------------- */

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();
}

/* ---------------------- USART1 init ---------------------- */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/* ---------------------- GPIO init ---------------------- */

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // сюда можно добавить диод на PA6, если надо
}

/* ---------------------- Error handler ---------------------- */

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
