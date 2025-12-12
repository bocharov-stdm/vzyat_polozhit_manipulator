/* main.c — ПЕРЕДАТЧИК: 6 потенциометров -> UART1 с фильтрацией от шумов */

#include "main.h"

/* Глобальные хэндлы */
ADC_HandleTypeDef   hadc1;
UART_HandleTypeDef  huart1;

/* Прототипы */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

/* --------- Конфиг протокола --------- */

#define NUM_CHANNELS      6U
#define FRAME_SIZE        (3U + NUM_CHANNELS*2U + 1U)

/* Каналы АЦП для 6 потенциометров: PA0..PA5 */
static const uint32_t adc_channels[NUM_CHANNELS] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5
};

/* Фильтрованные значения АЦП (0..4095) */
static uint32_t adc_filtered[NUM_CHANNELS] = {
    2048U, 2048U, 2048U, 2048U, 2048U, 2048U
};

/* ---------------------- Мини-драйвер LCD (битбанг SPI) ---------------------- */
/* Пины дисплея */
#define LCD_CS_PORT   GPIOB
#define LCD_CS_PIN    GPIO_PIN_0

#define LCD_RST_PORT  GPIOB
#define LCD_RST_PIN   GPIO_PIN_6

#define LCD_RS_PORT   GPIOB
#define LCD_RS_PIN    GPIO_PIN_7

#define LCD_SCK_PORT  GPIOB
#define LCD_SCK_PIN   GPIO_PIN_3

#define LCD_SDA_PORT  GPIOB
#define LCD_SDA_PIN   GPIO_PIN_5

/* Размер экрана (типичный TFT 128x160) */
#define LCD_WIDTH   128
#define LCD_HEIGHT  160

/* Цвета RGB565 */
#define LCD_COLOR_BLACK  0x0000
#define LCD_COLOR_GREEN  0x07E0

static void LCD_Select(void)
{
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
}

static void LCD_Unselect(void)
{
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

/* Очень короткая задержка для формирования фронтов SPI */
static void LCD_DelayShort(void)
{
    for (volatile int i = 0; i < 10; i++) {
        __NOP();
    }
}

/* Вывод одного байта по программному SPI (MSB first) */
static void LCD_SPI_WriteByte(uint8_t b)
{
    for (uint8_t i = 0; i < 8; i++) {
        /* SCK низкий */
        HAL_GPIO_WritePin(LCD_SCK_PORT, LCD_SCK_PIN, GPIO_PIN_RESET);

        /* Данные на SDA */
        if (b & 0x80U) {
            HAL_GPIO_WritePin(LCD_SDA_PORT, LCD_SDA_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(LCD_SDA_PORT, LCD_SDA_PIN, GPIO_PIN_RESET);
        }

        LCD_DelayShort();

        /* Фронт SCK */
        HAL_GPIO_WritePin(LCD_SCK_PORT, LCD_SCK_PIN, GPIO_PIN_SET);
        LCD_DelayShort();

        b <<= 1;
    }
}

static void LCD_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET); // командный режим
    LCD_Select();
    LCD_SPI_WriteByte(cmd);
    LCD_Unselect();
}

static void LCD_WriteData(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);   // данные
    LCD_Select();
    LCD_SPI_WriteByte(data);
    LCD_Unselect();
}

static void LCD_WriteData16(uint16_t data)
{
    uint8_t hi = (uint8_t)(data >> 8);
    uint8_t lo = (uint8_t)(data & 0xFFU);

    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);
    LCD_Select();
    LCD_SPI_WriteByte(hi);
    LCD_SPI_WriteByte(lo);
    LCD_Unselect();
}

/* Установка области вывода */
static void LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    if (x1 >= LCD_WIDTH)  x1 = LCD_WIDTH - 1;
    if (y1 >= LCD_HEIGHT) y1 = LCD_HEIGHT - 1;

    LCD_WriteCommand(0x2A); // Column addr set
    LCD_WriteData((uint8_t)(x0 >> 8));
    LCD_WriteData((uint8_t)(x0 & 0xFFU));
    LCD_WriteData((uint8_t)(x1 >> 8));
    LCD_WriteData((uint8_t)(x1 & 0xFFU));

    LCD_WriteCommand(0x2B); // Row addr set
    LCD_WriteData((uint8_t)(y0 >> 8));
    LCD_WriteData((uint8_t)(y0 & 0xFFU));
    LCD_WriteData((uint8_t)(y1 >> 8));
    LCD_WriteData((uint8_t)(y1 & 0xFFU));

    LCD_WriteCommand(0x2C); // Memory write
}

/* Один пиксель */
static void LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
        return;

    LCD_SetAddressWindow(x, y, x, y);
    LCD_WriteData16(color);
}

/* Заливка всего экрана */
static void LCD_FillScreen(uint16_t color)
{
    LCD_SetAddressWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    uint32_t total = (uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT;
    for (uint32_t i = 0; i < total; i++) {
        LCD_WriteData16(color);
    }
}

/* Простейший 8x8 шрифт только для букв R,A,D,I,O,N и пробела */
typedef struct {
    char c;
    const uint8_t bitmap[8];
} LCD_Glyph;

/* биты идут слева направо (MSB самый левый пиксель) */
static const LCD_Glyph lcd_font[] = {
    { 'R', { 0xF0, 0x88, 0x88, 0xF0, 0xA0, 0x90, 0x88, 0x00 } }, // R
    { 'A', { 0x70, 0x88, 0x88, 0xF8, 0x88, 0x88, 0x88, 0x00 } }, // A
    { 'D', { 0xE0, 0x90, 0x88, 0x88, 0x88, 0x90, 0xE0, 0x00 } }, // D
    { 'I', { 0xE0, 0x40, 0x40, 0x40, 0x40, 0x40, 0xE0, 0x00 } }, // I
    { 'O', { 0x60, 0x90, 0x88, 0x88, 0x88, 0x90, 0x60, 0x00 } }, // O
    { 'N', { 0x88, 0xC8, 0xA8, 0x98, 0x88, 0x88, 0x88, 0x00 } }, // N
    { ' ', { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } }, // space
};

static const uint8_t* LCD_GetGlyphBitmap(char c)
{
    for (uint32_t i = 0; i < sizeof(lcd_font)/sizeof(lcd_font[0]); i++) {
        if (lcd_font[i].c == c)
            return lcd_font[i].bitmap;
    }
    return lcd_font[sizeof(lcd_font)/sizeof(lcd_font[0]) - 1].bitmap; // space
}

/* Маленький вариант (на всякий случай, сейчас не используется) */
static void LCD_DrawChar(uint16_t x, uint16_t y, char c,
                         uint16_t color, uint16_t bg)
{
    const uint8_t *bmp = LCD_GetGlyphBitmap(c);

    for (uint8_t row = 0; row < 8; row++) {
        uint8_t line = bmp[row];
        for (uint8_t col = 0; col < 8; col++) {
            uint16_t px = (line & (1U << (7 - col))) ? color : bg;
            LCD_DrawPixel(x + col, y + row, px);
        }
    }
}

/* Увеличенный символ 16x16, "жирный" — каждый пиксель 2x2 */
static void LCD_DrawCharBig(uint16_t x, uint16_t y, char c,
                            uint16_t color, uint16_t bg)
{
    const uint8_t *bmp = LCD_GetGlyphBitmap(c);

    for (uint8_t row = 0; row < 8; row++) {
        uint8_t line = bmp[row];
        for (uint8_t col = 0; col < 8; col++) {
            uint16_t px = (line & (1U << (7 - col))) ? color : bg;

            /* Масштаб 2x2 */
            uint16_t x0 = x + (uint16_t)col * 2U;
            uint16_t y0 = y + (uint16_t)row * 2U;

            LCD_DrawPixel(x0,     y0,     px);
            LCD_DrawPixel(x0 + 1, y0,     px);
            LCD_DrawPixel(x0,     y0 + 1, px);
            LCD_DrawPixel(x0 + 1, y0 + 1, px);
        }
    }
}

/* Текст большими буквами 16x16 */
static void LCD_DrawTextBig(uint16_t x, uint16_t y,
                            const char *text,
                            uint16_t color, uint16_t bg)
{
    uint16_t cx = x;
    while (*text) {
        LCD_DrawCharBig(cx, y, *text, color, bg);
        cx += 16; // ширина символа
        text++;
    }
}

/* Инициализация контроллера, + поворот на 180 градусов */
static void LCD_Init(void)
{
    /* аппаратный сброс */
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);

    /* выход из сна */
    LCD_WriteCommand(0x11); // SLPOUT
    HAL_Delay(120);

    /* формат пикселя: 16 бит */
    LCD_WriteCommand(0x3A); // COLMOD
    LCD_WriteData(0x05);    // 16-bit

    /* ориентация: поворот на 180° (MADCTL) */
    LCD_WriteCommand(0x36); // MADCTL
    /* MX | MY | BGR */
    LCD_WriteData(0xC8);    // 0xC0 + 0x08 (BGR). Если цвета уйдут, можно попробовать 0xC0.

    /* включить дисплей */
    LCD_WriteCommand(0x29); // DISPON
}

/* ---------------------- АЦП утилита ---------------------- */
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
    return val;
}

/* =========================   main   ========================= */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();

    /* --- Дисплей --- */
    LCD_Init();
    LCD_FillScreen(LCD_COLOR_BLACK);

    /* 8 символов * 16 пикс = 128 => во всю ширину.
       По высоте примерно по центру (160 - 16)/2 ≈ 72 */
    LCD_DrawTextBig(0, 72, "RADIO ON", LCD_COLOR_GREEN, LCD_COLOR_BLACK);

    uint8_t  frame[FRAME_SIZE];

    while (1) {
        /* 1. Считываем и фильтруем 6 потенциометров */
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            uint16_t raw = Read_ADC_Channel(adc_channels[i]);

            adc_filtered[i] = (adc_filtered[i] * 7U + (uint32_t)raw) / 8U;

            uint16_t v = (uint16_t)adc_filtered[i];
            v = (uint16_t)((uint32_t)v & ~0x3U); // шаг 4
            adc_filtered[i] = v;
        }

        /* 2. Формируем кадр */
        frame[0] = 0xAA;
        frame[1] = 0x55;
        frame[2] = NUM_CHANNELS;

        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            uint16_t v = (uint16_t)adc_filtered[i];
            frame[3 + 2U*i]     = (uint8_t)(v >> 8);
            frame[3 + 2U*i + 1] = (uint8_t)(v & 0xFFU);
        }

        uint8_t crc = 0;
        for (uint8_t i = 0; i < FRAME_SIZE - 1U; i++) {
            crc ^= frame[i];
        }
        frame[FRAME_SIZE - 1U] = crc;

        HAL_UART_Transmit(&huart1, frame, FRAME_SIZE, HAL_MAX_DELAY);

        HAL_Delay(20);
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
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  |
                                  RCC_CLOCKTYPE_SYSCLK|
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
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

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
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
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* ---------------------- GPIO init ---------------------- */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB0, PB3, PB5, PB6, PB7 — CS, SCK, SDA, RST, RS для дисплея */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_5 |
                          GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_5 |
                              GPIO_PIN_6 | GPIO_PIN_7,
                      GPIO_PIN_RESET);
}

/* ---------------------- Error handler ---------------------- */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
