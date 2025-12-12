/* main.c — ПРИЁМНИК: принимает 6 каналов по UART1 и двигает 6 серв.
 * Лампа связи на PA6 мигает 1 Гц, если сигнал есть.
 * Плавный ход серв + индивидуальная калибровка для КАЖДОЙ сервы.
 */

#include "main.h"

/* ================== НАСТРОЙКИ СЕРВОПРИВОДОВ ==================
 *
 * БАЗОВЫЕ ЗНАЧЕНИЯ (общие для всех серво по умолчанию):
 * Можно оставить так:
 *   500..2500 мкс — максимально широкий ход
 * Или, если хочешь щадящий режим:
 *   1000..2000 мкс
 */

#define SERVO_GLOBAL_MIN_US   1000U
#define SERVO_GLOBAL_MAX_US   2000U

/* ДАЛЬШЕ — ИНДИВИДУАЛЬНАЯ НАСТРОЙКА ДЛЯ КАЖДОЙ СЕРВЫ
 *
 * Если серва ведёт себя странно — уменьшаешь диапазон.
 * Если серва крутится "наоборот" — просто меняешь местами MIN и MAX.
 *
 * Соответствие:
 *  Servo1: Pot1 (PA0) -> PA7  (TIM3_CH2)
 *  Servo2: Pot2 (PA1) -> PB0  (TIM3_CH3)
 *  Servo3: Pot3 (PA2) -> PB1  (TIM3_CH4)
 *  Servo4: Pot4 (PA3) -> PB6  (TIM4_CH1)
 *  Servo5: Pot5 (PA4) -> PB7  (TIM4_CH2)
 *  Servo6: Pot6 (PA5) -> PB9  (TIM4_CH4)
 */

/* ===== Тут ты потом просто меняешь цифры под каждую серву ===== */

#define SERVO1_MIN_US   SERVO_GLOBAL_MIN_US
#define SERVO1_MAX_US   SERVO_GLOBAL_MAX_US

#define SERVO2_MIN_US   SERVO_GLOBAL_MIN_US
#define SERVO2_MAX_US   SERVO_GLOBAL_MAX_US

#define SERVO3_MIN_US   SERVO_GLOBAL_MIN_US
#define SERVO3_MAX_US   SERVO_GLOBAL_MAX_US

#define SERVO4_MIN_US   SERVO_GLOBAL_MIN_US
#define SERVO4_MAX_US   SERVO_GLOBAL_MAX_US

#define SERVO5_MIN_US   SERVO_GLOBAL_MIN_US
#define SERVO5_MAX_US   SERVO_GLOBAL_MAX_US

#define SERVO6_MIN_US   SERVO_GLOBAL_MIN_US
#define SERVO6_MAX_US   SERVO_GLOBAL_MAX_US

/* Примеры:
 *  - Ограничить ход:       #define SERVO1_MIN_US 900  / SERVO1_MAX_US 2100
 *  - Инвертировать ход:    #define SERVO3_MIN_US 2500 / SERVO3_MAX_US 500
 */

/* ============================================================= */

UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim3;
TIM_HandleTypeDef  htim4;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* --------- Протокол / параметры серв --------- */

#define NUM_CHANNELS        6U
#define FRAME_SIZE          (3U + NUM_CHANNELS*2U + 1U)  // 3 байта + данные + CRC

#define SERVO_SMOOTH_STEP_US   50U    // шаг сглаживания (мкс) за один цикл
#define LINK_TIMEOUT_MS        500U   // если 0.5 сек нет валидных кадров — связи нет
#define BLINK_HALF_PERIOD      500U   // смена состояния LED каждые 500 мс → мигание 1 Гц

typedef struct
{
    uint16_t min_us;
    uint16_t max_us;
} ServoConfig;

/* Массив конфигов — ЗДЕСЬ УЖЕ ПОДСТАВЛЯЮТСЯ ТВОИ МАКРОСЫ */
static const ServoConfig servo_config[NUM_CHANNELS] = {
    /* Servo1 (PA7 / TIM3_CH2)  <- Pot1 (PA0) */
    { SERVO1_MIN_US, SERVO1_MAX_US },

    /* Servo2 (PB0 / TIM3_CH3)  <- Pot2 (PA1) */
    { SERVO2_MIN_US, SERVO2_MAX_US },

    /* Servo3 (PB1 / TIM3_CH4)  <- Pot3 (PA2) */
    { SERVO3_MIN_US, SERVO3_MAX_US },

    /* Servo4 (PB6 / TIM4_CH1)  <- Pot4 (PA3) */
    { SERVO4_MIN_US, SERVO4_MAX_US },

    /* Servo5 (PB7 / TIM4_CH2)  <- Pot5 (PA4) */
    { SERVO5_MIN_US, SERVO5_MAX_US },

    /* Servo6 (PB9 / TIM4_CH4)  <- Pot6 (PA5) */
    { SERVO6_MIN_US, SERVO6_MAX_US }
};

/* Целевые и текущие значения ширины импульсов по 6 каналам */
static uint16_t servo_target_us[NUM_CHANNELS] = {
    SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US,
    SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US
};

static uint16_t servo_current_us[NUM_CHANNELS] = {
    SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US,
    SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US, SERVO_GLOBAL_MIN_US
};

/* Преобразование принятого кадра в целевые ширины импульсов серв.
 * Возвращает 1, если кадр валидный, 0 — если мусор.
 * Использует servo_config[i].min_us / max_us.
 */
static uint8_t ParseFrame(uint8_t *buf, uint16_t *servo_out)
{
    /* Проверяем заголовок и количество каналов */
    if (buf[0] != 0xAA || buf[1] != 0x55 || buf[2] != NUM_CHANNELS)
        return 0U;

    /* Проверяем CRC XOR */
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < FRAME_SIZE - 1U; i++)
        crc ^= buf[i];

    if (crc != buf[FRAME_SIZE - 1U])
        return 0U;

    /* Разбираем 6 значений АЦП и переводим в мкс по индивидуальной калибровке */
    for (uint8_t i = 0U; i < NUM_CHANNELS; i++)
    {
        uint16_t adc =
            ((uint16_t)buf[3U + 2U*i] << 8) |
            (uint16_t)buf[3U + 2U*i + 1U];

        if (adc > 4095U)
            adc = 4095U;

        uint16_t cfg_min = servo_config[i].min_us;
        uint16_t cfg_max = servo_config[i].max_us;

        uint16_t lo, hi;
        uint8_t  reversed;

        if (cfg_max >= cfg_min)
        {
            lo = cfg_min;
            hi = cfg_max;
            reversed = 0U;
        }
        else
        {
            lo = cfg_max;
            hi = cfg_min;
            reversed = 1U; // пользователь задал max < min => инверсия
        }

        uint32_t span = (uint32_t)hi - (uint32_t)lo;
        uint32_t pulse;

        if (span == 0U)
        {
            pulse = lo;
        }
        else if (!reversed)
        {
            /* обычный случай: adc=0 -> lo, adc=4095 -> hi */
            pulse = lo + (uint32_t)adc * span / 4095U;
        }
        else
        {
            /* инверсия: adc=0 -> hi, adc=4095 -> lo */
            pulse = hi - (uint32_t)adc * span / 4095U;
        }

        if (pulse < lo) pulse = lo;
        if (pulse > hi) pulse = hi;

        servo_out[i] = (uint16_t)pulse;
    }

    return 1U;
}

/* Обновление ШИМа на таймерах под 6 серв */
static void UpdateServos(const uint16_t *servo)
{
    // 0 -> PA7 / TIM3_CH2
    // 1 -> PB0 / TIM3_CH3
    // 2 -> PB1 / TIM3_CH4
    // 3 -> PB6 / TIM4_CH1
    // 4 -> PB7 / TIM4_CH2
    // 5 -> PB9 / TIM4_CH4

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, servo[1]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, servo[2]);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, servo[3]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, servo[4]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, servo[5]);
}

/* Плавное приближение текущих значений к целевым */
static void SmoothServos(void)
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        uint16_t cur = servo_current_us[i];
        uint16_t tgt = servo_target_us[i];

        if (cur < tgt)
        {
            uint16_t diff = tgt - cur;
            if (diff > SERVO_SMOOTH_STEP_US)
                diff = SERVO_SMOOTH_STEP_US;
            cur += diff;
        }
        else if (cur > tgt)
        {
            uint16_t diff = cur - tgt;
            if (diff > SERVO_SMOOTH_STEP_US)
                diff = SERVO_SMOOTH_STEP_US;
            cur -= diff;
        }

        servo_current_us[i] = cur;
    }

    UpdateServos(servo_current_us);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    /* Запускаем PWM на всех каналах */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    uint8_t  frame[FRAME_SIZE];
    uint32_t last_rx_ok_time   = 0;
    uint32_t last_blink_toggle = 0;
    uint8_t  led_state         = 0;

    UpdateServos(servo_current_us);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

    while (1)
    {
        /* Пытаемся принять один кадр (FRAME_SIZE байт) с таймаутом 100 мс */
        if (HAL_UART_Receive(&huart1, frame, FRAME_SIZE, 100) == HAL_OK)
        {
            if (ParseFrame(frame, servo_target_us))
            {
                last_rx_ok_time = HAL_GetTick();
            }
        }

        /* Плавно тянем сервы к целям */
        SmoothServos();

        /* Логика лампы связи */
        uint32_t now = HAL_GetTick();

        if (now - last_rx_ok_time <= LINK_TIMEOUT_MS)
        {
            /* Связь есть — мигаем LED на PA6 с частотой 1 Гц */
            if (now - last_blink_toggle >= BLINK_HALF_PERIOD)
            {
                led_state ^= 1U;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
                                  led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
                last_blink_toggle = now;
            }
        }
        else
        {
            /* Связь давно пропала — гасим лампу */
            led_state = 0;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        }
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

/* ---------------------- TIM3 init (PA7, PB0, PB1) ---------------------- */

static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance           = TIM3;
    htim3.Init.Prescaler     = 83;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = 19999;  // 20 мс при 1 МГц -> 50 Гц
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = SERVO_GLOBAL_MIN_US;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim3);
}

/* ---------------------- TIM4 init (PB6, PB7, PB9) ---------------------- */

static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance           = TIM4;
    htim4.Init.Prescaler     = 83;
    htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim4.Init.Period        = 19999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = SERVO_GLOBAL_MIN_US;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();

    HAL_TIM_MspPostInit(&htim4);
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

/* ---------------------- GPIO init (PA6 — лампа связи) ---------------------- */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6 — лампа связи */
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
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
