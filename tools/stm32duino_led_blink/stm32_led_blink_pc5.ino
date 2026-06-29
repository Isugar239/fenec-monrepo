/* stm32_led_blink_pc5.ino — мигание внешним LED на PC5

  Минимальная проверка «STM жива»: если LED мигает ~1 Гц — MCU загрузилась,
  тактирование работает, PC5 драйвится.

  Пин: PC5 = GPIO output, push-pull.
  Период: 500 мс вкл / 500 мс выкл (1 Гц).

  Прошивка: Arduino IDE / STM32Duino, плата STM32F446 (как на control board).
  Serial не нужен.

  Важно: SystemClock_Config() ниже принудительно использует внутренний HSI (16 МГц),
  без ожидания внешнего кварца HSE. Так тест отвечает на вопрос «MCU вообще жива?»
  даже если HSE/кварц не стартует.

  После прошивки: BOOT0 = LOW, power-cycle.

  Если не мигает:
    - проверить полярность LED и резистор
    - 3.3 V на плате, BOOT0 = LOW
    - правильная плата в Arduino (STM32F446, тот же чип что на control board)
*/

#include <Arduino.h>

// Переопределяем тактирование ядра STM32duino: только HSI, без HSE.
extern "C" void SystemClock_Config(void) {
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY)) {
  }

  RCC->CFGR &= ~RCC_CFGR_SW;
  while ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) {
  }

  RCC->CFGR |= RCC_CFGR_SW_HSI;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
  }

  RCC->CR &= ~RCC_CR_HSEON;
  RCC->CR &= ~RCC_CR_PLLON;

  SystemCoreClock = 16000000U;
}

static const uint32_t LED_PIN = 5;       // PC5
static const uint32_t BLINK_MS = 500;

static void led_pc5_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  GPIOC->MODER &= ~(3UL << (LED_PIN * 2));
  GPIOC->MODER |= (1UL << (LED_PIN * 2));   // output

  GPIOC->OTYPER &= ~(1UL << LED_PIN);         // push-pull
  GPIOC->OSPEEDR &= ~(3UL << (LED_PIN * 2));  // low speed — для LED достаточно
  GPIOC->PUPDR &= ~(3UL << (LED_PIN * 2));    // без подтяжки

  GPIOC->BSRR = (1UL << (LED_PIN + 16));      // старт с выключенным LED
}

static inline void led_pc5_on(void) {
  GPIOC->BSRR = (1UL << LED_PIN);
}

static inline void led_pc5_off(void) {
  GPIOC->BSRR = (1UL << (LED_PIN + 16));
}

void setup() {
  led_pc5_init();

  // Три быстрых вспышки при старте — видно, что setup() дошёл.
  for (int i = 0; i < 3; ++i) {
    led_pc5_on();
    delay(100);
    led_pc5_off();
    delay(100);
  }
  delay(400);
}

void loop() {
  led_pc5_on();
  delay(BLINK_MS);
  led_pc5_off();
  delay(BLINK_MS);
}
