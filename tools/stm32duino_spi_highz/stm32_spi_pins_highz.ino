/* stm32_spi_pins_highz.ino — SPI-пины в high-Z, периферия SPI1 выключена

  Диагностика общей шины MISO: если вторая STM тянет линию (~0.8 V вместо 3.3 V
  или 0 V), прошейте ЭТУ прошивку на подозреваемую плату и снова смотрите MISO
  с Pi / осциллографом. При успехе линия должна вести себя как «пустая» (подтяжка
  Pi или вторая плата), без полупериода от этой STM.

  Что делает прошивка:
    - SPI1 полностью выключен (SPE=0), такт SPI1 снят
    - PA4 (NSS), PA5 (SCK), PA6 (MISO), PA7 (MOSI) → режим Analog (high-Z), без подтяжек
    - AFR для PA4..PA7 сброшен (не alternate function)
    - Никаких ISR SPI, никаких записей в DR

  Единственный активный пин: PC5 — LED «MCU жива» (как в stm32_led_blink_pc5.ino).
  На шину SPI эта STM не влияет.

  Прошивка: Arduino IDE / STM32Duino, STM32F446 (control board).
  После заливки: BOOT0 = LOW, power-cycle.

  Проверка с Pi (robot.service остановлен):
    sudo systemctl stop robot.service
    sudo python3 tools/spi_loopback.py 0 0 1000000
    sudo python3 tools/spi_loopback.py 0 1 1000000

  Интерпретация:
    - MISO на этой плате ровный 3.3 V (или чистый ответ другой STM) → эта STM
      больше не мешает; искать проблему на другой плате / в кабеле
    - MISO всё ещё «холмики» ~0.8 V при обмене только с CE другой платы →
      скорее кабель / первая STM / подтяжки на Pi
*/

#include <Arduino.h>

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

static const uint32_t LED_PIN = 5;  // PC5
static const uint32_t BLINK_MS = 500;

static void led_pc5_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  GPIOC->MODER &= ~(3UL << (LED_PIN * 2));
  GPIOC->MODER |= (1UL << (LED_PIN * 2));
  GPIOC->OTYPER &= ~(1UL << LED_PIN);
  GPIOC->OSPEEDR &= ~(3UL << (LED_PIN * 2));
  GPIOC->PUPDR &= ~(3UL << (LED_PIN * 2));

  GPIOC->BSRR = (1UL << (LED_PIN + 16));
}

static inline void led_pc5_on(void) {
  GPIOC->BSRR = (1UL << LED_PIN);
}

static inline void led_pc5_off(void) {
  GPIOC->BSRR = (1UL << (LED_PIN + 16));
}

static void spi1_shutdown(void) {
  NVIC_DisableIRQ(SPI1_IRQn);

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  SPI1->CR1 &= ~SPI_CR1_SPE;
  SPI1->CR1 = 0;
  SPI1->CR2 = 0;
  (void)SPI1->DR;
  (void)SPI1->SR;
  RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
}

static void spi_pins_park_highz(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  spi1_shutdown();

  // Сброс alternate function PA4..PA7
  uint32_t afr = GPIOA->AFR[0];
  afr &= ~((0xFUL << (4 * 4)) | (0xFUL << (5 * 4)) | (0xFUL << (6 * 4)) |
           (0xFUL << (7 * 4)));
  GPIOA->AFR[0] = afr;

  // Analog = high-Z, без подтяжек (MODER=11, PUPDR=00)
  GPIOA->MODER &= ~((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2)));
  GPIOA->MODER |= ((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                   (3UL << (7 * 2)));

  GPIOA->PUPDR &= ~((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2)));

  GPIOA->OTYPER &= ~((1UL << 4) | (1UL << 5) | (1UL << 6) | (1UL << 7));
  GPIOA->OSPEEDR &= ~((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                      (3UL << (7 * 2)));
}

void setup() {
  spi_pins_park_highz();
  led_pc5_init();

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
