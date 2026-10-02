/* stm32_spi_rx_led.ino — HSE как у SPIne + SPI echo + LED при приёме

  Тест control board, не прошивка ног.

  1) Такт от внешнего кварца 8 МГц (PH0/PH1) → PLL 168 МГц.
     (168 МГц без OverDrive. PLLR=2 обязателен на F446, иначе PLL не
     лочится и LED молчит.)
     HSI-fallback нет.
     Медленные вспышки пачками: 1=HSE, 3=PLL, 4=переключение SYSCLK.
     3 быстрые вспышки после reset = кварц и PLL живы.

  2) SPI1 slave echo, mode 0, 8 bit, hardware NSS (как stm32_spi_slave.ino):
     PA4 NSS, PA5 SCK, PA6 MISO, PA7 MOSI (AF5)
     Первый байт на MISO = 0xA0; каждый принятый байт уходит обратно
     со сдвигом на 1 (pipeline). Проверка: tools/spi_loopback.py

  3) LED PC5 активный HIGH: после старта гаснет; пока идёт SPI — мигает ~8 Гц.

  Arduino IDE:
    Generic STM32F4 / Generic F446RETx
    Upload: STM32CubeProgrammer (SWD), USB support: None
    После заливки: BOOT0 = LOW, снять ST-Link, power-cycle.

  Проверка:
    sudo systemctl stop robot.service
    sudo python3 tools/spi_loopback.py 0 0 1000000
    sudo python3 tools/spi_loopback.py 0 1 1000000
*/

#include <Arduino.h>

static const uint32_t LED_BIT = 5;
static const uint32_t ACTIVITY_HOLD_MS = 200;
static const uint32_t BLINK_HALF_MS = 62;

static volatile uint32_t rx_flag = 0;
static uint32_t last_rx_ms = 0;

static void led_pc5_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  __DSB();
  (void)RCC->AHB1ENR;

  GPIOC->MODER &= ~(3UL << (LED_BIT * 2));
  GPIOC->MODER |= (1UL << (LED_BIT * 2));
  GPIOC->OTYPER &= ~(1UL << LED_BIT);
  GPIOC->OSPEEDR &= ~(3UL << (LED_BIT * 2));
  GPIOC->PUPDR &= ~(3UL << (LED_BIT * 2));
  GPIOC->BSRR = (1UL << (LED_BIT + 16));
}

static inline void led_pc5_on(void) { GPIOC->BSRR = (1UL << LED_BIT); }
static inline void led_pc5_off(void) { GPIOC->BSRR = (1UL << (LED_BIT + 16)); }

static void busy_wait(uint32_t n) {
  for (volatile uint32_t i = 0; i < n; ++i) {
  }
}

static void hang_clock_error(int pulses) {
  led_pc5_init();
  for (;;) {
    for (int i = 0; i < pulses; ++i) {
      led_pc5_on();
      busy_wait(200000);
      led_pc5_off();
      busy_wait(200000);
    }
    busy_wait(1200000);
  }
}

extern "C" void SystemClock_Config(void) {
  led_pc5_init();
  led_pc5_on();
  busy_wait(100000);
  led_pc5_off();

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  __DSB();
  (void)RCC->APB1ENR;

  PWR->CR |= PWR_CR_VOS;
  busy_wait(10000);

  RCC->CR |= RCC_CR_HSEON;
  uint32_t t = 0;
  while (!(RCC->CR & RCC_CR_HSERDY)) {
    if (++t > 2000000UL) {
      hang_clock_error(1);
    }
  }

  /* HSE 8 MHz / M=4 * N=168 / P=2 = 168 MHz.
     PLLR bits[30:28]=010 (/2) нельзя обнулять на F446 — PLL не лочится. */
  RCC->CR &= ~RCC_CR_PLLON;
  while (RCC->CR & RCC_CR_PLLRDY) {
  }
  RCC->PLLCFGR = (4UL << RCC_PLLCFGR_PLLM_Pos) | (168UL << RCC_PLLCFGR_PLLN_Pos) |
                 (0UL << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE |
                 (7UL << RCC_PLLCFGR_PLLQ_Pos) | (2UL << 28);

  RCC->CR |= RCC_CR_PLLON;
  t = 0;
  while (!(RCC->CR & RCC_CR_PLLRDY)) {
    if (++t > 2000000UL) {
      hang_clock_error(3);
    }
  }

  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

  uint32_t cfgr = RCC->CFGR;
  cfgr &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_SW);
  cfgr |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
  RCC->CFGR = cfgr;

  RCC->CFGR |= RCC_CFGR_SW_PLL;
  t = 0;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    if (++t > 2000000UL) {
      hang_clock_error(4);
    }
  }

  SystemCoreClock = 168000000U;
}

static void spi1_slave_echo_setup(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  __DSB();
  (void)RCC->APB2ENR;

  GPIOA->MODER &= ~((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2)));
  GPIOA->MODER |= ((2UL << (4 * 2)) | (2UL << (5 * 2)) | (2UL << (6 * 2)) |
                   (2UL << (7 * 2)));
  GPIOA->OSPEEDR |= (3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2));
  GPIOA->PUPDR &= ~((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2)));
  GPIOA->OTYPER &= ~((1UL << 4) | (1UL << 5) | (1UL << 6) | (1UL << 7));

  uint32_t afr = GPIOA->AFR[0];
  afr &= ~((0xFUL << (4 * 4)) | (0xFUL << (5 * 4)) | (0xFUL << (6 * 4)) |
           (0xFUL << (7 * 4)));
  afr |= ((5UL << (4 * 4)) | (5UL << (5 * 4)) | (5UL << (6 * 4)) |
          (5UL << (7 * 4)));
  GPIOA->AFR[0] = afr;

  SPI1->CR1 &= ~SPI_CR1_SPE;
  SPI1->CR1 = 0;
  SPI1->CR2 = SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->DR = 0xA0;

  NVIC_EnableIRQ(SPI1_IRQn);
}

extern "C" void SPI1_IRQHandler(void) {
  uint32_t sr = SPI1->SR;
  if (sr & SPI_SR_RXNE) {
    uint8_t b = (uint8_t)SPI1->DR;
    SPI1->DR = b;
    rx_flag = 1;
  }
  if (sr & SPI_SR_OVR) {
    (void)SPI1->DR;
    (void)SPI1->SR;
    rx_flag = 1;
  }
}

void setup() {
  spi1_slave_echo_setup();

  for (int i = 0; i < 3; ++i) {
    led_pc5_on();
    delay(100);
    led_pc5_off();
    delay(100);
  }
  delay(400);
}

void loop() {
  if (rx_flag) {
    rx_flag = 0;
    last_rx_ms = millis();
  }
  if (SPI1->SR & SPI_SR_OVR) {
    (void)SPI1->DR;
    (void)SPI1->SR;
  }

  uint32_t now = millis();
  if ((now - last_rx_ms) > ACTIVITY_HOLD_MS) {
    led_pc5_off();
    return;
  }
  if ((now / BLINK_HALF_MS) & 1U) {
    led_pc5_on();
  } else {
    led_pc5_off();
  }
}
