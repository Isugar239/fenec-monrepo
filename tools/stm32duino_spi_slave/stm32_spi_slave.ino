/* stm32_spi_slave.ino — SPI1 byte-echo loopback (без UART)

  Для проверки Pi ↔ STM по SPI без осциллографа и без Serial.

  Пины STM32F446 (SPI1):
    PA4 = NSS  (AF5, hardware NSS — обязательно AF, не GPIO input)
    PA5 = SCK  (AF5)
    PA6 = MISO (AF5)
    PA7 = MOSI (AF5)

  Поведение: каждый принятый байт с MOSI сразу кладётся в DR → уходит на MISO
  на следующем байте (стандартный pipeline SPI slave). Перед первым обменом в DR
  записан 0xA0 — совпадает с первым байтом tools/spi_loopback.py.

  Прошивка: Arduino IDE / STM32Duino, плата STM32F446 (как на control board).
  Serial не используется — UART смотреть не нужно.

  Проверка с Pi (robot.service остановлен):
    sudo systemctl stop robot.service
    sudo python3 tools/spi_loopback.py 0 0 1000000
    sudo python3 tools/spi_loopback.py 0 1 1000000
*/

#include <Arduino.h>

void spi1_slave_setup() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  // PA4..PA7 → AF5 (SPI1)
  GPIOA->MODER &= ~((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2)));
  GPIOA->MODER |= ((2UL << (4 * 2)) | (2UL << (5 * 2)) | (2UL << (6 * 2)) |
                   (2UL << (7 * 2)));

  GPIOA->OSPEEDR |= (3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2));
  GPIOA->PUPDR &= ~((3UL << (4 * 2)) | (3UL << (5 * 2)) | (3UL << (6 * 2)) |
                    (3UL << (7 * 2)));

  uint32_t afr = GPIOA->AFR[0];
  afr &= ~((0xFUL << (4 * 4)) | (0xFUL << (5 * 4)) | (0xFUL << (6 * 4)) |
           (0xFUL << (7 * 4)));
  afr |= ((5UL << (4 * 4)) | (5UL << (5 * 4)) | (5UL << (6 * 4)) |
          (5UL << (7 * 4)));
  GPIOA->AFR[0] = afr;

  // Slave, CPOL=0 CPHA=0 (mode 0), hardware NSS (SSM=0), 8-bit
  SPI1->CR1 = 0;
  SPI1->CR2 = SPI_CR2_RXNEIE;
  SPI1->CR1 |= SPI_CR1_SPE;

  // Первый байт на MISO = 0xA0 (как TX[0] в spi_loopback.py)
  SPI1->DR = 0xA0;

  NVIC_EnableIRQ(SPI1_IRQn);
}

extern "C" void SPI1_IRQHandler(void) {
  uint32_t sr = SPI1->SR;

  if (sr & SPI_SR_RXNE) {
    uint8_t b = SPI1->DR & 0xFF;
    SPI1->DR = b;
  }

  if (sr & SPI_SR_OVR) {
    (void)SPI1->DR;
  }
}

void setup() {
  spi1_slave_setup();
}

void loop() {
  if (SPI1->SR & SPI_SR_OVR) {
    (void)SPI1->DR;
  }
}
