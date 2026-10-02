/* stm32_led_blink_pc5.ino
 *
 * Тест «STM жива»: светодиод на PC5.
 * На control board Pupper / SPIneV1 LED сидит именно на PC5 (output push-pull,
 * активный HIGH). Другого штатного LED на MCU нет.
 *
 * Arduino IDE:
 *   Плата: Generic STM32F4 series
 *   Board part number: Generic F446RETx  (STM32F446RE)
 *   Upload method: STM32CubeProgrammer (SWD) / ST-Link
 *   USB support: None
 *   После заливки: BOOT0 = LOW, снять ST-Link, power-cycle.
 *
 * Тактирование: только внутренний HSI 16 МГц, без HSE.
 * Если кварц плохой — скетч всё равно стартует.
 *
 * Поведение:
 *   3 быстрые вспышки при старте, дальше мигание 1 Гц.
 *   Если мигает — MCU загрузилась и PC5 драйвится.
 */

#include <Arduino.h>

#ifndef LED_PIN
#define LED_PIN PC5
#endif

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

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (int i = 0; i < 3; ++i) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  delay(400);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
