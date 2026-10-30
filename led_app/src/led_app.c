/*
 * Лампочка (светодиод) на разъёме расширения Crazyflie.
 *
 * Подключение (левый разъём, считая от пина 1 у квадратной площадки):
 *   пин 9  = IO_4 (PC12) -> через резистор 100-220 Ом -> длинная ножка светодиода (+)
 *   пин 10 = GND         -> короткая ножка светодиода (-)
 *
 * Включается параметром extled.on (0 = выкл, 1 = вкл), его меняет led_space.py.
 */

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "deck.h"

#define LED_PIN DECK_GPIO_IO4

static uint8_t ledOn = 0;

void appMain()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  uint8_t shown = 0;
  while (1) {
    vTaskDelay(M2T(20));
    if (ledOn != shown) {
      shown = ledOn;
      digitalWrite(LED_PIN, shown ? HIGH : LOW);
    }
  }
}

PARAM_GROUP_START(extled)
PARAM_ADD(PARAM_UINT8, on, &ledOn)
PARAM_GROUP_STOP(extled)
