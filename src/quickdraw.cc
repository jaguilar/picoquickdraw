
#include <FreeRTOSConfig.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "FreeRTOS.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/clocks.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"

#ifndef NDEBUG
#define logf(...) printf(__VA_ARGS__)
#else
#define logf(...)
#endif

struct NextGameParams {
  int num_players;
  int target_ms;
};
NextGameParams next_game_params;
SemaphoreHandle_t next_game_params_mutex;

// adc0 holds number of players
// adc1 holds target ms
void potentiometer_task(void *) {
  static TaskHandle_t potentiometer_task_handle = xTaskGetCurrentTaskHandle();
  next_game_params = {.num_players = 1, .target_ms = 200};
  static constexpr int kAdcFifoIrq = 22;
  static constexpr int kTaskNotificationIndex = 0;
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);

  // We are an ISR, so we need a high priority.
  vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 2);

  portDISABLE_INTERRUPTS();
  // Set our affinity to the current core. This is the same core that will have
  // the IRQ handler, so it makes sense????
  vTaskCoreAffinitySet(nullptr, 1 << portGET_CORE_ID());
  irq_set_exclusive_handler(
      kAdcFifoIrq, +[]() {
        // Each time we are interrupted, notify the current task and clear the
        // irq.
        vTaskNotifyGiveIndexedFromISR(potentiometer_task_handle,
                                      kTaskNotificationIndex, nullptr);
        irq_clear(kAdcFifoIrq);
      });
  portENABLE_INTERRUPTS();

  // We want a sample ever ~5ms per adc, so set the clock to 1 per 2ms
  adc_set_clkdiv(static_cast<float>(clock_get_hz(clk_ref)) / 500);
  adc_set_round_robin(0b11);
  // We will receive an interrupt every time there are two samples in the fifo.
  adc_fifo_setup(true, false, 2, 0, false);
  adc_run(true);

  while (true) {
    // Wait for our task to be notified.
    xTaskNotifyWaitIndexed(kTaskNotificationIndex, 0b1, 0, nullptr,
                           portMAX_DELAY);
    // FIFO contains two values. Read each one.
    const int level = adc_fifo_get_level();
    if (level != 2) {
      [[unlikely]] logf("adc level not as expected: %d\n", level);
      adc_fifo_drain();
      continue;
    }

    const uint16_t num_players_sample = adc_fifo_get();
    const uint16_t target_ms_sample = adc_fifo_get();

    if (xSemaphoreTake(next_game_params_mutex, pdMS_TO_TICKS(1)) != pdTRUE) {
      [[unlikely]] logf("Failed to take next_game_params_mutex\n");
      continue;
    };
    NextGameParams prev = next_game_params;

    constexpr auto kMaxSample = std::numeric_limits<uint16_t>::max();
    next_game_params.num_players =
        1 + static_cast<int>(std::round(3.0 * num_players_sample / kMaxSample));
    next_game_params.target_ms =
        static_cast<int>(std::round(1000.0 * target_ms_sample / kMaxSample));
    if (xSemaphoreGive(next_game_params_mutex) != pdTRUE) {
      [[unlikely]] logf("Failed to give next_game_params_mutex\n");
    }

    if (prev.num_players != next_game_params.num_players ||
        prev.target_ms != next_game_params.target_ms) {
      logf("change: num_players: %d, target_ms: %d\n",
           next_game_params.num_players, next_game_params.target_ms);
    }
  }
}

extern "C" void main_task(void *) {
  xTaskCreate(potentiometer_task, "potentiometer_task", 512, nullptr, 0,
              nullptr);
  vTaskDelay(portMAX_DELAY);
}
