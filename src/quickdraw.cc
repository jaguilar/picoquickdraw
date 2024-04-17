
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
#include "hardware/regs/intctrl.h"
#include "hardware/structs/clocks.h"
#include "pico/platform.h"
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

extern "C" void adc_irq_handler();
static int irq_called_times = 0;

static TaskHandle_t potentiometer_task_handle;
static constexpr int kAdcTaskNotificationIndex = 0;

// adc0 holds number of players
// adc1 holds target ms
void potentiometer_task(void *) {
  potentiometer_task_handle = xTaskGetCurrentTaskHandle();
  next_game_params = {.num_players = 1, .target_ms = 200};
  static constexpr int kAdcFifoIrq = 22;
  logf("init adc\n");
  adc_init();
  adc_gpio_init(26);
  adc_gpio_init(27);

  // We are an ISR, so we need a high priority.
  vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 2);

  portDISABLE_INTERRUPTS();
  // Set our affinity to the current core. This is the same core that will have
  // the IRQ handler, so it makes sense????
  vTaskCoreAffinitySet(nullptr, 1 << portGET_CORE_ID());
  portENABLE_INTERRUPTS();
  irq_add_shared_handler(ADC_IRQ_FIFO, adc_irq_handler, 128);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  irq_set_priority(ADC_IRQ_FIFO, 0);

  adc_select_input(0);
  adc_set_round_robin(0b11);

  // We want a sample ever ~5ms per adc, so set the clock to 1 per 2ms
  const uint32_t clock_hz = clock_get_hz(clk_adc);
  const uint32_t target_hz = 500;
  const float divider = 1.0f * clock_hz / target_hz;
  logf("setting divider to %f\n", divider);
  adc_set_clkdiv(divider);
  // We will receive an interrupt every time there are two samples in the fifo.
  adc_fifo_setup(true, false, 2, 0, false);
  adc_fifo_drain();
  adc_run(true);
  logf("adc running\n");

  for (int i = 0; i < 5; ++i) {
    // Wait for our task to be notified.
    logf("waiting for notification\n");
    if (xTaskNotifyWaitIndexed(kAdcTaskNotificationIndex, 0b1, 0, nullptr,
                               pdMS_TO_TICKS(200)) != pdPASS) {
      [[unlikely]] logf("failed to wait for notification\n");
    }
    // FIFO contains two values. Read each one.
    const uint8_t level = adc_fifo_get_level();
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
  logf("irq called times: %d\n", irq_called_times);
  vTaskDelay(portMAX_DELAY);
}

extern "C" void __time_critical_func(adc_irq_handler)() {
  // Each time we are interrupted, notify the current task and clear the
  // irq.
  irq_called_times++;
  vTaskNotifyGiveIndexedFromISR(potentiometer_task_handle,
                                kAdcTaskNotificationIndex, nullptr);
  irq_clear(ADC_IRQ_FIFO);
}

extern "C" void main_task(void *) {
  xTaskCreate(potentiometer_task, "potentiometer_task", 512, nullptr, 0,
              nullptr);
  vTaskDelay(portMAX_DELAY);
}
