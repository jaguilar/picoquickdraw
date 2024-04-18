
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
#include "hardware/regs/adc.h"
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

namespace {
struct SampleStruct {
  uint16_t num_players;
  uint16_t target_ms;
};
QueueHandle_t sample_queue;
struct NextGameParams {
  int num_players;
  int target_ms;
};
NextGameParams next_game_params;
SemaphoreHandle_t next_game_params_mutex;
int irq_called_times = 0;
int failed_isr_sends = 0;
TaskHandle_t potentiometer_task_handle;
}  // namespace

extern "C" void __time_critical_func(my_adc_irq_handler)() {
  // Each time we are interrupted, notify the current task and clear the
  // irq.
  irq_called_times++;
  SampleStruct s;
  s.num_players = adc_fifo_get();
  s.target_ms = adc_fifo_get();
  assert(adc_fifo_get_level() == 0);
  if (xQueueSendFromISR(sample_queue, &s, nullptr) != pdTRUE) {
    ++failed_isr_sends;
  }
}

// adc0 holds number of players
// adc1 holds target ms
void potentiometer_task(void *) {
  potentiometer_task_handle = xTaskGetCurrentTaskHandle();
  sample_queue = xQueueCreate(1, sizeof(SampleStruct));
  next_game_params_mutex = xSemaphoreCreateMutex();
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
  irq_add_shared_handler(ADC_IRQ_FIFO, my_adc_irq_handler,
                         PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  irq_set_priority(ADC_IRQ_FIFO, 0);
  portENABLE_INTERRUPTS();

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
  logf("enabling adc irq\n");
  adc_irq_set_enabled(true);
  logf("adc irq enabled\n");
  adc_run(true);
  logf("adc running\n");

  for (int i = 0; true; ++i) {
    // Wait for our task to be notified.
    struct SampleStruct samples;
    if (xQueueReceive(sample_queue, &samples, pdMS_TO_TICKS(200)) != pdPASS) {
      [[unlikely]] logf("failed to wait for notification\n");
      continue;
    }

    if (xSemaphoreTake(next_game_params_mutex, pdMS_TO_TICKS(1)) != pdTRUE) {
      [[unlikely]] logf("Failed to take next_game_params_mutex\n");
      continue;
    };
    NextGameParams prev = next_game_params;

    next_game_params.num_players =
        1 + static_cast<int>(
                std::round(3.0 * samples.num_players / ADC_FIFO_VAL_BITS));
    // target_ms rounded to nearest 25 to reduce jitter.
    next_game_params.target_ms =
        static_cast<int>(
            std::round(1000 / 25 * samples.target_ms / ADC_FIFO_VAL_BITS)) *
        25;
    if (xSemaphoreGive(next_game_params_mutex) != pdTRUE) {
      [[unlikely]] logf("Failed to give next_game_params_mutex\n");
    }

    if (prev.num_players != next_game_params.num_players ||
        prev.target_ms != next_game_params.target_ms) {
      logf("change: num_players: %d, target_ms: %d samples: %d %d\n",
           next_game_params.num_players, next_game_params.target_ms,
           samples.num_players, samples.target_ms);
    }
  }
  adc_run(false);
  adc_irq_set_enabled(false);
  logf("irq called times: %d failed_isr_sends: %d\n", irq_called_times,
       failed_isr_sends);
  vTaskDelay(portMAX_DELAY);
}

extern "C" void main_task(void *) {
  xTaskCreate(potentiometer_task, "potentiometer_task", 512, nullptr, 0,
              nullptr);
  vTaskDelay(portMAX_DELAY);
}
