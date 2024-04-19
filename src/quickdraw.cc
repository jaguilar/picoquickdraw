
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
#include "jagspico/cd74hc595.h"
#include "jagspico/disp4digit.h"
#include "pico/platform.h"
#include "pico/time.h"
#include "pico/types.h"
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

void display_task(void *) {
  SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
  uint16_t to_show = 0;
  uint8_t decimal_pos = 0;
  jagspico::Disp4Digit display{
      {.digit_driver = *jagspico::Cd74Hc595DriverPio::Create(
           {.pio = pio0, .pin_srclk = 19, .pin_ser = 21}),
       .pin_select = 10,
       .get_content_callback = [&]() {
         if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1)) != pdTRUE) {
           panic("Failed to take mutex after 1ms\n");
         }
         jagspico::Disp4Digit::DisplayValue v{to_show, decimal_pos};
         xSemaphoreGive(mutex);
         return v;
       }}};

  absolute_time_t start = get_absolute_time();
  constexpr int kResetTime = 125000;
  while (true) {
    absolute_time_t now = get_absolute_time();
    int32_t elapsed_ms =
        static_cast<int32_t>(absolute_time_diff_us(start, now) / 1000);
    elapsed_ms %= kResetTime;  // Every 100s the counter will reset.

    uint16_t local_to_show;
    uint8_t local_decimal_pos;
    if (elapsed_ms < 10000) {
      local_to_show = elapsed_ms;
      local_decimal_pos = 3;
    } else if (elapsed_ms < 100000) {
      local_to_show = elapsed_ms / 10;
      local_decimal_pos = 2;
    } else if (elapsed_ms < 1000000) {
      local_to_show = elapsed_ms / 100;
      local_decimal_pos = 1;
    } else {
      local_to_show = (elapsed_ms / 1000) % 10000;
      local_decimal_pos = 0;
    }

    // Compute the position of the decimal. If we're less than 10s, it's 3
    // (after the MSB) If we're less than 100s, it's 2. If we're less than 1000,
    // it's 1.

    xSemaphoreTake(mutex, portMAX_DELAY);
    to_show = local_to_show;
    decimal_pos = local_decimal_pos;
    xSemaphoreGive(mutex);
    vTaskDelay(pdMS_TO_TICKS(8));  // Approx 120hz
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
  xTaskCreate(display_task, "display_task", 512, nullptr, 0, nullptr);
  vTaskDelay(portMAX_DELAY);
}
