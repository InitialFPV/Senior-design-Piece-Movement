#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include <PinDefs.h>
#include <cstring>
#include <motionPos.h>
#include <cmath>
#include <esp_log.h>


// UART configuration
#define EX_UART_NUM UART_NUM_0

//--------------------------------------------
// CoreXY Gantry Configuration
//--------------------------------------------

const float PULLEY_TEETH   = 20.0;      // teeth on pulley
const float BELT_PITCH_MM  = 2.0;       // mm per tooth
const float STEP_ANGLE_DEG = 1.8;       // motor full-step angle
const float MICROSTEP      = 1;      // microstepping factor

// Derived parameters
const float STEPS_PER_REV  = 360.0 / STEP_ANGLE_DEG;
const float DIST_PER_REV_MM = PULLEY_TEETH * BELT_PITCH_MM;
const float STEPS_PER_MM    = (STEPS_PER_REV * MICROSTEP) / DIST_PER_REV_MM;

// void stepMotor(int motor, int direction);
// void updateMotion();
// void moveTo(float x_target_mm, float y_target_mm);
bool moveLinearToPWM(float x_target_mm, float y_target_mm, float speed_mm_s, uint8_t duty_percent);
// static void update_motion_timer_cb(void* arg);
void setDirectionPin(int motor, int direction);

// --- limited-pulse PWM globals (support up to 2 concurrent PWMs) ---
typedef struct {
  bool in_use;
  gpio_num_t pin;
  ledc_channel_t channel;
  ledc_timer_t timer;
  volatile uint32_t pulse_count;
  uint32_t pulse_target;
  TaskHandle_t watcher_task;
  int motor_id;   // 1 -> A, 2 -> B, 0 -> unknown
  int direction;  // 1 -> increment, 0 -> decrement
} pwm_handle_t;

static pwm_handle_t s_pwms[2] = {0};
static bool s_gpio_isr_installed = false;

// ISR for counting rising edges on the PWM output pin.
// arg is the index (0..1) into s_pwms.
static void IRAM_ATTR pwm_gpio_isr(void* arg) {
  int idx = (int)(uintptr_t)arg;
  if (idx < 0 || idx >= (int)(sizeof(s_pwms)/sizeof(s_pwms[0]))) return;
  pwm_handle_t* h = &s_pwms[idx];
  if (!h->in_use) return;

  // increment count
  h->pulse_count++;

  // update motor position immediately on each pulse
  if (h->motor_id == 1) {
    if (h->direction) motors.A_pos++;
    else motors.A_pos--;
  } else if (h->motor_id == 2) {
    if (h->direction) motors.B_pos++;
    else motors.B_pos--;
  }

  // if we've reached target, disable interrupt for this pin and notify watcher
  if (h->pulse_count >= h->pulse_target) {
    gpio_intr_disable(h->pin);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (h->watcher_task) {
      vTaskNotifyGiveFromISR(h->watcher_task, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }
}

// Watcher task: waits for notification from ISR then stops PWM and cleans up
// pvParameters is the index (int) of the pwm slot.
static void pwm_watcher_task(void* pvParameters) {
  // block until ISR notifies
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  int idx = (int)(uintptr_t)pvParameters;
  if (idx < 0 || idx >= (int)(sizeof(s_pwms)/sizeof(s_pwms[0]))) {
    vTaskDelete(NULL);
    return;
  }
  pwm_handle_t* h = &s_pwms[idx];

  // Stop the LEDC PWM output for this channel
  ledc_set_duty((ledc_mode_t)0, h->channel, 0);
  ledc_update_duty((ledc_mode_t)0, h->channel);

  // Remove ISR handler for this pin
  gpio_isr_handler_remove(h->pin);

  // mark slot free
  h->in_use = false;
  h->watcher_task = NULL;

  // If no other PWM is active, update motion flags
  bool any_active = false;
  for (size_t i = 0; i < sizeof(s_pwms)/sizeof(s_pwms[0]); ++i) {
    if (s_pwms[i].in_use) { any_active = true; break; }
  }
  if (!any_active) {
    gantry.motion_active = false;
    gantry.position_reached = (motors.A_pos == motors.A_target && motors.B_pos == motors.B_target);
  }

  vTaskDelete(NULL);
}

void setDirectionPin(int motor, int direction) {
  gpio_num_t dir_pin = (motor == 1) ? (gpio_num_t)DIR1_PIN : (gpio_num_t)DIR2_PIN;
  gpio_set_level(dir_pin, direction ? 1 : 0);
}

// Start a PWM on the given GPIO pin, emit a specified number of pulses, then stop.
// This function is designed to prevent race conditions by setting up the ISR *before* starting the PWM.
bool send_limited_pwm_pulses(gpio_num_t gpio, uint32_t freq_hz, uint8_t duty_percent, uint32_t pulse_count) {
  if (pulse_count == 0) return false;
  if (duty_percent > 100) duty_percent = 100;

  // Find a free PWM slot
  int slot = -1;
  for (int i = 0; i < 2; ++i) {
    if (!s_pwms[i].in_use) { slot = i; break; }
  }
  if (slot < 0) {
    ESP_LOGE("PWM", "No free PWM slot available.");
    return false;
  }

  // Assign LEDC channel and timer per slot
  ledc_channel_t channel = (ledc_channel_t)(LEDC_CHANNEL_0 + slot);
  ledc_timer_t timer = (ledc_timer_t)(LEDC_TIMER_0 + slot);

  // --- Start of critical reordering ---

  // 1. Stop any previous PWM on this channel to ensure a clean start
  ledc_stop((ledc_mode_t)0, channel, 0);

  // 2. Configure LEDC timer
  const int pwm_resolution_bits = 12; // Using 12-bit for better low-frequency precision
  ledc_timer_config_t ledc_timer_cfg = {
    .speed_mode = (ledc_mode_t)0,
    .duty_resolution = (ledc_timer_bit_t)pwm_resolution_bits,
    .timer_num = timer,
    .freq_hz = freq_hz,
    .clk_cfg = LEDC_USE_RTC8M_CLK
  };
  if (ledc_timer_config(&ledc_timer_cfg) != ESP_OK) {
    ESP_LOGE("PWM", "ledc_timer_config failed.");
    return false;
  }

  // 3. Configure LEDC channel (but don't start it yet)
  ledc_channel_config_t ledc_ch_cfg = {
    .gpio_num = (int)gpio,
    .speed_mode = (ledc_mode_t)0,
    .channel = channel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = timer,
    .duty = 0,
    .hpoint = 0
  };
  if (ledc_channel_config(&ledc_ch_cfg) != ESP_OK) {
    ESP_LOGE("PWM", "ledc_channel_config failed.");
    return false;
  }

  // 4. Ensure GPIO ISR service is installed
  if (!s_gpio_isr_installed) {
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("PWM", "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return false;
    }
    s_gpio_isr_installed = true;
  }

  // 5. Configure GPIO pin for interrupt
  gpio_set_direction(gpio, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_intr_type(gpio, GPIO_INTR_POSEDGE);

  // 6. Initialize slot state
  pwm_handle_t* h = &s_pwms[slot];
  h->in_use = true;
  h->pin = gpio;
  h->channel = channel;
  h->timer = timer;
  h->pulse_count = 0;
  h->pulse_target = pulse_count;
  h->watcher_task = NULL;

  // Infer motor ID and direction
  if (gpio == (gpio_num_t)STEP1_PIN) {
    h->motor_id = 1;
    h->direction = gpio_get_level((gpio_num_t)DIR1_PIN);
  } else if (gpio == (gpio_num_t)STEP2_PIN) {
    h->motor_id = 2;
    h->direction = gpio_get_level((gpio_num_t)DIR2_PIN);
  } else {
    h->motor_id = 0;
    h->direction = 1;
  }

  // 7. Create watcher task which will block until ISR notifies
  BaseType_t res = xTaskCreate(pwm_watcher_task, "pwm_watcher", 2048, (void*)(uintptr_t)slot, tskIDLE_PRIORITY + 2, &h->watcher_task);
  if (res != pdPASS) {
    ESP_LOGE("PWM", "Failed to create watcher task.");
    gpio_set_intr_type(gpio, GPIO_INTR_DISABLE);
    h->in_use = false;
    return false;
  }

  // 8. Add ISR handler for the pin (pass slot index)
  if (gpio_isr_handler_add(gpio, pwm_gpio_isr, (void*)(uintptr_t)slot) != ESP_OK) {
      ESP_LOGE("PWM", "gpio_isr_handler_add failed.");
      vTaskDelete(h->watcher_task);
      h->watcher_task = NULL;
      gpio_set_intr_type(gpio, GPIO_INTR_DISABLE);
      h->in_use = false;
      return false;
  }

  // 9. NOW, finally, start the PWM by setting the duty cycle.
  // This is the last step to ensure the ISR is ready for the first pulse.
  uint32_t duty = (1 << (pwm_resolution_bits - 1)) * duty_percent / 100; // 50% duty for selected resolution
  if (ledc_set_duty((ledc_mode_t)0, channel, duty) != ESP_OK) {
    ESP_LOGE("PWM", "ledc_set_duty failed.");
    gpio_isr_handler_remove(gpio);
    vTaskDelete(h->watcher_task);
    h->watcher_task = NULL;
    h->in_use = false;
    return false;
  }
  if (ledc_update_duty((ledc_mode_t)0, channel) != ESP_OK) {
    ESP_LOGE("PWM", "ledc_update_duty failed.");
    gpio_isr_handler_remove(gpio);
    vTaskDelete(h->watcher_task);
    h->watcher_task = NULL;
    h->in_use = false;
    return false;
  }

  // --- End of critical reordering ---

  return true;
}


// void uart_init()
// {
//   const uart_config_t uart_config = {
//     .baud_rate = 9600,
//     .data_bits = UART_DATA_8_BITS,
//     .parity    = UART_PARITY_DISABLE,
//     .stop_bits = UART_STOP_BITS_1,
//     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
//   };
//   uart_driver_install(EX_UART_NUM, 256, 0, 0, NULL, 0);
//   uart_param_config(EX_UART_NUM, &uart_config);
//   uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

void gpio_output_init(gpio_num_t pin)
{
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << pin),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf);
}

void digitalWrite_esp(gpio_num_t pin, int level)
{
  gpio_set_level(pin, level);
}

void delay_esp(int ms)
{
  // vTaskDelay(ms / portTICK_PERIOD_MS);
  vTaskDelay(pdMS_TO_TICKS(ms));
}


extern "C" void app_main(void)
{
    delay_esp(2000);  // Wait for system to stabilize
    ESP_LOGI("init", "im starting");

    gpio_reset_pin(SLEEP_PIN);
    gpio_output_init(SLEEP_PIN);
    gpio_output_init(EN_PIN);
    gpio_output_init(MODE0_PIN);
    gpio_output_init(MODE1_PIN);
    gpio_output_init(DIR1_PIN);
    gpio_output_init(MODE2_PIN);
    gpio_output_init(HFS_PIN);

    digitalWrite_esp(SLEEP_PIN, 1);
    digitalWrite_esp(EN_PIN, 1);
    digitalWrite_esp(MODE0_PIN, 0);
    digitalWrite_esp(MODE1_PIN, 0);
    digitalWrite_esp(DIR1_PIN, 0);
    digitalWrite_esp(MODE2_PIN, 0);
    digitalWrite_esp(HFS_PIN, 1);

    setupMotion();  

  // // Create a periodic esp_timer to call updateMotion every 100 microseconds
  // static esp_timer_handle_t update_timer = NULL;
  // static esp_timer_create_args_t update_timer_args = {
  //   .callback = &update_motion_timer_cb,
  //   .arg = NULL,
  //   .name = "update_motion_timer",
  //   .skip_unhandled_events = false
  // };

  // // Define the callback function (calls the C++ updateMotion)
  // // Must be declared static with C linkage for the function pointer type
  // // (defined below as a local static forward declaration)
  // if (esp_timer_create(&update_timer_args, &update_timer) == ESP_OK) {
  //   // Start periodic timer: period in microseconds (100 us)
  //   esp_timer_start_periodic(update_timer, 100ULL);
  //   ESP_LOGI("init", "updateMotion timer started\r\n");
  // } else {
  //   ESP_LOGI("init", "Failed to create updateMotion timer\r\n");
  // }


  gpio_reset_pin(STEP2_PIN);
  gpio_set_direction(STEP2_PIN, GPIO_MODE_OUTPUT);

  gpio_reset_pin(STEP1_PIN);
  gpio_set_direction(STEP1_PIN, GPIO_MODE_OUTPUT);

 //move 20,20
  moveLinearToPWM(20.0f, 20.0f, 100.0f, 50); //move to 20,20 at 100mm/s with 50% duty cycle

  

}




//-------------------------------------------------------------------------
// Move in a straight line using LEDC-generated limited PWM pulses
// Computes required step counts and per-motor pulse frequencies and
// calls send_limited_pwm_pulses for each motor's STEP pin.
// Returns true if at least one PWM was started successfully.
// Notes/assumptions:
// - Uses CoreXY kinematics: A = (x+y)*STEPS_PER_MM, B = (x-y)*STEPS_PER_MM
// - Direction pins are set before starting PWM.
// - If a motor requires 0 steps, its PWM is not started.
// - duty_percent is the PWM duty (0-100). Frequency is computed so that
//   pulses are distributed evenly over the movement time (distance / speed).
//-------------------------------------------------------------------------
bool moveLinearToPWM(float x_target_mm, float y_target_mm, float speed_mm_s, uint8_t duty_percent)
{
  if (speed_mm_s <= 0.0f) return false;

  // compute distance
  float dx = x_target_mm - gantry.x;
  float dy = y_target_mm - gantry.y;
  float distance = sqrtf(dx*dx + dy*dy);
  if (distance <= 0.0f) return false;

  // movement time in seconds
  float time_s = distance / speed_mm_s;
  if (time_s <= 0.0f) return false;

  // compute new motor targets using CoreXY mapping
  long newA = lroundf((x_target_mm + y_target_mm) * STEPS_PER_MM);
  long newB = lroundf((x_target_mm - y_target_mm) * STEPS_PER_MM);

  long deltaA = newA - motors.A_pos;
  long deltaB = newB - motors.B_pos;

  uint32_t pulsesA = (deltaA >= 0) ? (uint32_t)deltaA : (uint32_t)(-deltaA);
  uint32_t pulsesB = (deltaB >= 0) ? (uint32_t)deltaB : (uint32_t)(-deltaB);

  // compute required frequencies (pulses per second)
  uint32_t freqA = 0;
  uint32_t freqB = 0;
  if (pulsesA > 0) {
    float f = (float)pulsesA / time_s;
    if (f < 1.0f) f = 1.0f;
    freqA = (uint32_t)lroundf(f);
  }
  if (pulsesB > 0) {
    float f = (float)pulsesB / time_s;
    if (f < 1.0f) f = 1.0f;
    freqB = (uint32_t)lroundf(f);
  }

  bool startedA = false;
  bool startedB = false;

  // set direction pins before starting PWM
  if (pulsesA > 0) {
    setDirectionPin(1, (deltaA > 0) ? 1 : 0);
    // send PWM to STEP1_PIN if supported by send_limited_pwm_pulses
    gpio_num_t gpioA = (gpio_num_t)STEP1_PIN;
    startedA = send_limited_pwm_pulses(gpioA, freqA, duty_percent, pulsesA);
    if (!startedA) {
      ESP_LOGE("movePWM", "Failed to start PWM on STEP1_PIN (gpio %d)", (int)gpioA);
    } else {
      ESP_LOGI("movePWM", "Starting PWM on STEP1_PIN (gpio %d) at %lu Hz, duty %d%%, pulses %lu", (int)gpioA, freqA, duty_percent, pulsesA);
    }
  }

  if (pulsesB > 0) {
    setDirectionPin(2, (deltaB > 0) ? 1 : 0);
    gpio_num_t gpioB = (gpio_num_t)STEP2_PIN;
    startedB = send_limited_pwm_pulses(gpioB, freqB, duty_percent, pulsesB);
    if (!startedB) {
      ESP_LOGE("movePWM", "Failed to start PWM on STEP2_PIN (gpio %d)", (int)gpioB);
    } else {
      ESP_LOGI("movePWM", "Starting PWM on STEP2_PIN (gpio %d) at %lu Hz, duty %d%%, pulses %lu", (int)gpioB, freqB, duty_percent, pulsesB);
    }
  }

  // update target positions and gantry target/state
  motors.A_target = newA;
  motors.B_target = newB;
  gantry.x_target = x_target_mm;
  gantry.y_target = y_target_mm;
  gantry.motion_active = (startedA || startedB);
  gantry.position_reached = !(startedA || startedB);

  // NOTE: motors.A_pos / B_pos are left unchanged here; they will be
  // effectively stepped by the hardware PWM. We set the target so other
  // logic can reference expected end positions.

  return (startedA || startedB);
}