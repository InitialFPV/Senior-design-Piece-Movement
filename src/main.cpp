#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include <PinDefs.h>
#include <cstring>
#include "motionPos.h"
#include <cmath>
#include <esp_log.h>
#include "move_queue.h"

// AHAHHHHHHHHHH
static esp_timer_handle_t step_timer = nullptr;

typedef struct {
    uint32_t total_A, total_B;
    uint32_t sent_A, sent_B;
    int dirA, dirB;
    uint32_t leader_steps;
    uint32_t follower_steps;
    int leader_id; // 1=A, 2=B
    float step_period_us;
    int error_term;
    bool active;
} move_state_t;

static move_state_t move_ctx{};

inline void pulse_step(gpio_num_t pin) {
    gpio_set_level(pin, 1);
    uint64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < 2) { }
    gpio_set_level(pin, 0);
}

static void IRAM_ATTR step_timer_cb(void* arg) {
    if (!move_ctx.active) return;

    if (move_ctx.leader_id == 1) {
        if (move_ctx.sent_A < move_ctx.total_A) {
            pulse_step(STEP1_PIN);
            motors.A_pos += (move_ctx.dirA > 0 ? 1 : -1);
            move_ctx.sent_A++;
            move_ctx.error_term += move_ctx.follower_steps;
            if (move_ctx.error_term >= (int)move_ctx.leader_steps) {
                move_ctx.error_term -= move_ctx.leader_steps;
                if (move_ctx.sent_B < move_ctx.total_B) {
                    pulse_step(STEP2_PIN);
                    motors.B_pos += (move_ctx.dirB > 0 ? 1 : -1);
                    move_ctx.sent_B++;
                }
            }
        }
    } else { // B is leader
        if (move_ctx.sent_B < move_ctx.total_B) {
            pulse_step(STEP2_PIN);
            motors.B_pos += (move_ctx.dirB > 0 ? 1 : -1);
            move_ctx.sent_B++;
            move_ctx.error_term += move_ctx.follower_steps;
            if (move_ctx.error_term >= (int)move_ctx.leader_steps) {
                move_ctx.error_term -= move_ctx.leader_steps;
                if (move_ctx.sent_A < move_ctx.total_A) {
                    pulse_step(STEP1_PIN);
                    motors.A_pos += (move_ctx.dirA > 0 ? 1 : -1);
                    move_ctx.sent_A++;
                }
            }
        }
    }

    // stop when both complete
    if (move_ctx.sent_A >= move_ctx.total_A && move_ctx.sent_B >= move_ctx.total_B) {
        move_ctx.active = false;
        esp_timer_stop(step_timer);
        gantry.motion_active = false;
        gantry.position_reached = true;
    }
    
    // Update live XY coordinates
    gantry.x = (motors.A_pos + motors.B_pos) / (2.0f * STEPS_PER_MM);
    gantry.y = (motors.A_pos - motors.B_pos) / (2.0f * STEPS_PER_MM);

}

void gpio_output_init(gpio_num_t pin) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
}

void gpio_input_init(gpio_num_t pin, gpio_int_type_t intr_type) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = intr_type
    };
    gpio_config(&cfg);
}


bool moveToXY(float x_target_mm, float y_target_mm, float speed_mm_s, bool magnet_on) {
    
    
    gpio_set_level(MAGNET_PIN, magnet_on);

    if (speed_mm_s <= 0.0f) return false;


    float dx = x_target_mm - gantry.x;
    float dy = y_target_mm - gantry.y;
    float distance = sqrtf(dx*dx + dy*dy);
    if (distance <= 0.0f) return false;

    float time_s = distance / speed_mm_s;

    long newA = lroundf((x_target_mm + y_target_mm) * STEPS_PER_MM);
    long newB = lroundf((x_target_mm - y_target_mm) * STEPS_PER_MM);
    long deltaA = newA - motors.A_pos;
    long deltaB = newB - motors.B_pos;

    move_ctx.dirA = (deltaA >= 0) ? 1 : -1;
    move_ctx.dirB = (deltaB >= 0) ? 1 : -1;

    move_ctx.total_A = abs(deltaA);
    move_ctx.total_B = abs(deltaB);
    move_ctx.sent_A = move_ctx.sent_B = 0;
    move_ctx.error_term = 0;

    // Set directions
    gpio_set_level(DIR1_PIN, move_ctx.dirA > 0 ? 1 : 0);
    gpio_set_level(DIR2_PIN, move_ctx.dirB > 0 ? 1 : 0);

    // Determine leader/follower
    if (move_ctx.total_A >= move_ctx.total_B) {
        move_ctx.leader_id = 1;
        move_ctx.leader_steps = move_ctx.total_A;
        move_ctx.follower_steps = move_ctx.total_B;
    } else {
        move_ctx.leader_id = 2;
        move_ctx.leader_steps = move_ctx.total_B;
        move_ctx.follower_steps = move_ctx.total_A;
    }

    // step frequency based on leader
    float freq = move_ctx.leader_steps / time_s;
    move_ctx.step_period_us = 1e6 / freq;

    move_ctx.active = true;
    gantry.motion_active = true;
    gantry.position_reached = false;

    gantry.x_target = x_target_mm;
    gantry.y_target = y_target_mm;
    motors.A_target = newA;
    motors.B_target = newB;

    if (!step_timer) {
        esp_timer_create_args_t args = {
            .callback = &step_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "step_timer",
            .skip_unhandled_events = false
        };
        esp_timer_create(&args, &step_timer);
    }

    esp_timer_start_periodic(step_timer, (uint64_t)move_ctx.step_period_us);

    ESP_LOGI("MOVE", "Straight move: freq=%.1f Hz, period=%.1f us, A=%lu B=%lu", freq, move_ctx.step_period_us, move_ctx.total_A, move_ctx.total_B);
    return true;
}

void moveDispatchTask(void *pvParameters) {
    while (true) {
        // If idle and there are queued moves, dispatch the next one
        if (gantry.position_reached && !move_queue_is_empty()) {
            MoveCommand next;
            if (move_queue_pop(&next)) {
                ESP_LOGI("MOVE", "Dispatching queued move to (%.2f, %.2f) speed=%.1f magnet=%d", next.x, next.y, next.speed, next.magnet ? 1 : 0);
                moveToXY(next.x, next.y, next.speed, next.magnet);
            }
            else {
                ESP_LOGI("INIT", "Pop failed unexpectedly");
            }
        }

        if (gantry.position_reached) {
            ESP_LOGI("MOVE", "Idle: position reached");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

volatile bool limit_x_triggered = false;
volatile bool limit_y_triggered = false;

static void IRAM_ATTR limit_switch_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_num == LIMIT_Y_PIN) {
        limit_y_triggered = true;
    } else if (gpio_num == LIMIT_X_PIN) {
        limit_x_triggered = true;
    }
    // Stop motors immediately
    move_ctx.active = false;
    esp_timer_stop(step_timer);
    gantry.motion_active = false;
}

extern "C" void app_main(void) {
    gpio_output_init(STEP1_PIN);
    gpio_output_init(STEP2_PIN);
    gpio_output_init(DIR1_PIN);
    gpio_output_init(MAGNET_PIN);
    gpio_output_init(DIR2_PIN);
    gpio_output_init(SLEEP_PIN);
    gpio_output_init(EN_PIN);
    gpio_output_init(HFS_PIN);
    gpio_output_init(MODE0_PIN);
    gpio_output_init(MODE1_PIN);
    gpio_output_init(MODE2_PIN);
    
    gpio_input_init(LIMIT_Y_PIN, GPIO_INTR_NEGEDGE);
    gpio_input_init(LIMIT_X_PIN, GPIO_INTR_NEGEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LIMIT_Y_PIN, limit_switch_isr_handler, (void*) LIMIT_Y_PIN);
    gpio_isr_handler_add(LIMIT_X_PIN, limit_switch_isr_handler, (void*) LIMIT_X_PIN);

    gpio_set_level(SLEEP_PIN, 1);
    gpio_set_level(EN_PIN, 1);
    gpio_set_level(HFS_PIN, 1);
    gpio_set_level(MODE0_PIN, 0);
    gpio_set_level(MODE1_PIN, 0);
    gpio_set_level(MODE2_PIN, 1);
    
    setupMotion();
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI("INIT", "Startup, beginning homing sequence.");

    int homeOK = home_gantry();
    if (homeOK == -1) {
        ESP_LOGI("INIT", "Homing failed. Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    else{
        ESP_LOGI("INIT", "Homing sequence complete.");
    }

    // movement tests
    plan_move(0, 0, 5, 3, true);
    plan_move(5, 3, 6, 4, true);
    plan_move(6, 4, 6, 6, true);
    // plan_move(6, 6, 8, 7, true);
    plan_move(10, 7, 0, 0, false);
    ESP_LOGI("INIT", "Gantry motion and position status: active=%d reached=%d", gantry.motion_active ? 1 : 0, gantry.position_reached ? 1 : 0);

    if (homeOK != -1) {
        xTaskCreate(moveDispatchTask, "MoveDispatch", 4096, NULL, 5, NULL);
    }
}