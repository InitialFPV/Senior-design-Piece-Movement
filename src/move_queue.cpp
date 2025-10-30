#include "move_queue.h"
#include "freertos/semphr.h"

// Simple fixed-size circular buffer
#define MOVE_QUEUE_CAPACITY 16

static MoveCommand q_buf[MOVE_QUEUE_CAPACITY];
static int q_head = 0; // index of next pop
static int q_tail = 0; // index of next push
static int q_count = 0;
static SemaphoreHandle_t q_mutex = NULL;

void move_queue_init(void) {
    if (!q_mutex) {
        q_mutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(q_mutex, portMAX_DELAY);
    q_head = q_tail = q_count = 0;
    xSemaphoreGive(q_mutex);
}

bool move_queue_push(const MoveCommand* cmd) {
    if (!cmd) return false;
    if (!q_mutex) move_queue_init();
    bool ok = false;
    if (xSemaphoreTake(q_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (q_count < MOVE_QUEUE_CAPACITY) {
            q_buf[q_tail] = *cmd;
            q_tail = (q_tail + 1) % MOVE_QUEUE_CAPACITY;
            q_count++;
            ok = true;
        }
        xSemaphoreGive(q_mutex);
    }
    return ok;
}

bool move_queue_pop(MoveCommand* out) {
    if (!out) return false;
    if (!q_mutex) move_queue_init();
    bool ok = false;
    if (xSemaphoreTake(q_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (q_count > 0) {
            *out = q_buf[q_head];
            q_head = (q_head + 1) % MOVE_QUEUE_CAPACITY;
            q_count--;
            ok = true;
        }
        xSemaphoreGive(q_mutex);
    }
    return ok;
}

bool move_queue_is_empty(void) {
    if (!q_mutex) move_queue_init();
    bool empty = true;
    if (xSemaphoreTake(q_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        empty = (q_count == 0);
        xSemaphoreGive(q_mutex);
    }
    return empty;
}
