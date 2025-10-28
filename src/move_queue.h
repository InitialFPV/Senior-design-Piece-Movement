#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    float x;
    float y;
    float speed;
    bool magnet;
} MoveCommand;

// Initialize the move queue. Must be called before other APIs.
void move_queue_init(void);

// Push a command onto the queue. Returns true on success, false if full.
bool move_queue_push(const MoveCommand* cmd);

// Pop a command from the queue. Returns true on success, false if empty.
bool move_queue_pop(MoveCommand* out);

// Return true if queue empty.
bool move_queue_is_empty(void);
