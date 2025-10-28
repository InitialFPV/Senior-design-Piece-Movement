#include "motionPos.h"
#include "move_queue.h"
#include <cmath>
#include <algorithm>

// Implementation of setupMotion moved from header to avoid multiple-definition
void setupMotion() {
    gantry.motion_active = false;
    gantry.position_reached = true;
    gantry.x = 0;
    gantry.y = 0;
    motors.A_pos = 0;
    motors.B_pos = 0;
}


inline float half_dx_between(int a1, int a2) {
    a1 = std::clamp(a1, 0, 11);
    a2 = std::clamp(a2, 0, 11);
    return fabs(board_pos[a2][0][0] - board_pos[a1][0][0]) / 2.0f;
}

inline float half_dy_between(int b1, int b2) {
    b1 = std::clamp(b1, 0, 7);
    b2 = std::clamp(b2, 0, 7);
    return fabs(board_pos[0][b2][1] - board_pos[0][b1][1]) / 2.0f;
}

void plan_move(int A_from, int B_from, int A_to, int B_to, bool direct) {
    MoveCommand mc;
    float fromX = board_pos[A_from][B_from][0];
    float fromY = board_pos[A_from][B_from][1];
    float toX   = board_pos[A_to][B_to][0];
    float toY   = board_pos[A_to][B_to][1];

    // ---- Step 0: Move to "from" square (magnet off)
    mc = {fromX, fromY, 120.0f, false};
    move_queue_push(&mc);

    // ---- Step 1: If direct, go straight there
    if (direct) {
        mc = {toX, toY, 100.0f, true};
        move_queue_push(&mc);
    } else {
        mc.magnet = true;
        mc.speed = 80.0f;

        //---------------------------------------------------------
        // Step 1: Move halfway out of source square into corridor
        //---------------------------------------------------------
        float x1 = fromX;
        float y1 = fromY;

        if (A_to > A_from)
            x1 += half_dx_between(A_from, A_from + 1);
        else if (A_to < A_from)
            x1 -= half_dx_between(A_from, A_from - 1);
        else
            x1 -= half_dx_between(A_from, std::max(0, A_from - 1)); // default left

        if (B_to > B_from)
            y1 += half_dy_between(B_from, B_from + 1);
        else if (B_to < B_from)
            y1 -= half_dy_between(B_from, B_from - 1);
        else
            y1 += half_dy_between(B_from, std::min(7, B_from + 1)); // default up

        mc.x = x1;
        mc.y = y1;
        move_queue_push(&mc);

        //---------------------------------------------------------
        // Step 2a: Move horizontally in corridor (constant Y)
        //---------------------------------------------------------
        float x2 = x1;
        float y2 = y1;
        if (A_to != A_from) {
            if (A_to > A_from)
                x2 = toX - half_dx_between(A_to - 1, A_to);
            else
                x2 = toX + half_dx_between(A_to + 1, A_to);
        }
        mc.x = x2;
        mc.y = y2;
        move_queue_push(&mc);

        //---------------------------------------------------------
        // Step 2b: Move vertically in corridor (constant X)
        //---------------------------------------------------------
        float x3 = x2;
        float y3 = y2;
        if (B_to != B_from) {
            if (B_to > B_from)
                y3 = toY - half_dy_between(B_to - 1, B_to);
            else
                y3 = toY + half_dy_between(B_to + 1, B_to);
        }
        mc.x = x3;
        mc.y = y3;
        move_queue_push(&mc);

        //---------------------------------------------------------
        // Step 3: Move to destination center
        //---------------------------------------------------------
        mc.x = toX;
        mc.y = toY;
        mc.speed = 60.0f;
        move_queue_push(&mc);
    }

    // ---- Step 4: Release magnet
    mc = {toX, toY, 0.0f, false};
    move_queue_push(&mc);
}
