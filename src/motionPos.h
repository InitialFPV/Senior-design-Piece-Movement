const float PULLEY_TEETH   = 20.0f;
const float BELT_PITCH_MM  = 2.0f;
const float STEP_ANGLE_DEG = 1.8f;
const float MICROSTEP      = 16.0f;

const float STEPS_PER_REV   = 360.0f / STEP_ANGLE_DEG;
const float DIST_PER_REV_MM = PULLEY_TEETH * BELT_PITCH_MM;
const float STEPS_PER_MM    = (STEPS_PER_REV * MICROSTEP) / DIST_PER_REV_MM;


//--------------------------------------------
// Motion State
//--------------------------------------------
struct {
    float x, y;              // current XY position (mm)
    float x_target, y_target;
    bool motion_active;
    bool position_reached;
} gantry;

struct {
    long A_pos, B_pos;        // current step counts
    long A_target, B_target;  // target step counts
} motors;

//--------------------------------------------
// Initialization
//--------------------------------------------
void setupMotion() {
    gantry.motion_active = false;
    gantry.position_reached = true;
    gantry.x = 0;
    gantry.y = 0;
    motors.A_pos = 0;
    motors.B_pos = 0;
}