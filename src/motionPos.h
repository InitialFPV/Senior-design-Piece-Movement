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