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
// Named structs so we can declare extern instances in this header
typedef struct {
    float x, y;              // current XY position (mm)
    float x_target, y_target;
    bool motion_active;
    bool position_reached;
} Gantry_t;

typedef struct {
    long A_pos, B_pos;        // current step counts
    long A_target, B_target;  // target step counts
} Motors_t;

// Declare global instances (defined in motionPos.cpp)
extern Gantry_t gantry;
extern Motors_t motors;

//--------------------------------------------
// Initialization
//--------------------------------------------
void setupMotion();
void plan_move(int A_from, int B_from, int A_to, int B_to, bool direct);
int home_gantry();

// Declare moveToXY so it can be used in motionPos.cpp
bool moveToXY(float x_target_mm, float y_target_mm, float speed_mm_s, bool magnet_on);

extern volatile bool limit_y_triggered;
extern volatile bool limit_x_triggered;

constexpr float board_pos[12][8][2] = {
    {{10.000f, 10.000f}, {10.000f, 47.000f}, {10.000f, 84.000f}, {10.000f, 121.000f}, {10.000f, 158.000f}, {10.000f, 195.000f}, {10.000f, 232.000f}, {10.000f, 269.000f}},
    {{40.000f, 10.000f}, {40.000f, 47.000f}, {40.000f, 84.000f}, {40.000f, 121.000f}, {40.000f, 158.000f}, {40.000f, 195.000f}, {40.000f, 232.000f}, {40.000f, 269.000f}},
    {{85.090f, 10.000f}, {85.090f, 47.000f}, {85.090f, 84.000f}, {85.090f, 121.000f}, {85.090f, 158.000f}, {85.090f, 195.000f}, {85.090f, 232.000f}, {85.090f, 269.000f}},
    {{122.090f, 10.000f}, {122.090f, 47.000f}, {122.090f, 84.000f}, {122.090f, 121.000f}, {122.090f, 158.000f}, {122.090f, 195.000f}, {122.090f, 232.000f}, {122.090f, 269.000f}},
    {{159.090f, 10.000f}, {159.090f, 47.000f}, {159.090f, 84.000f}, {159.090f, 121.000f}, {159.090f, 158.000f}, {159.090f, 195.000f}, {159.090f, 232.000f}, {159.090f, 269.000f}},
    {{196.090f, 10.000f}, {196.090f, 47.000f}, {196.090f, 84.000f}, {196.090f, 121.000f}, {196.090f, 158.000f}, {196.090f, 195.000f}, {196.090f, 232.000f}, {196.090f, 269.000f}},
    {{233.090f, 10.000f}, {233.090f, 47.000f}, {233.090f, 84.000f}, {233.090f, 121.000f}, {233.090f, 158.000f}, {233.090f, 195.000f}, {233.090f, 232.000f}, {233.090f, 269.000f}},
    {{270.090f, 10.000f}, {270.090f, 47.000f}, {270.090f, 84.000f}, {270.090f, 121.000f}, {270.090f, 158.000f}, {270.090f, 195.000f}, {270.090f, 232.000f}, {270.090f, 269.000f}},
    {{307.090f, 10.000f}, {307.090f, 47.000f}, {307.090f, 84.000f}, {307.090f, 121.000f}, {307.090f, 158.000f}, {307.090f, 195.000f}, {307.090f, 232.000f}, {307.090f, 269.000f}},
    {{344.090f, 10.000f}, {344.090f, 47.000f}, {344.090f, 84.000f}, {344.090f, 121.000f}, {344.090f, 158.000f}, {344.090f, 195.000f}, {344.090f, 232.000f}, {344.090f, 269.000f}},
    {{389.180f, 10.000f}, {389.180f, 47.000f}, {389.180f, 84.000f}, {389.180f, 121.000f}, {389.180f, 158.000f}, {389.180f, 195.000f}, {389.180f, 232.000f}, {389.180f, 269.000f}},
    {{419.180f, 10.000f}, {419.180f, 47.000f}, {419.180f, 84.000f}, {419.180f, 121.000f}, {419.180f, 158.000f}, {419.180f, 195.000f}, {419.180f, 232.000f}, {419.180f, 269.000f}}
};