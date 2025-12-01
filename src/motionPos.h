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
    {{0.000f, 0.000f}, {0.000f, 37.000f}, {0.000f, 74.000f}, {0.000f, 111.000f}, {0.000f, 148.000f}, {0.000f, 185.000f}, {0.000f, 222.000f}, {0.000f, 259.000f}},
    {{30.000f, 0.000f}, {30.000f, 37.000f}, {30.000f, 74.000f}, {30.000f, 111.000f}, {30.000f, 148.000f}, {30.000f, 185.000f}, {30.000f, 222.000f}, {30.000f, 259.000f}},
    {{75.090f, 0.000f}, {75.090f, 37.000f}, {75.090f, 74.000f}, {75.090f, 111.000f}, {75.090f, 148.000f}, {75.090f, 185.000f}, {75.090f, 222.000f}, {75.090f, 259.000f}},
    {{112.090f, 0.000f}, {112.090f, 37.000f}, {112.090f, 74.000f}, {112.090f, 111.000f}, {112.090f, 148.000f}, {112.090f, 185.000f}, {112.090f, 222.000f}, {112.090f, 259.000f}},
    {{149.090f, 0.000f}, {149.090f, 37.000f}, {149.090f, 74.000f}, {149.090f, 111.000f}, {149.090f, 148.000f}, {149.090f, 185.000f}, {149.090f, 222.000f}, {149.090f, 259.000f}},
    {{186.090f, 0.000f}, {186.090f, 37.000f}, {186.090f, 74.000f}, {186.090f, 111.000f}, {186.090f, 148.000f}, {186.090f, 185.000f}, {186.090f, 222.000f}, {186.090f, 259.000f}},
    {{223.090f, 0.000f}, {223.090f, 37.000f}, {223.090f, 74.000f}, {223.090f, 111.000f}, {223.090f, 148.000f}, {223.090f, 185.000f}, {223.090f, 222.000f}, {223.090f, 259.000f}},
    {{260.090f, 0.000f}, {260.090f, 37.000f}, {260.090f, 74.000f}, {260.090f, 111.000f}, {260.090f, 148.000f}, {260.090f, 185.000f}, {260.090f, 222.000f}, {260.090f, 259.000f}},
    {{297.090f, 0.000f}, {297.090f, 37.000f}, {297.090f, 74.000f}, {297.090f, 111.000f}, {297.090f, 148.000f}, {297.090f, 185.000f}, {297.090f, 222.000f}, {297.090f, 259.000f}},
    {{334.090f, 0.000f}, {334.090f, 37.000f}, {334.090f, 74.000f}, {334.090f, 111.000f}, {334.090f, 148.000f}, {334.090f, 185.000f}, {334.090f, 222.000f}, {334.090f, 259.000f}},
    {{379.180f, 0.000f}, {379.180f, 37.000f}, {379.180f, 74.000f}, {379.180f, 111.000f}, {379.180f, 148.000f}, {379.180f, 185.000f}, {379.180f, 222.000f}, {379.180f, 259.000f}},
    {{409.180f, 0.000f}, {409.180f, 37.000f}, {409.180f, 74.000f}, {409.180f, 111.000f}, {409.180f, 148.000f}, {409.180f, 185.000f}, {409.180f, 222.000f}, {409.180f, 259.000f}}
};