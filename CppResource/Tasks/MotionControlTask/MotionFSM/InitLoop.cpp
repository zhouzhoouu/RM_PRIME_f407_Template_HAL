#include "MotionFSM.h"
#include "math.h"
using namespace MotionFSM;

static uint32_t counter = 0;
static float acc_x0, v_x, acc_y0, v_y;

StateLoopArg MotionFSM::InitLoop(const volatile DBus::RCState* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){
    InitFlag.InitNI = false;

    return cur_sta;
}