#include "MotionFSM.h"
#include "math.h"
using namespace MotionFSM;

static uint32_t counter = 0;
static float acc_x0, v_x, acc_y0, v_y;

StateLoopArg MotionFSM::InitLoop(const volatile DBus::RCState* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){

    if(HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)== GPIO_PIN_SET){

        float rad_bias = GimbalControl::PI*(1/180.f) * 21.f; //10度偏差
        GimbalControl::YawZero = GimbalControl::angleMod(  rad_bias+GimbalControl::getYawState().pos);
        InitFlag.InitNI = false;
    }


    return {
        {{0,0,0}},
        {{cur_sta.YawSta.pos,5}},
        {{0,0}}
    };
}