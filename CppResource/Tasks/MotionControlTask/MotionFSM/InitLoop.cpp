#include "MotionFSM.h"
#include "math.h"
using namespace MotionFSM;

static uint32_t counter = 0;
static float acc_x0, v_x, acc_y0, v_y;

StateLoopArg MotionFSM::InitLoop(const volatile DBus::RCState* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){

    if(counter == 0){
        acc_x0 = 0.f;
        acc_y0 = 0.f;
        v_x = 0.f;
        v_y = 0.f;
    }
    else if(counter <= 1000){
        acc_x0 += hINS.getAccel().x;
        acc_y0 += hINS.getAccel().y;
    }
    else if(counter <= 1600){
        v_x += hINS.getAccel().x;
        v_y += hINS.getAccel().y;
        counter++;
        return {
            {{700,0,0}},
            {{cur_sta.YawSta.pos,0}},
            {{0,0}}
        };
    }
    else if(counter >= 1600){
        v_x -= acc_x0 * 0.6f;
        v_y -= acc_y0 * 0.6f;

        float rad_zp = atan2(-v_x,v_y);// + -GimbalControl::PI*0.5f;
        GimbalControl::YawZero = GimbalControl::angleMod( rad_zp + GimbalControl::getYawState().pos);
        InitFlag.InitNI = false;
    }
    counter++;
    return {
        {{0,0,0}},
        {{cur_sta.YawSta.pos,0}},
        {{0,0}}
    };
}