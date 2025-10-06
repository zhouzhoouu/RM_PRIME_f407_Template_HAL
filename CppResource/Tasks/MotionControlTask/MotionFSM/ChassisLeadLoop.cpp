#include "MotionFSM.h"
#include "arm_math.h"

using namespace MotionFSM;
using namespace MotionParameter;

StateLoopArg MotionFSM::ChassisLeadLoop(const volatile DBus::RCState* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){

    if(InitFlag.ChassisLeadNI){
        InitFlag.ChassisLeadNI = false;
    }

    ChassisControl::MoveState target_state = {
            {(float)RCsta->ch[3] * CHASSIS_K_CHX,
             -(float)RCsta->ch[2] * CHASSIS_K_CHY,
             -(float)RCsta->ch[0] * CHASSIS_K_OMEGA}
    };

    float ref_ang = (float) RCsta->ch[1] * GIMBAL_K_CH_PITH;

    StateLoopArg rel = {
            target_state,
            {{GimbalControl::YawZero, 0}},
            {{ref_ang, 0}}
    };

    return rel;

}
