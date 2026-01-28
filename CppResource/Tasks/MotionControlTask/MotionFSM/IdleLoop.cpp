#include "MotionFSM.h"
#include "arm_math.h"

using namespace MotionFSM;

StateLoopArg MotionFSM::IdleLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg& cur_sta){
    if(InitFlag.IdleNI){
        InitFlag.IdleNI = false;
    }

    return cur_sta;
}
