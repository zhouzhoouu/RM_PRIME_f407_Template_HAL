#include "MotionFSM.h"
#include "arm_math.h"

using namespace MotionFSM;

StateLoopArg MotionFSM::AutoAimLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&){
    if(InitFlag.AutoAimNI){
        InitFlag.AutoAimNI = false;
    }
    return DefaultStateArg;
}
