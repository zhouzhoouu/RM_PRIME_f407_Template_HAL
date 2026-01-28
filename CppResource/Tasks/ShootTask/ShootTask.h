#ifndef F407_RM_TMPLATE_HAL_SHOOTTASK_H
#define F407_RM_TMPLATE_HAL_SHOOTTASK_H

#include "TaskList.h"
#include "DJiMotorGroup.h"
#include "DBus.h"
#include "DeltaPID.h"

using namespace Device;
using namespace Component;

constexpr float MAX_Fib_CUR = 10000.0f;
constexpr float MAX_Tri_CUR = 3000.0f;
constexpr float Tri_BACKING_SPEED = 4000.f;
constexpr float Tri_BACKING_TIME = 200;
constexpr float TRIG_STUCK_RATE = 0.20f;
constexpr uint32_t TOPEN = 600;

namespace TriggerControl{
    void setTrigSpeed(float target_speed);
}

namespace FribControl{
    void setFribSpeed(float target_speed);
}

namespace ShootFSM{

    bool getIsFribOpened();
    bool getIsZeroCross();
    void ShootFSMLoop(short tin);

}

#endif //F407_RM_TMPLATE_HAL_SHOOTTASK_H
