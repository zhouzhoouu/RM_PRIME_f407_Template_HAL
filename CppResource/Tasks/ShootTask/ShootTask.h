#ifndef F407_RM_TMPLATE_HAL_SHOOTTASK_H
#define F407_RM_TMPLATE_HAL_SHOOTTASK_H

#include "TaskList.h"
#include "DJiMotorGroup.h"
#include "DBus.h"
#include "DeltaPID.h"
#include "VT03.h"

using namespace Device;
using namespace Component;

constexpr float MAX_Fib_CUR = 10000.0f;
constexpr float MAX_Tri_CUR = 2800.0f;
constexpr float MAX_TRIG_SPEED = 3500.f;
constexpr float Tri_BACKING_SPEED = 3000.f;
constexpr float Tri_BACKING_TIME = 200;
constexpr float TRIG_STUCK_RATE = 0.20f;

constexpr uint32_t SHOOT_GAP_MS = (uint32_t)((36.f*60000.f)/(3.4615f*MAX_TRIG_SPEED)) + 10;
constexpr uint32_t TOPEN = 600;

namespace TriggerControl{

    void NotifyPowerSate(bool s);
    void setSpeed(float target_speed);
    void Loop();
    void AddStep();

}

namespace FribControl{
    void NotifyPowerSate(bool s);
    void setFribSpeed(float target_speed);
}

namespace ShootFSM{

    void FSM_Reset(void);
    void ShootFSM_Button_Loop(bool b);
    bool getIsFribOpened();
    bool getIsZeroCross();
    void ShootFSM_Wheel_Loop(short tin);

}

#endif //F407_RM_TMPLATE_HAL_SHOOTTASK_H
