#ifndef F407_RM_TMPLATE_HAL_MOTIONCONTROL_H
#define F407_RM_TMPLATE_HAL_MOTIONCONTROL_H

#include "TaskList.h"
#include "DJiMotorGroup.h"
#include "INS_Device.h"
#include "DM4310.h"
#include "DeltaPID.h"
#include "DBus.h"

#ifdef _
#undef _
#endif
#include "boost/sml.hpp"


namespace fsm = boost::sml;
using namespace Device;
using namespace Component;


namespace ChassisControl{

    union MoveState{
        struct {
            float vx;
            float vy;
            float omega;
        };
        float data[3];
    };

    //extern DJiMotorGroup m3508Group_Chassis;

    MoveState setMove(MoveState target_state);

}


namespace GimbalControl{

    static float constexpr YawZero = 0.23013f;
    static float constexpr PI = 3.141592653f;
    static float constexpr YawGearRate = 0.7;
    static float constexpr PithGearRate = 1.f;

    extern DM4310 YawMotor;
    extern DM4310 PithMotor;

    union AxisState{
        struct {
            float pos;
            float omega;
        };
        float data[2];
    };
    //将角度规范化到 -PI 到 PI
    float angleMod(float angle);

    AxisState getYawState();
    AxisState getPithState();

    void setYawRelative(AxisState target_s);
    void setPithRelative(AxisState target_s);
}


namespace MotionFSM{

    struct StateLoopArg{
        ChassisControl::MoveState ChassisSta;
        GimbalControl::AxisState YawSta;
        GimbalControl::AxisState PithSta;
    };

    using StateHandler = StateLoopArg(*)(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);

    // 各 Loop 函数需统一 StateHandler 签名
    StateLoopArg IdleLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg ChassisLeadLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg GimbalLeadLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg AutoAimLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg AutoRotateLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);


    inline StateHandler CurrentHandler = IdleLoop;
    constexpr StateLoopArg DefaultStateArg = {
            {{0,0,0}},
            {{0,0}},
            {{0,0}}
    };

    struct Flag{
        bool IdleNI,ChassisLeadNI,GimbalLeadNI,AutoAimNI,AutoRotateNI;
    };
    volatile inline Flag InitFlag = {true,false,false,false,false};


    //状态定义
    struct Init{};
    struct Idle{};
    struct ChassisLead{};
    struct GimbalLead{};
    struct AutoAim{};
    struct AutoRotate{};

    //事件定义
    struct IntoIdle{};
    struct IntoChassisLead{};
    struct IntoGimbalLead{};
    struct IntoAutoAim{};
    struct IntoAutoRotate{};

    struct motionTransition{
        auto operator()() const{
            using namespace boost::sml;
            return make_transition_table(
                    *state<Idle> + event<IntoChassisLead> / []{ CurrentHandler = ChassisLeadLoop;InitFlag.ChassisLeadNI = true;} = state<ChassisLead>,

                    state<_> + event<IntoChassisLead> / []{ CurrentHandler = ChassisLeadLoop;InitFlag.ChassisLeadNI = true;} = state<ChassisLead>,
                    state<_> + event<IntoGimbalLead> / []{ CurrentHandler = GimbalLeadLoop;InitFlag.GimbalLeadNI = true;} = state<GimbalLead>,
                    state<_> + event<IntoAutoAim> / []{ CurrentHandler = AutoAimLoop;InitFlag.AutoAimNI = true;} = state<AutoAim>,
                    state<_> + event<IntoAutoRotate> / []{ CurrentHandler = AutoRotateLoop;InitFlag.AutoRotateNI = true;} = state<AutoRotate>,
                    state<_> + event<IntoIdle> / []{ CurrentHandler = IdleLoop;InitFlag.IdleNI = true;} = state<Idle>
            );
        }
    };
}


#endif //F407_RM_TMPLATE_HAL_MOTIONCONTROL_H
