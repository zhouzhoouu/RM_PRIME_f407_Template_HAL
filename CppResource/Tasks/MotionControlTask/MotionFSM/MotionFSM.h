#ifndef F407_RM_TMPLATE_HAL_MOTIONFSM_H
#define F407_RM_TMPLATE_HAL_MOTIONFSM_H

#include "MotionControl.h"

namespace MotionFSM{

    struct StateLoopArg{
        ChassisControl::MoveState ChassisSta;
        GimbalControl::AxisState YawSta;
        GimbalControl::AxisState PithSta;
    };

    using StateHandler = StateLoopArg(*)(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    // 各 Loop 函数需统一 StateHandler 签名
    StateLoopArg InitLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg IdleLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg ChassisLeadLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg GimbalLeadLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg AutoAimLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);
    StateLoopArg AutoRotateLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&);

    inline StateHandler CurrentHandler = InitLoop;
    constexpr StateLoopArg DefaultStateArg = {
            {{0,0,0}},
            {{0,0}},
            {{0,0}}
    };

    struct Flag{
        bool InitNI,IdleNI,ChassisLeadNI,GimbalLeadNI,AutoAimNI,AutoRotateNI;
    };
    volatile inline Flag InitFlag = {
            true,
            true,
            true,
            true,
            true,
            true}
            ;

    static float Abs_deg_recode;


    //状态定义
    struct Init{};
    struct Idle{};
    struct ChassisLead{};
    struct GimbalLead{};
    struct AutoAim{};
    struct AutoRotate{};

    //事件定义
    struct InitComplete{};
    struct IntoIdle{};
    struct IntoChassisLead{};
    struct IntoGimbalLead{};
    struct IntoAutoAim{};
    struct IntoAutoRotate{};

    struct motionTransition{
        auto operator()() const{
            using namespace boost::sml;
            return make_transition_table(
                    *state<Init> + event<InitComplete> / []{ CurrentHandler = IdleLoop;InitFlag.IdleNI = true;} = state<Idle>,

                    state<_> + event<IntoChassisLead> / []{ CurrentHandler = ChassisLeadLoop;InitFlag.ChassisLeadNI = true;} = state<ChassisLead>,
                    state<_> + event<IntoGimbalLead> / []{ CurrentHandler = GimbalLeadLoop;InitFlag.GimbalLeadNI = true;} = state<GimbalLead>,
                    state<_> + event<IntoAutoAim> / []{ CurrentHandler = AutoAimLoop;InitFlag.AutoAimNI = true;} = state<AutoAim>,
                    state<_> + event<IntoAutoRotate> / []{ CurrentHandler = AutoRotateLoop;InitFlag.AutoRotateNI = true;} = state<AutoRotate>,
                    state<_> + event<IntoIdle> / []{ CurrentHandler = IdleLoop;InitFlag.IdleNI = true;} = state<Idle>
            );
        }
    };
}


#endif
