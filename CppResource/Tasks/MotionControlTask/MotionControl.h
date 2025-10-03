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

    struct MoveState{
        union {
            struct {
                float vx;
                float vy;
                float omega;
            };
            float data[3];
        };
    };

    extern DJiMotorGroup m3508Group_Chassis;

    //实测omega=1980时，实际转动速度2rad/s左右
    MoveState setMove(MoveState target_state);
}


namespace GimbalControl{

    extern DM4310 YawMotor;
    extern DM4310 PithMotor;


    //将角度规范化到 -PI 到 PI
    float angleMod(float angle);

    void setYawRelative(float angle, float omega_forward = 0.f);

    void setPithRelative(float angle, float omega_forward = 0.f);
}


namespace MotionFSM{

    void IdleLoop();
    void ChassisLeadLoop();
    void GimbalLeadLoop();
    void AutoAimLoop();
    void AutoRotateLoop();

    struct Init{
        void on_entry();
    };
    struct Idle{
        void on_entry();
    };
    struct ChassisLead{
        void on_entry();
    };
    struct GimbalLead{
        void on_entry();
    };
    struct AutoAim{
        void on_entry();
    };
    struct AutoRotate{
        void on_entry();
    };

    struct LeverInput{
        uint8_t Left;
        uint8_t Right;
    };
    struct InitComplete{};

    constexpr auto is_Idle = [](const LeverInput& lin){ return lin.Left == 1;};
    constexpr auto is_ChassisLead = [](const LeverInput& lin){ return lin.Left == 3;};
    constexpr auto is_GimbalLead = [](const LeverInput& lin){ return lin.Left==2 && lin.Right==3;};
    constexpr auto is_AutoAim = [](const LeverInput& lin){ return lin.Left==2 && lin.Right==1;};
    constexpr auto is_AutoRotate = [](const LeverInput& lin){ return lin.Left==2 && lin.Right==2;};



    struct motionTransition{
        auto operator()() const{
            using namespace boost::sml;
            return make_transition_table(
                    *state<Idle> + event<LeverInput>[is_ChassisLead] = state<ChassisLead>,
                    state<GimbalLead> + event<LeverInput>[is_ChassisLead] = state<ChassisLead>,
                    state<AutoAim> + event<LeverInput>[is_ChassisLead] = state<ChassisLead>,
                    state<AutoRotate> + event<LeverInput>[is_ChassisLead] = state<ChassisLead>,

                    state<Idle> + event<LeverInput>[is_GimbalLead] = state<GimbalLead>,
                    state<ChassisLead> + event<LeverInput>[is_GimbalLead] = state<GimbalLead>,
                    state<AutoAim> + event<LeverInput>[is_GimbalLead] = state<GimbalLead>,
                    state<AutoRotate> + event<LeverInput>[is_GimbalLead] = state<GimbalLead>,

                    state<Idle> + event<LeverInput>[is_AutoAim] = state<AutoAim>,
                    state<ChassisLead> + event<LeverInput>[is_AutoAim] = state<AutoAim>,
                    state<GimbalLead> + event<LeverInput>[is_AutoAim] = state<AutoAim>,
                    state<AutoRotate> + event<LeverInput>[is_AutoAim] = state<AutoAim>,

                    state<Idle> + event<LeverInput>[is_AutoRotate] = state<AutoRotate>,
                    state<ChassisLead> + event<LeverInput>[is_AutoRotate] = state<AutoRotate>,
                    state<GimbalLead> + event<LeverInput>[is_AutoRotate] = state<AutoRotate>,
                    state<AutoAim> + event<LeverInput>[is_AutoRotate] = state<AutoRotate>,

                    state<ChassisLead> + event<LeverInput>[is_Idle] = state<Idle>,
                    state<GimbalLead> + event<LeverInput>[is_Idle] = state<Idle>,
                    state<AutoAim> + event<LeverInput>[is_Idle] = state<Idle>,
                    state<AutoRotate> + event<LeverInput>[is_Idle] = state<Idle>
            );
        }
    };
}


#endif //F407_RM_TMPLATE_HAL_MOTIONCONTROL_H
