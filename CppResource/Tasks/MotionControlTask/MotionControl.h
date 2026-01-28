#ifndef F407_RM_TMPLATE_HAL_MOTIONCONTROL_H
#define F407_RM_TMPLATE_HAL_MOTIONCONTROL_H

#include "TaskList.h"
#include "DJiMotorGroup.h"
#include "SupCap.h"
#include "Referee.h"
#include "INS_Device.h"
#include "DM4310.h"
#include "DeltaPID.h"
#include "DBus.h"
#include "boost/sml.hpp"

namespace fsm = boost::sml;
using namespace Device;
using namespace Component;

namespace MotionParameter{
    static float constexpr T_SAMPLE = 0.001f; //1ms

    static float constexpr RC_MAX_INPUT = 660.f;

    static float constexpr CHASSIS_K_CHX = 3.5f;
    static float constexpr CHASSIS_K_CHY = 3.5f;
    static float constexpr CHASSIS_K_OMEGA = 4.0f;
    static float constexpr CHASSIS_MAX_ACCEL = 500000.f;
    static float constexpr CHASSIS_MAX_ALPHA = 500000.f;

    static float constexpr CHASSIS_FOLLOW_OMEGA = 1200.f;
    static float constexpr CHASSIS_FOLLOW_DES = 0.3; //rad


    static float constexpr GIMBAL_YAW_KD = .42f;
    static float constexpr GIMBAL_YAW_KP = 12.f;
    static float constexpr GIMBAL_PITCH_KP = 10.f;
    static float constexpr GIMBAL_PITCH_KD = 1.f;
    static float constexpr GIMBAL_PITCH_CMX = -.73f;
    static float constexpr GIMBAL_PITCH_CMY = -0.f;

    static float constexpr GIMBAL_K_CH_PITH = 1/1100.f;
    static float constexpr GIMBAL_K_OMEGA_FORWARD = 2.37f/1980.f;
    static float constexpr GIMBAL_K_LEAD_OMEGA = 2.3f/1000.f;

    static float constexpr GIMBAL_MOVE_SPEED = RC_MAX_INPUT * CHASSIS_K_OMEGA * GIMBAL_K_LEAD_OMEGA + 1; //云台运动参考速度
    static float constexpr GIMBAL_RAD_VEL = .5f; //匀速运动区间
    static float constexpr GIMBAL_RAD_POS = 0.3f; //位置控区间
    static float constexpr GIMBAL_MAX_ALPHA = 12000.f;

    static float constexpr AUTOROTATE_MAX_OMEGA = 6000.f;
    static float constexpr AUTOROTATE_ANGLE_BIAS = -0.3f; //rad

}

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

    static float constexpr PI = 3.141592653f;
    static float constexpr YawGearRate = 1;
    static float constexpr PithGearRate = 1.f;

    extern float YawZero;
    extern DM4310 YawMotor;

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



#endif //F407_RM_TMPLATE_HAL_MOTIONCONTROL_H
