#include "MotionFSM.h"
#include "arm_math.h"

using namespace MotionFSM;
using namespace MotionParameter;

StateLoopArg MotionFSM::AutoRotateLoop(const volatile DBus::RCState* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){
    if(InitFlag.AutoRotateNI){
        Abs_deg_recode = hINS.getAngle().yaw;
        InitFlag.AutoRotateNI = false;
    }

    float relativeYaw = GimbalControl::angleMod(cur_sta.YawSta.pos-GimbalControl::YawZero + AUTOROTATE_ANGLE_BIAS) * (180.f/PI);
    float cos_yaw, sin_yaw;
    arm_sin_cos_f32(relativeYaw, &sin_yaw, &cos_yaw);

    ChassisControl::MoveState AbsMove = {
            {(float)RCsta->ch[3] * CHASSIS_K_CHX,
             -(float)RCsta->ch[2] * CHASSIS_K_CHY,
             -(float)RCsta->ch[0] * CHASSIS_K_OMEGA}
    };


    ChassisControl::MoveState target_state = {
            {AbsMove.vx * cos_yaw - AbsMove.vy * sin_yaw,
             AbsMove.vx * sin_yaw + AbsMove.vy * cos_yaw,
             AUTOROTATE_MAX_OMEGA}
    };

    float real_omega = AbsMove.omega * GIMBAL_K_LEAD_OMEGA;
    Abs_deg_recode = GimbalControl::angleMod(Abs_deg_recode +  real_omega* T_SAMPLE);

    float ref_ang = (float) RCsta->ch[1] * GIMBAL_K_CH_PITH;

    auto YawData = GimbalControl::getYawState();
    float taget_pos = YawData.pos + Abs_deg_recode - hINS.getAngle().yaw;
    StateLoopArg rel = {
            target_state,
            {{taget_pos,  -cur_sta.ChassisSta.omega * GIMBAL_K_OMEGA_FORWARD}},
            {{ref_ang, 0}}
    };


    return rel;
}
