#include "MotionControl.h"
#include "arm_math.h"

using namespace MotionFSM;

StateLoopArg MotionFSM::IdleLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&){
    if(InitFlag.IdleNI){
        InitFlag.IdleNI = false;
    }

    return DefaultStateArg;
}

StateLoopArg MotionFSM::ChassisLeadLoop(const volatile DBus::RCState* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){

    if(InitFlag.ChassisLeadNI){
        InitFlag.ChassisLeadNI = false;
    }

    ChassisControl::MoveState target_state = {
            {(float)RCsta->ch[3] * 1.5f,
            -(float)RCsta->ch[2] * 1.5f,
            -(float)RCsta->ch[0] * 1.5f}
    };

    float ref_ang = (float) RCsta->ch[1] * (1 / 1500.f);

    constexpr float MOVE_SPEED = 2.f;
    constexpr float RAD_VEL = 0.07f;
    constexpr float RAD_POS = 0.01f;


    auto YawData = GimbalControl::getYawState();

    float des_pos = GimbalControl::YawZero;
    float dpos = GimbalControl::angleMod(YawData.pos - des_pos);
    int dir = 1;
    float alpha = 0;

    if(dpos < 0) dir = -1;


    if(dpos * (float)dir > RAD_VEL){
        alpha = 1;
    }
    else if (dpos * (float)dir < RAD_POS){
        alpha = 0;
    } else {
        alpha = (dpos * (float)dir - RAD_POS) / (RAD_VEL - RAD_POS);
    }
    //alpha=0;
    //YawData.pos = YawData.pos * alpha + des_pos * (1.f - alpha);
    YawData.omega = -alpha * (MOVE_SPEED * (float)dir);

    StateLoopArg rel = {
        target_state,
        YawData,
        {{ref_ang, 0}}
    };

    return rel;

}


static float Abs_deg_recode;

StateLoopArg MotionFSM::GimbalLeadLoop(const volatile DBus::RCState* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){

    if(InitFlag.GimbalLeadNI){
        Abs_deg_recode = hINS.getAngle().yaw;
        InitFlag.GimbalLeadNI = false;
    }

    float relativeYaw = GimbalControl::angleMod(cur_sta.YawSta.pos-GimbalControl::YawZero);

    ChassisControl::MoveState AbsMove = {
            {(float)RCsta->ch[3] * 1.5f,
             -(float)RCsta->ch[2] * 1.5f,
             -(float)RCsta->ch[0] * 1.5f}
    };


    ChassisControl::MoveState target_state = {
            {(float)RCsta->ch[3] * 1.5f,
             -(float)RCsta->ch[2] * 1.5f,
             AbsMove.omega}
    };

    //Abs_deg_recode = GimbalControl::angleMod(Abs_deg_recode + AbsMove.omega * (2.66f / 900) * 0.001f);
    float ref_ang = (float) RCsta->ch[1] * (1 / 1500.f);

    auto YawData = GimbalControl::getYawState();
    float taget_pos = YawData.pos + Abs_deg_recode - hINS.getAngle().yaw;
    StateLoopArg rel = {
            target_state,
            {{taget_pos, -cur_sta.ChassisSta.omega * (2.66f/900)}},
            {{ref_ang, 0}}
    };


    return rel;
}

StateLoopArg MotionFSM::AutoAimLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&){
    if(InitFlag.AutoAimNI){
        InitFlag.AutoAimNI = false;
    }
    return DefaultStateArg;
}

StateLoopArg MotionFSM::AutoRotateLoop(const volatile DBus::RCState*, INS_Device&, const StateLoopArg&){
    if(InitFlag.AutoRotateNI){
        InitFlag.AutoRotateNI = false;
    }
    return DefaultStateArg;
}
