#include "MotionControl.h"


void MotionFSM::IdleLoop(){

}

void MotionFSM::ChassisLeadLoop(){

//    ChassisControl::MoveState target_state = {
//            (float)sta->ch[3] * 3.f,
//            (float)sta->ch[2] * 3.f,
//            (float)sta->ch[0] * 3.f
//    };
//
//    float ref_ang = -(float) sta->ch[1] * (1 / 1000.f);
//
//    ChassisControl::MoveState Chassis_bf = ChassisControl::setMove(target_state);
//
//    taget_pos = GimbalControl::YawMotor.getMotorState().pos - hINS.getAngle().yaw;
//
//    GimbalControl::setYawRelative(taget_pos, Chassis_bf.omega * 0.001f);
//    GimbalControl::setPithRelative(ref_ang);


}

void MotionFSM::GimbalLeadLoop(){

}

void MotionFSM::AutoAimLoop(){

}

void MotionFSM::AutoRotateLoop(){

}
