#include "MotionControl.h"

using namespace MotionParameter;

namespace GimbalControl{

    DM4310 YawMotor(&hcan1, 1);
    DM4310 PithMotor(&hcan2, 2);

    float YawZero = 0.f;

    //将角度规范化到 -PI 到 PI
    float angleMod(float angle){
        while (angle > PI) angle -= 2.f * PI;
        while (angle < -PI) angle += 2.f * PI;
        return angle;
    }


    AxisState getYawState(){
        return {
                {YawMotor.getMotorState().pos * YawGearRate,
                 YawMotor.getMotorState().vel * YawGearRate}
        };
    }

    AxisState getPithState(){
        return {
            {PithMotor.getMotorState().pos * PithGearRate,
            PithMotor.getMotorState().vel * PithGearRate}
        };
    }

    void setYawRelative(AxisState target_s){

        float  current_pos = angleMod(getYawState().pos);
        float des_pos = angleMod(target_s.pos);

        float dpos = GimbalControl::angleMod(  des_pos - current_pos);
        int dir = 1;
        float alpha = 0;

        //符号判断
        if(dpos < -0.f) dir = -1;

        //线性插值（可改成其他插值函数）
        if(dpos * (float)dir > GIMBAL_RAD_VEL){
            alpha = 1;
        }
        else if (dpos * (float)dir < GIMBAL_RAD_POS){
            alpha = 0;
        }
        else {
            alpha = (dpos * (float)dir - GIMBAL_RAD_POS) / (GIMBAL_RAD_VEL - GIMBAL_RAD_POS);
        }

        target_s.pos = current_pos * alpha + des_pos * (1.f - alpha);
        target_s.omega = (alpha * (GIMBAL_MOVE_SPEED * (float)dir) + (1.f - alpha)* target_s.omega);

        float angle_err = angleMod(target_s.pos - current_pos);
        //限制幅度
        float except_t = angle_err * GIMBAL_YAW_KP;
        if(except_t > 9.9f)except_t = 9.9f;
        if(except_t < -9.9f)except_t = -9.9f;

        if(target_s.omega > 20.f) target_s.omega = 20.f;
        if(target_s.omega < -20.f) target_s.omega = -20.f;

        YawMotor.setMITcmd(0,target_s.omega * (1.f/YawGearRate), 0, GIMBAL_YAW_KD, except_t);
//        YawMotor.setMITcmd(0,0,0,0,0);
    }

    void setPithRelative(AxisState target_s){
        PithMotor.setMITcmd(target_s.pos-0.02f,target_s.omega * (1.f/PithGearRate),GIMBAL_PITCH_KP,GIMBAL_PITCH_KD,0);
    }

}
