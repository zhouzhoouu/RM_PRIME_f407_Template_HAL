#include "MotionControl.h"
#include "arm_math.h"

using namespace MotionParameter;

namespace GimbalControl{

    DM4310 YawMotor(&hcan1, 2);
    DM4310 PithMotor(&hcan2, 1);

    float YawZero = 1.f;
    float PitchZero = 0.5f;

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
            {(PithMotor.getMotorState().pos - PitchZero)* PithGearRate,
            PithMotor.getMotorState().vel * PithGearRate}
        };
    }

    void setYawRelative(AxisState target_s){

        //角速度限制
        float current_omega = getYawState().omega;
        float delta_omega = target_s.omega - current_omega;
        float max_delta_omega = GIMBAL_MAX_ALPHA*T_SAMPLE;
        if(delta_omega > max_delta_omega) delta_omega = max_delta_omega;
        if(delta_omega < -max_delta_omega) delta_omega = -max_delta_omega;
        target_s.omega = current_omega + delta_omega;

        //位置限制与插值
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
        float kp_omega = alpha * (GIMBAL_MOVE_SPEED * (float)dir);

        float angle_err = angleMod(target_s.pos - current_pos);
        //限制幅度
        float except_t = angle_err * GIMBAL_YAW_KP;
        if(except_t > 9.9f)except_t = 9.9f;
        if(except_t < -9.9f)except_t = -9.9f;

        if(target_s.omega > 20.f) target_s.omega = 20.f;
        if(target_s.omega < -20.f) target_s.omega = -20.f;

        YawMotor.setMITcmd(0,(target_s.omega+kp_omega) * (1.f/YawGearRate), 0, GIMBAL_YAW_KD, except_t);
//        YawMotor.setMITcmd(0,0,0,0,0);
    }

    void setPithRelative(AxisState target_s){

        float yaw_rad = angleMod(getYawState().pos - YawZero + PI/2.f);
        if((yaw_rad < PI/6.f + 0.0f) && (yaw_rad > -PI/6.f - 0.0f)){
            if(target_s.pos > 0.29f)
                target_s.pos = 0.29f;
        }

        float pitch_rad = getPithState().pos + 0.03f;
        float error_pos = target_s.pos - pitch_rad;
        float error_omega = target_s.omega - getPithState().omega;

        //重力补偿
        float c,s;
        arm_sin_cos_f32(pitch_rad, &s, &c);
        float t_compen = c*GIMBAL_PITCH_CMX+s*GIMBAL_PITCH_CMY;
        float tor = error_pos * GIMBAL_PITCH_KP + error_omega * GIMBAL_PITCH_KD + t_compen;

        //tor = t_compen;

        if(tor > 9.9f) tor = 9.9f;
        if(tor < -9.9f) tor = -9.9f;

        PithMotor.setMITcmd(0,0,0,0,tor);
    }

}
