#include "MotionControl.h"

namespace GimbalControl{


    DM4310 YawMotor(&hcan1, 1);
    DM4310 PithMotor(&hcan2, 2);

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
        float angle_err = angleMod(target_s.pos - YawMotor.getMotorState().pos * YawGearRate);
        float excep_t = angle_err*10.f;
        if(excep_t > 9.9f)excep_t = 9.9f;
        if(excep_t < -9.9f)excep_t = -9.9f;

        YawMotor.setMITcmd(0,target_s.omega * (1.f/YawGearRate),0,1,excep_t);
    }

    void setPithRelative(AxisState target_s){
        PithMotor.setMITcmd(target_s.pos,target_s.omega * (1.f/PithGearRate),8,1,0);
    }

}
