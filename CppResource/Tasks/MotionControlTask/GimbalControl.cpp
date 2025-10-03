#include "MotionControl.h"

namespace GimbalControl{

    static float constexpr PI = 3.141592653f;

    DM4310 YawMotor(&hcan1, 2);
    DM4310 PithMotor(&hcan2, 1);

    DeltaPID anglePID_Yaw = DeltaPID(8.0f, 0.0f, 800.0f, 0.0f, 7, -7);

    //将角度规范化到 -PI 到 PI
    float angleMod(float angle){
        while (angle > PI) angle -= 2.f * PI;
        while (angle < -PI) angle += 2.f * PI;
        return angle;
    }

    void setYawRelative(float angle, float omega_forward){
        float angle_err = angleMod(angle - YawMotor.getMotorState().pos);
        anglePID_Yaw.Run(0, angle_err);
        YawMotor.setMITcmd(0,omega_forward,0,1,angle_err*10.f);
    }

    void setPithRelative(float angle, float omega_forward){
        PithMotor.setMITcmd(angle,omega_forward,8,1,0);
    }

}
