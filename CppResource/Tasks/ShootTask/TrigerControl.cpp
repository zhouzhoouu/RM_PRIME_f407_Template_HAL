#include "ShootTask.h"

namespace TriggerControl{

    DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);

    DeltaPID Triger_PID = DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Tri_CUR, -MAX_Tri_CUR);

    void setTrigSpeed(float target_speed){

        short tmp_cur[] = {0, 0, 0, 0};
        float motor_speed = m3508Group_triger.getMotorState(0).speed;
        Triger_PID.Run(target_speed, motor_speed);
        tmp_cur[0] = (short)Triger_PID.Output;
        m3508Group_triger.setMotorCurrent(tmp_cur);
    }

}