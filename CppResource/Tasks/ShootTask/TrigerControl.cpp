#include "ShootTask.h"

namespace TriggerControl{

    uint32_t timeout = 0;
    int dir = 1;
    constexpr uint32_t trig_id = 0;
    bool isbacking = false;

    DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);

    DeltaPID Triger_PID = DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Tri_CUR, -MAX_Tri_CUR);

    void setTrigSpeed(float target_speed){

        if(isbacking)
            target_speed = -Tri_BACKING_SPEED;

        short tmp_cur[] = {0, 0, 0, 0};
        float motor_speed = m3508Group_triger.getMotorState(trig_id).speed;

        float error = target_speed - dir*motor_speed;
        if(error > 150.f){
            timeout++;
            if(timeout > Tri_BACKING_TIME){
                isbacking = true;
            }
        }
        else{
            if(timeout > 0)
                timeout--;
            else if(timeout == 0)
                isbacking = false;

        }

        Triger_PID.Run(dir*target_speed, motor_speed);
        tmp_cur[trig_id] = (short)Triger_PID.Output;
        m3508Group_triger.setMotorCurrent(tmp_cur);
    }

}