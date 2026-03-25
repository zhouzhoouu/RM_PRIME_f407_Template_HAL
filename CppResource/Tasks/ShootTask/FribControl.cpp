#include "ShootTask.h"

namespace FribControl{

    DJiMotorGroup m3508Group_frib(&hcan2, 0x205, 0x1FF);
    DeltaPID Frib_PID[] = {
            DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR),
            DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR),
    };


    void setFribSpeed(float target_speed){

        float target_speed_arr[] = {-target_speed, target_speed};

        short tmp_cur[] = {0, 0, 0, 0};
        for (int i = 0; i < 2; ++i) {
            float motor_speed = m3508Group_frib.getMotorState(i).speed;
            Frib_PID[i].Run(target_speed_arr[i], motor_speed);
            tmp_cur[i] = (short)Frib_PID[i].Output;
            //tmp_cur[i] = 600;
        }
        m3508Group_frib.setMotorCurrent(tmp_cur);
    }


}
