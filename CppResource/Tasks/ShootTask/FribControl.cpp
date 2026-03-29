#include "ShootTask.h"

namespace FribControl{

    static bool isPowerOn = false;
    DJiMotorGroup m3508Group_frib(&hcan2, 0x201, 0x200);
    DeltaPID Frib_PID[] = {
            DeltaPID(3.0f, 0.07f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR),
            DeltaPID(3.0f, 0.07f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR),
    };


    void NotifyPowerSate(bool s){
        if(isPowerOn && !s){
            for(auto & i : Frib_PID){
                i.Reset();
            }
        }
        isPowerOn = s;
    }

    void setFribSpeed(float target_speed){
        if(!isPowerOn) return;

        float target_speed_arr[] = {target_speed, -target_speed};

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
