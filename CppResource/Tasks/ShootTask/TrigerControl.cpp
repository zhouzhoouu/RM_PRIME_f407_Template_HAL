#include "ShootTask.h"

namespace TriggerControl{

    constexpr float i_tor_limit = 16000.f;

    uint32_t timeout = 0;
    int dir = -1;
    constexpr uint32_t trig_id = 2;
    bool isbacking = false;

    float last_error = 0;
    float integral = 0;

    DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);

    void setTrigSpeed(float target_speed){

        if(isbacking)
            target_speed = -Tri_BACKING_SPEED;

        short tmp_cur[] = {0, 0, 0, 0};
        float motor_speed = m3508Group_triger.getMotorState(trig_id).speed;

        float ref = target_speed - dir*motor_speed;
        if(ref > 200 && !isbacking){
            timeout++;
            if(timeout > Tri_BACKING_TIME){
                isbacking = true;
                integral = 0;
            }
        }
        else{
            if(timeout > 0)
                timeout--;
            else if(timeout == 0)
                isbacking = false;

        }

        if(ref < -500){
            integral = 0;
        }

        if(timeout < 0)
            timeout = 0;



        float error = dir*target_speed - motor_speed;
        integral += error;
        float i_tor = integral * 0.01;

        if(i_tor > i_tor_limit)
            i_tor = i_tor_limit;
        else if(i_tor < -i_tor_limit)
            i_tor = -i_tor_limit;


        float exp_i = error * 2. + i_tor;

        if(exp_i > MAX_Tri_CUR)
            exp_i = MAX_Tri_CUR;
        else if(exp_i < -MAX_Tri_CUR)
            exp_i = -MAX_Tri_CUR;

        tmp_cur[trig_id] = exp_i;
        m3508Group_triger.setMotorCurrent(tmp_cur);
    }

}