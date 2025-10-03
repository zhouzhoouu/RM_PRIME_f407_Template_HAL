#include "MotionControl.h"

namespace ChassisControl{

    constexpr float MAX_CUR = 16000.0f;

    DJiMotorGroup m3508Group_Chassis(&hcan1, 0x201, 0x200);

    DeltaPID Speed_PID[4] = {
            DeltaPID(4.0f, 0.1f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(4.0f, 0.1f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(4.0f, 0.1f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(4.0f, 0.1f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR)
    };

    //实测omega=1980时，实际转动速度2rad/s左右
    MoveState setMove(MoveState target_state){

        short tmp_cur[] = {0, 0, 0, 0};
        float motor_v_target[4] = {
                target_state.vx - target_state.vy + target_state.omega,
                -target_state.vx - target_state.vy + target_state.omega,
                target_state.vx - target_state.vy - target_state.omega,
                -target_state.vx - target_state.vy - target_state.omega
        };
        float motor_speed[4] = {0,0,0,0};

        for (int i = 0; i < 4; ++i) {
            DJiMotorGroup::MotorState bf = m3508Group_Chassis.getMotorState(i);
            motor_speed[i] = bf.speed;
            Speed_PID[i].Run(motor_v_target[i], motor_speed[i]);
            tmp_cur[i] = (short)Speed_PID[i].Output;
//            tmp_cur[i] = 600;
        }

        MoveState measure_state = {
                0.25f * (motor_speed[0] - motor_speed[1] + motor_speed[2] - motor_speed[3]),
                -0.25f * ( motor_speed[0] + motor_speed[1] + motor_speed[2] + motor_speed[3]),
                0.25f * ( motor_speed[0] + motor_speed[1] - motor_speed[2] - motor_speed[3])
        };

        m3508Group_Chassis.setMotorCurrent(tmp_cur);

        return measure_state;
    }
}
