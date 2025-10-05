#include "MotionControl.h"

namespace ChassisControl{

    constexpr float MAX_CUR = 16000.0f;

    DJiMotorGroup m3508Group_Chassis(&hcan1, 0x201, 0x200);

    DeltaPID Speed_PID[4] = {
            DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_CUR, -MAX_CUR)
    };

    constexpr void MotionCalMecanumForward(const MoveState& target_motion, float* motor_v_target) {
        motor_v_target[0] = target_motion.vx - target_motion.vy + target_motion.omega;
        motor_v_target[1] = -target_motion.vx - target_motion.vy + target_motion.omega;
        motor_v_target[2] = target_motion.vx - target_motion.vy - target_motion.omega;
        motor_v_target[3] = -target_motion.vx - target_motion.vy - target_motion.omega;
    }

    constexpr void MotionCalMecanumBackward(const float motor_v_target[4], MoveState* cla_motion) {

        cla_motion->vx = 0.25f * (motor_v_target[0] - motor_v_target[1] + motor_v_target[2] - motor_v_target[3]);
        cla_motion->vy = -0.25f * ( motor_v_target[0] + motor_v_target[1] + motor_v_target[2] + motor_v_target[3]);
        cla_motion->omega = 0.25f * ( motor_v_target[0] + motor_v_target[1] - motor_v_target[2] - motor_v_target[3]);
    }

    constexpr void MotionCalOmnidForward(const MoveState& target_motion, float* motor_v_target) {
        motor_v_target[0] = target_motion.vx + target_motion.vy + target_motion.omega;
        motor_v_target[1] = -target_motion.vx + target_motion.vy + target_motion.omega;
        motor_v_target[2] = -target_motion.vx - target_motion.vy + target_motion.omega;
        motor_v_target[3] = target_motion.vx - target_motion.vy + target_motion.omega;
    }

    constexpr void MotionCalOmnidBackward(const float motor_v_target[4], MoveState* cla_motion){
        cla_motion->vx = 0.25f * (motor_v_target[0] - motor_v_target[1] - motor_v_target[2] + motor_v_target[3]);
        cla_motion->vy = 0.25f * (motor_v_target[0] + motor_v_target[1] - motor_v_target[2] - motor_v_target[3]);
        cla_motion->omega = 0.25f * (motor_v_target[0] + motor_v_target[1] + motor_v_target[2] + motor_v_target[3]);
    }

    //实测omega=1980时，实际转动速度2rad/s左右(英雄机器人)
    // omega=900时，实际转动速度约1rad/s(云台跟随)
    MoveState setMove(MoveState target_state){

        short tmp_cur[] = {0, 0, 0, 0};

        float motor_v_target[4] = {};
        MotionCalOmnidForward(target_state, motor_v_target);

        float motor_speed[4] = {0,0,0,0};
        for (int i = 0; i < 4; ++i) {
            DJiMotorGroup::MotorState bf = m3508Group_Chassis.getMotorState(i);
            motor_speed[i] = bf.speed;
            Speed_PID[i].Run(motor_v_target[i], motor_speed[i]);
            tmp_cur[i] = (short)Speed_PID[i].Output;
//            tmp_cur[i] = 600;
        }

        MoveState measure_state = {};
        MotionCalOmnidBackward(motor_speed, &measure_state);
        m3508Group_Chassis.setMotorCurrent(tmp_cur);

        return measure_state;
    }
}
