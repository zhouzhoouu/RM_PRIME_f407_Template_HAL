#include "MotionControl.h"
#include "AHRS_middleware.h"

using namespace MotionParameter;

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
        // 直接用代数表达式（等价于对 (vx,vy) 逆时针旋转 +90° 后的映射）
        motor_v_target[0] = target_motion.vx - target_motion.vy + target_motion.omega;
        motor_v_target[1] = target_motion.vx + target_motion.vy + target_motion.omega;
        motor_v_target[2] = -target_motion.vx + target_motion.vy + target_motion.omega;
        motor_v_target[3] = -target_motion.vx - target_motion.vy + target_motion.omega;
    }

    constexpr void MotionCalOmnidBackward(const float motor_v_target[4], MoveState* cla_motion){
        // 直接用代数逆解（不使用中间变量），保证与前向映射互为逆运算
        float m0 = motor_v_target[0];
        float m1 = motor_v_target[1];
        float m2 = motor_v_target[2];
        float m3 = motor_v_target[3];

        cla_motion->vx = 0.25f * (m0 + m1 - m2 - m3);
        cla_motion->vy = 0.25f * (m1 + m2 - m0 - m3);
        cla_motion->omega = 0.25f * (m0 + m1 + m2 + m3);
    }

    //实测omega=1980时，实际转动速度2rad/s左右(英雄机器人)
    // omega=900时，实际转动速度约1rad/s(云台跟随)
    MoveState setMove(MoveState target_state){


        MoveState measure_state{};
        float motor_speed[4] = {0,0,0,0};
        for (int i = 0; i < 4; ++i) {
            DJiMotorGroup::MotorState bf = m3508Group_Chassis.getMotorState(i);
            motor_speed[i] = bf.speed;
        }
        MotionCalOmnidBackward(motor_speed, &measure_state);

        MoveState dv{{target_state.vx - measure_state.vx,
                   target_state.vy - measure_state.vy,
                   target_state.omega - measure_state.omega}};

        float vsq = dv.vx * dv.vx + dv.vy * dv.vy;
        float k_ref = CHASSIS_MAX_ACCEL*T_SAMPLE*AHRS_invSqrt(vsq);
        float k_omega_ref = CHASSIS_MAX_ALPHA*T_SAMPLE / (dv.omega > 0 ? dv.omega : -dv.omega + 0.0001f);
        if (k_ref < 1.0f ) {
            dv.vx *= k_ref;
            dv.vy *= k_ref;
        }
        if(k_omega_ref < 1.0f){
            dv.omega *= k_omega_ref;
        }
        MoveState target_state_accel_limit{{measure_state.vx + dv.vx,
                                 measure_state.vy + dv.vy,
                                 measure_state.omega + dv.omega}};

        float motor_v_target[4] = {};
        MotionCalOmnidForward(target_state_accel_limit, motor_v_target);

        short tmp_cur[] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            Speed_PID[i].Run(motor_v_target[i], motor_speed[i]);
            tmp_cur[i] = (short)Speed_PID[i].Output;
            //tmp_cur[i] = 600;
        }

        m3508Group_Chassis.setMotorCurrent(tmp_cur);

        return measure_state;
    }
}
