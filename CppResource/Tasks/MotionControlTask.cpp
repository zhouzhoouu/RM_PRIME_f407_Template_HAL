#include "TaskList.h"
#include "DJiMotorGroup.h"
#include "INS_Device.h"
#include "DM4310.h"
#include "DeltaPID.h"
#include "DBus.h"

using namespace Device;
using namespace Component;

DJiMotorGroup m3508Group_frib(&hcan2, 0x205, 0x1ff);
DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);


namespace ChassisControl{

    struct MoveState{
        union {
            struct {
                float vx;
                float vy;
                float omega;
            };
            float data[3];
        };
    };

    DJiMotorGroup m3508Group_Chassis(&hcan1, 0x201, 0x200);

    DeltaPID Speed_PID[4] = {
            DeltaPID(1.0f, 0.01f, 1.0f, 0.0f, 16383.0f, -16383.0f),
            DeltaPID(1.0f, 0.01f, 1.0f, 0.0f, 16383.0f, -16383.0f),
            DeltaPID(1.0f, 0.01f, 1.0f, 0.0f, 16383.0f, -16383.0f),
            DeltaPID(1.0f, 0.01f, 1.0f, 0.0f, 16383.0f, -16383.0f)
    };

    //实测omega=1980时，实际转动速度2rad/s左右
    inline static MoveState setMove(MoveState target_state){

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
            //tmp_cur[i] = 600;
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

namespace GimbalControl{

    static float constexpr PI = 3.141592653f;

    DM4310 YawMotor(&hcan1, 2);
    DM4310 PithMotor(&hcan2, 1);

    DeltaPID anglePID_Yaw = DeltaPID(8.0f, 0.0f, 800.0f, 0.0f, 7, -7);

    //将角度规范化到 -PI 到 PI
    inline static float angleMod(float angle){
        while (angle > PI) angle -= 2.f * PI;
        while (angle < -PI) angle += 2.f * PI;
        return angle;
    }

    inline static void setYawRelative(float angle, float omega_forward = 0.f){
        float angle_err = angleMod(angle - YawMotor.getMotorState().pos);
        anglePID_Yaw.Run(0, angle_err);
        YawMotor.setMITcmd(0,omega_forward,0,1,angle_err*10.f);
    }

    inline static void setPithRelative(float angle, float omega_forward = 0.f){
        PithMotor.setMITcmd(angle,omega_forward,8,1,0);
    }

}

void MotionControlTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();

    float taget_pos = GimbalControl::YawMotor.getMotorState().pos;


    while (1){

        const volatile DBus::RCState* sta = hDbus.getState();

        ChassisControl::MoveState target_state = {
                (float)sta->ch[3] * 3.f,
                (float)sta->ch[2] * 3.f,
                (float)sta->ch[0] * 3.f
        };

        float ref_ang = -(float)sta->ch[1] * (1/1000.f);

        ChassisControl::MoveState Chassis_bf = ChassisControl::setMove(target_state);

        taget_pos = GimbalControl::YawMotor.getMotorState().pos - hINS.getAngle().yaw;

        GimbalControl::setYawRelative(taget_pos, Chassis_bf.omega * 0.001f);
        GimbalControl::setPithRelative(ref_ang);

        osDelay(1);
    }
}



void DebugTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    const volatile DBus::RCState* sta = hDbus.getState();
    INS_Device &hINS = INS_Device::getInstance();

    while (1){

        float pack[4];

        pack[0] = hINS.getAngle().yaw;
        pack[1] = GimbalControl::YawMotor.getMotorState().pos;

        pack[2] = hINS.getOmiga().yaw;
        pack[3] = GimbalControl::YawMotor.getMotorState().vel;

        Debug::print_vofa(pack, 4);

        osDelay(50);
    }
}