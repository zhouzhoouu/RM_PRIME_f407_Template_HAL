#ifndef F407_RM_TMPLATE_HAL_DM4310_H
#define F407_RM_TMPLATE_HAL_DM4310_H

#include "InterfaceCAN.h"

namespace Device {

    class DM4310 : public BSP::InterfaceCAN {
    public:

        enum StateCode {
            DISABLED = 0x0,
            ENABLED = 0x1,
            OVERVOLT = 0x8,
            UNDERVOLT = 0x9,
            OVERCURRENT = 0xA,
            MOS_OVERHEAT = 0xB,
            ROTOR_OVERHEAT = 0xC,
            COMM_LOST = 0xD,
            OVERLOAD = 0xE
        };

        struct MotorState {
            uint8_t id;             // 控制器id
            StateCode sta;            // 状态类型
            uint16_t pos_int;
            uint16_t vel_int;
            uint16_t tor_int;
            float pos;              // 位置信息
            float vel;              // 速度信息
            float tor;              // 扭矩信息
            float T_Mos;            // MOS管上的温度信息
            float T_Rotor;          // 电机线圈的温度信息
        };

        DM4310(CAN_HandleTypeDef *hc, uint32_t motorid);

        void init() override;
        bool receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len) override;

        void setMITcmd(float target_pos, float target_vel, float Kp, float Kd, float target_tor);
        void setPosZero();

    private:
        MotorState state;
        static int float_to_uint(float x_float, float x_min, float x_max, int bits);
        static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    };
}


#endif
