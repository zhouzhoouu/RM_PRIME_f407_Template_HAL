#include "MotionFSM.h"
#include "math.h"
using namespace MotionFSM;

static Referee &hreferee = Referee::getInstance();
static auto last_sta = GPIO_PIN_RESET;

StateLoopArg MotionFSM::InitLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg& cur_sta){


    bool gimbal_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
    bool chassis_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;
    auto sta = HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin);


    if(!(chassis_on && gimbal_on))
    {
        last_sta = sta;
        return {
                {{0,0,0}},
                {{cur_sta.YawSta.pos,0}},
                cur_sta.PithSta,
                cur_sta.taget_pitch
        };
    } else if(sta == GPIO_PIN_SET && last_sta == GPIO_PIN_RESET){

        float rad_bias = GimbalControl::PI*(1/180.f) * (21.f - 90); //10度偏差
        GimbalControl::YawZero = GimbalControl::angleMod(  rad_bias+GimbalControl::getYawState().pos);
        InitFlag.InitNI = false;
    }

    last_sta = sta;

    return {
        {{0,0,0}},
        {{cur_sta.YawSta.pos,5}},
            cur_sta.PithSta,
            cur_sta.taget_pitch
    };
}