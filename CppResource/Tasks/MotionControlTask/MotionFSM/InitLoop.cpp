#include "MotionFSM.h"
using namespace MotionFSM;

static Referee &hreferee = Referee::getInstance();

StateLoopArg MotionFSM::InitLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg& cur_sta){
    InitFlag.InitNI = false;

    bool gimbal_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
    bool chassis_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;
    auto sta = HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin);


    if(!(chassis_on && gimbal_on))
    {

    }else{
        InitFlag.InitNI = false;
    }


    return {
            {{0,0,0}},
            {{cur_sta.YawSta.pos,0}},
            cur_sta.PithSta,
            cur_sta.taget_pitch
    };
}