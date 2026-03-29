#include "MotionFSM.h"
using namespace MotionFSM;

static Referee &hreferee = Referee::getInstance();
static uint32_t initcount = 0;
StateLoopArg MotionFSM::InitLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg& cur_sta){
    InitFlag.InitNI = true;

    bool gimbal_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
    bool chassis_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;
//    auto sta = HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin);

    if(hreferee.RefereeExist()==false){
        gimbal_on = true;
        chassis_on = true;
    }


    if(!(chassis_on && gimbal_on))
    {
        initcount = 0;
    }else{
        if(initcount < 500){
            initcount++;
        } else
        {
            InitFlag.InitNI = false;
        }

    }


    return {
            {{0,0,0}},
            {{cur_sta.YawSta.pos,0}},
            cur_sta.PithSta,
            cur_sta.taget_pitch
    };
}