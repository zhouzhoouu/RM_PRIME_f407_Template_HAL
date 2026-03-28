#include "TaskList.h"
#include "Referee.h"


using namespace Device;

[[noreturn]] void RefereeProcessTask(void const * argument){

    Referee &href = Referee::getInstance();

    while (true){
        href.ProcessData();

        bool gimbal_on = href.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
        bool frib_on = href.getRefereeInfo<RefereeType::GameRobotState>().power_management_shooter_output;

        if(!href.RefereeExist()){
//            gimbal_on = true;
//            frib_on = true;
            gimbal_on = false;
            frib_on = false;
        }

        HAL_GPIO_WritePin(Frib_Relay_GPIO_Port, Frib_Relay_Pin, frib_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Pitch_Relay_GPIO_Port, Pitch_Relay_Pin, gimbal_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

        osDelay(2);
    }
}