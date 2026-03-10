#include "MotionControl.h"
#include "MotionFSM.h"

using namespace MotionFSM;

[[noreturn]] void MotionControlTask(void const * argument){

    INS_Device &hINS = INS_Device::getInstance();
    RemoteContrlMidware::RemoteContrlMidwareInit();

    osDelay(200);
    MotionFSM::StateLoopArg input_sta{
        ChassisControl::setMove({{0,0,0}}),
        GimbalControl::getYawState(),
        GimbalControl::getPithState(),
        0.f
    };

    while (true){

        auto CMD = RemoteContrlMidware::RemoteContrlMidwareLoop();

        input_sta.YawSta = GimbalControl::getYawState();
        input_sta.PithSta = GimbalControl::getPithState();

        auto output_sta = MotionFSM::CurrentHandler(&CMD, hINS, input_sta);

        GimbalControl::setYawRelative(output_sta.YawSta);
        GimbalControl::setPithRelative(output_sta.PithSta);
        input_sta.ChassisSta = ChassisControl::setMove(output_sta.ChassisSta);

        if(MotionParameter::GIMBAL_PITCH_LOWER > output_sta.taget_pitch) output_sta.taget_pitch = MotionParameter::GIMBAL_PITCH_LOWER;
        else if(MotionParameter::GIMBAL_PITCH_UPPER < output_sta.taget_pitch) output_sta.taget_pitch = MotionParameter::GIMBAL_PITCH_UPPER;
        else input_sta.taget_pitch = output_sta.taget_pitch;

        osDelay(2);
    }
}


//SupCap supCap(&hcan1);

Referee &hreferee = Referee::getInstance();
#include "ui.h"

[[noreturn]] void DebugTask(void const * argument){

//    VT03 &hVT03 = VT03::getInstance();
//    HAL_UART_Init(&huart6);

    ui_remove_g_Ungroup();
    ui_init_g_Ungroup();

    while (true){
//        if(ui_g_Ungroup_engbar->end_x > ui_g_Ungroup_engbar->start_x)
//            ui_g_Ungroup_engbar->end_x--;
        ui_g_Ungroup_engbar->start_x = 500;
        ui_update_g_Ungroup();

        float pack[5];

        pack[0] = hreferee.getRefereeInfo<RefereeType::PowerHeatData>().chassis_power_buffer;
        pack[1] = ChassisControl::GetPower();
        pack[2] = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
        pack[3] = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;
        pack[4] = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_shooter_output;

        Debug::print_vofa(pack, 5);

        osDelay(50);
    }
}