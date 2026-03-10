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

[[noreturn]] void DebugTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();
    using namespace RefereeType;


    while (true){

        auto sta = hDbus.getState();
//        supCap.SetRestEng(hreferee.getRefereeInfo<RefereeType::PowerHeatData>().chassis_power_buffer);

        float pack[5];
        pack[1] = hDbus.getState()->ch[2];
//        pack[0] = hreferee.getRefereeInfo<PowerHeatData>().reserved;
//        //pack[1] = hreferee.getRefereeInfo<PowerHeatData>().chassis_power_buffer;
//        pack[1] = supCap.getPower();
//        pack[2] = hreferee.getRefereeInfo<PowerHeatData>().reserved - supCap.getPower();
//        pack[0] = hreferee.getRefereeInfo<PowerHeatData>().shooter_id1_17mm_cooling_heat;
        pack[0] = hINS.getAngle().pitch * 180 / GimbalControl::PI;

        Debug::print_vofa(pack, 3);

        osDelay(50);
    }
}