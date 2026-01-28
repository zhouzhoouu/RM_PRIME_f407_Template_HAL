#include "MotionControl.h"
#include "MotionFSM.h"

using namespace MotionFSM;



[[noreturn]] void MotionControlTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();

    osDelay(200);
    fsm::sm<motionTransition> motion_fsm;
    MotionFSM::StateLoopArg input_sta{
        ChassisControl::setMove({{0,0,0}}),
        GimbalControl::getYawState(),
        GimbalControl::getPithState()
    };

    while (true){

        const volatile DBus::RCState* sta = hDbus.getState();

        uint8_t linLeft = sta->s[1];
        uint8_t linRight = sta->s[0];
        static short last_sta;
        short sta_check = (short)(linLeft) << 8 | linRight;

        if(InitFlag.InitNI){

        }
        else{
            if(motion_fsm.is(fsm::state<Init>))
                motion_fsm.process_event(InitComplete{});
            else if(last_sta != sta_check){
                last_sta = sta_check;
                if(linLeft == 1)motion_fsm.process_event(IntoIdle{});
                else if(linLeft == 3)motion_fsm.process_event(IntoChassisLead{});
                else if(linLeft==2 && linRight==3)motion_fsm.process_event(IntoGimbalLead{});
                else if(linLeft==2 && linRight==1)motion_fsm.process_event(IntoAutoAim{});
                else if(linLeft==2 && linRight==2)motion_fsm.process_event(IntoAutoRotate{});
            }
        }


        input_sta.YawSta = GimbalControl::getYawState();
        input_sta.PithSta = GimbalControl::getPithState();

        auto output_sta = MotionFSM::CurrentHandler(sta, hINS, input_sta);

        GimbalControl::setYawRelative(output_sta.YawSta);
        GimbalControl::setPithRelative(output_sta.PithSta);
        input_sta.ChassisSta = ChassisControl::setMove(output_sta.ChassisSta);

        osDelay(1);
    }
}


//SupCap supCap(&hcan1);
Referee &hreferee = Referee::getInstance();

[[noreturn]] void DebugTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();
    using namespace RefereeType;


    while (true){

//        const volatile DBus::RCState* sta = hDbus.getState();
////        supCap.SetRestEng(hreferee.getRefereeInfo<RefereeType::PowerHeatData>().chassis_power_buffer);
//
//        float pack[5];
//        pack[0] = hDbus.getState()->ch[2];
////        pack[0] = hreferee.getRefereeInfo<PowerHeatData>().reserved;
////        //pack[1] = hreferee.getRefereeInfo<PowerHeatData>().chassis_power_buffer;
////        pack[1] = supCap.getPower();
////        pack[2] = hreferee.getRefereeInfo<PowerHeatData>().reserved - supCap.getPower();
////        pack[0] = hreferee.getRefereeInfo<PowerHeatData>().shooter_id1_17mm_cooling_heat;
//
//        Debug::print_vofa(pack, 3);

        osDelay(50);
    }
}
