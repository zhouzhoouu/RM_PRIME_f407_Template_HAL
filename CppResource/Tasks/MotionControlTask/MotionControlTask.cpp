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


[[noreturn]] void DebugTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();


    while (true){

        const volatile DBus::RCState* sta = hDbus.getState();

        float pack[5];


//        pack[0] = sta->s[0];
//        pack[1] = sta->s[1];

        pack[0] = hINS.getAngle().yaw;
        pack[1] = GimbalControl::angleMod(GimbalControl::getYawState().pos);
        pack[2] = GimbalControl::getYawState().omega;
//        pack[2] = hINS.getAccel().x;
//        pack[3] = hINS.getAccel().y;

        Debug::print_vofa(pack, 3);

        osDelay(50);
    }
}