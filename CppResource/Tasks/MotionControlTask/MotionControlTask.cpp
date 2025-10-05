#include "MotionControl.h"

using namespace MotionFSM;

DJiMotorGroup m3508Group_frib(&hcan2, 0x205, 0x1ff);
DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);


void MotionControlTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();

    fsm::sm<motionTransition> motion_fsm;
    MotionFSM::StateLoopArg input_sta{
        ChassisControl::setMove({{0,0,0}}),
        GimbalControl::getYawState(),
        GimbalControl::getPithState()
    };

    while (1){

        const volatile DBus::RCState* sta = hDbus.getState();

        uint8_t linLeft = sta->s[1];
        uint8_t linRight = sta->s[0];
        static short last_sta;
        short sta_check = (short)(linLeft) << 8 | linRight;
        if(last_sta != sta_check){
            last_sta = sta_check;
            if(linLeft == 1)motion_fsm.process_event(IntoIdle{});
            else if(linLeft == 3)motion_fsm.process_event(IntoChassisLead{});
            else if(linLeft==2 && linRight==3)motion_fsm.process_event(IntoGimbalLead{});
            else if(linLeft==2 && linRight==1)motion_fsm.process_event(IntoAutoAim{});
            else if(linLeft==2 && linRight==2)motion_fsm.process_event(IntoAutoRotate{});
        }

        auto output_sta = MotionFSM::CurrentHandler(sta, hINS, input_sta);

        GimbalControl::setYawRelative(output_sta.YawSta);
        GimbalControl::setPithRelative(output_sta.PithSta);
        input_sta.ChassisSta = ChassisControl::setMove(output_sta.ChassisSta);

        osDelay(1);
    }
}



void DebugTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();


    while (1){

        const volatile DBus::RCState* sta = hDbus.getState();

        float pack[5];


//        pack[0] = sta->s[0];
//        pack[1] = sta->s[1];

        pack[0] = hINS.getAngle().yaw;
        pack[1] = GimbalControl::angleMod(GimbalControl::getYawState().pos);
        pack[2] = hINS.getOmiga().yaw;
        pack[3] = GimbalControl::getYawState().omega;
        pack[4] = GimbalControl::YawMotor.getMotorState().pos;

        Debug::print_vofa(pack, 5);

        osDelay(50);
    }
}