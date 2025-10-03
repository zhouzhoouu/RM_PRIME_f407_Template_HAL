#include "MotionControl.h"

using namespace MotionFSM;

DJiMotorGroup m3508Group_frib(&hcan2, 0x205, 0x1ff);
DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);


void MotionControlTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();

    float taget_pos = GimbalControl::YawMotor.getMotorState().pos;

    fsm::sm<motionTransition> motion_fsm;

    while (1){

        const volatile DBus::RCState* sta = hDbus.getState();

        MotionFSM::LeverInput lin = {
                sta->s[1],
                sta->s[0]
        };

        motion_fsm.process_event(lin);


        if(motion_fsm.is(fsm::state<ChassisLead>)) {
            ChassisLeadLoop();
        }
        else if(motion_fsm.is(fsm::state<GimbalLead>)){
            GimbalLeadLoop();
        }
        else if(motion_fsm.is(fsm::state<AutoAim>)){
            AutoAimLoop();
        }
        else if(motion_fsm.is(fsm::state<AutoRotate>)){
            AutoRotateLoop();
        }
        else{
            IdleLoop();
        }

        osDelay(1);
    }
}



void DebugTask(void const * argument){

    DBus &hDbus = DBus::getInstance();
    INS_Device &hINS = INS_Device::getInstance();


    while (1){

        const volatile DBus::RCState* sta = hDbus.getState();

        short pack[4];


        pack[0] = sta->s[0];
        pack[1] = sta->s[1];

//        pack[0] = hINS.getAngle().yaw;
//        pack[1] = GimbalControl::YawMotor.getMotorState().pos;
//        pack[2] = hINS.getOmiga().yaw;
//        pack[3] = GimbalControl::YawMotor.getMotorState().vel;

        Debug::print_vofa<short>(pack, 2);

        osDelay(50);
    }
}