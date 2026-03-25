#include "MotionControl.h"
#include "MotionFSM.h"

#define BITMASK(a,n) (( (a)&(1<<n) )>>n)

namespace RemoteContrlMidware{


    static fsm::sm<MotionFSM::motionTransition> motion_fsm;
    Referee &hreferee = Referee::getInstance();

    void setMODE_SW(uint8_t mode){
        if(mode == 0)
            motion_fsm.process_event(MotionFSM::IntoChassisLead{});
        else if(mode == 1)
            motion_fsm.process_event(MotionFSM::IntoGimbalLead{});
        else if(mode == 2)
            motion_fsm.process_event(MotionFSM::IntoAutoAim{});
    }

    VirtualRC_CMD DBus_input(){

        using namespace MotionFSM;

        DBus& hDBus = DBus::getInstance();

        auto sta = hDBus.getState();

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

        return {{sta->ch[0], sta->ch[1], sta->ch[2], sta->ch[3]}, false};
    }

    VirtualRC_CMD VT03_input(){

        using namespace MotionFSM;

        VT03& hVT03 = VT03::getInstance();

        static VT03::RCState last_sta;

        const VT03::RCState *sta = hVT03.getState();

        if(sta->pause && !last_sta.pause){

            if(motion_fsm.is(fsm::state<Idle>))
                setMODE_SW(sta->mode_sw);
            else
                motion_fsm.process_event(PausePress{});
        }

        if(sta->mode_sw != last_sta.mode_sw)
            setMODE_SW(sta->mode_sw);

        if(sta->fn_1 && !last_sta.fn_1)
        {
            if(motion_fsm.is(fsm::state<AutoRotate>))
                setMODE_SW(sta->mode_sw);
            else
                motion_fsm.process_event(IntoAutoRotate{});
        }

        short k_move = 300;
        if(BITMASK(sta->key_code, 4)) k_move = 660;
        if(BITMASK(sta->key_code, 5)) k_move = 100;

        short v_x = sta->ch[3] + (BITMASK(sta->key_code, 3) - BITMASK(sta->key_code, 2))*k_move;
        short v_y = sta->ch[2] + (BITMASK(sta->key_code, 0) - BITMASK(sta->key_code, 1))*k_move;
        short v_omega = sta->ch[0] + sta->mouse_x * 3;
        short v_pitch = sta->ch[1] - sta->mouse_y * 3;

        last_sta = *sta;
        return {{
                v_omega,
                v_pitch,
                v_x,
                v_y},
                static_cast<bool>(sta->trigger | BITMASK(sta->key_code, 4))
        };
    }

    void RemoteContrlMidwareInit(){

    }

    VirtualRC_CMD RemoteContrlMidwareLoop(){
        using namespace MotionFSM;

        bool gimbal_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
        bool chassis_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;

        if(hreferee.RefereeExist()==false){
            gimbal_on = true;
            chassis_on = true;
        }

        GimbalControl::NotifyPowerSate(gimbal_on);
        ChassisControl::NotifyPowerSate(chassis_on);

        if(MotionFSM::InitFlag.InitNI){

        }
        else{
            if(motion_fsm.is(fsm::state<Init>))
                motion_fsm.process_event(InitComplete{});
            else {
                if(!(chassis_on && gimbal_on))
                    motion_fsm.process_event(PowerOff{});
                return VT03_input() + DBus_input();
            }
        }

        return {{0,0,0,0}, false};

    }

}