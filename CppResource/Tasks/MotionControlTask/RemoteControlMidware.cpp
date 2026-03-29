#include "MotionControl.h"
#include "MotionFSM.h"

namespace RemoteContrlMidware{

    VT03& hVT03 = VT03::getInstance();
    DBus& hDBus = DBus::getInstance();

    static fsm::sm<MotionFSM::motionTransition> motion_fsm;
    Referee &hreferee = Referee::getInstance();

    void setMODE_SW(uint8_t mode){
        if(mode == 0)
            motion_fsm.process_event(MotionFSM::IntoIdle{});
        else if(mode == 1)
            motion_fsm.process_event(MotionFSM::IntoChassisLead{});
        else if(mode == 2)
            motion_fsm.process_event(MotionFSM::IntoGimbalLead{});
    }

    VirtualRC_CMD DBus_input(){

        using namespace MotionFSM;

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

        static bool odmode;
        static VT03::RCState last_sta;
        const VT03::RCState *sta = hVT03.getState();

        bool autorotate_b = (sta->pause && !last_sta.pause) ||
                          (BITMASK(sta->key_code, 6) && !BITMASK(last_sta.key_code, 6));
        if(autorotate_b){
            if(!motion_fsm.is(fsm::state<Idle>))
            {
                if(motion_fsm.is(fsm::state<AutoRotate>))
                    setMODE_SW(sta->mode_sw);
                else
                    motion_fsm.process_event(IntoAutoRotate{});
            }
        }

        bool mode_sw_cond = (sta->mode_sw != last_sta.mode_sw)||
                (motion_fsm.is(fsm::state<Idle>) && sta->mode_sw != 0);
        if(mode_sw_cond)\

            setMODE_SW(sta->mode_sw);

        if(sta->fn_2 && !last_sta.fn_2)
        {
            odmode = !odmode;
        }

        if(motion_fsm.is(fsm::state<GimbalLead>)||motion_fsm.is(fsm::state<AutoRotate>)){
            bool auto_aim_b = (sta->fn_1 && !last_sta.fn_1)||
                    (sta->mouse_r && !last_sta.mouse_r)||
                    ((BITMASK(sta->key_code, 7) && !BITMASK(last_sta.key_code, 7)));
            if(auto_aim_b)
            {
                AutoAimOn = !AutoAimOn;
            }
        } else{
            AutoAimOn = false;
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
                static_cast<bool>(odmode | BITMASK(sta->key_code, 4))
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
//                return VT03_input();// + DBus_input();
            }
        }

        return {{0,0,0,0}, false};

    }

}