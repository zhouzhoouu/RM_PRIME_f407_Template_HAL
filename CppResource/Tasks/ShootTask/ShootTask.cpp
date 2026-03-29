#include "ShootTask.h"
#include "Referee.h"

//#define USE_WHEEL_CTRL_

static uint32_t shootcount = 0;
[[noreturn]] void ShootTask(void const * argument){

    auto &hDbus = DBus::getInstance();
    auto &hVT03 = VT03::getInstance();
    auto &hreferee = Referee::getInstance();

    while (true){

        bool booster_power = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_shooter_output;
        TriggerControl::NotifyPowerSate(booster_power);
        FribControl::NotifyPowerSate(booster_power);
        if(!booster_power) ShootFSM::FSM_Reset();

#ifdef USE_WHEEL_CTRL_
        short tin = -hDbus.getState()->ch[4] + hVT03.getState()->wheel;

        ShootFSM::ShootFSM_Wheel_Loop(tin);
        bool isFribOpened = ShootFSM::getIsFribOpened();
        bool isZeroCross = ShootFSM::getIsZeroCross();

        float triger_speed, frib_speed;
        if(isFribOpened) {
            if(tin < 0) tin = 0;

            if(isZeroCross)
                triger_speed = 6.f*tin;
            else
                triger_speed = 0.f;


            frib_speed = 7000;//7000.f;
        }
        else{
            frib_speed = 0.f;
            triger_speed = 0.f;
            shootcount = SHOOT_GAP_MS;
        }

        FribControl::setFribSpeed(frib_speed);
        TriggerControl::setSpeed(triger_speed);
#else

        auto vt_sta = hVT03.getState();
        auto x = vt_sta->trigger | vt_sta->mouse_l;
        bool b = (vt_sta->fn_2 && vt_sta->trigger)||vt_sta->mouse_middle;
        ShootFSM::ShootFSM_Button_Loop(b);

        bool isFribOpened = ShootFSM::getIsFribOpened();
        bool isZeroCross = ShootFSM::getIsZeroCross();
        float frib_speed = 0.f;

        if(isFribOpened) {

            if(x && isZeroCross){
                shootcount++;
                if(shootcount > SHOOT_GAP_MS)
                {
                    shootcount = 0;
                    TriggerControl::AddStep();
                }
            } else{
                shootcount = SHOOT_GAP_MS;
            }
            frib_speed = 6000.f;
        }

        FribControl::setFribSpeed(frib_speed);
        TriggerControl::Loop();
#endif

        osDelay(1);

    }

}

