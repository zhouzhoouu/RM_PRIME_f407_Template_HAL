#include "ShootTask.h"

using namespace ShootFSM;
using namespace FribControl;
using namespace TriggerControl;

[[noreturn]] void ShootTask(void const * argument){

    DBus &hDbus = DBus::getInstance();

    while (true){

        short tin = hDbus.getState()->ch[4];
        tin = -tin;

        ShootFSMLoop(tin);
        bool isFribOpened = getIsFribOpened();
        bool isZeroCross = getIsZeroCross();

        float triger_speed, frib_speed;
        if(isFribOpened) {
            if(tin < 0) tin = 0;

            if(isZeroCross)
                triger_speed = 1.f*tin;
            else
                triger_speed = 0.f;

            frib_speed = 2300;//7300.f;
        }
        else{
            triger_speed = 0.f;
            frib_speed = 0.f;
        }

        setFribSpeed(frib_speed);
        setTrigSpeed(triger_speed);



        osDelay(1);

    }

}