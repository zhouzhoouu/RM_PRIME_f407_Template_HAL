#include "ShootTask.h"

using namespace Device;

DJiMotorGroup m3508Group_frib(&hcan2, 0x201, 0x200);
DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);



[[noreturn]] void ShootTask(void const * argument){

    DBus &hDbus = DBus::getInstance();

    while (true){

        short curr[] = {hDbus.getState()->ch[4],
                        hDbus.getState()->ch[4],0, 0};
        m3508Group_frib.setMotorCurrent(curr);

        osDelay(1);

    }

}