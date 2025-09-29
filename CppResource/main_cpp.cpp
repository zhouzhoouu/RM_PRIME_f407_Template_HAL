#include "main_cpp.h"
#include "DJiMotorGroup.h"
#include "DM4310.h"
#include "DeltaPID.h"
#include "Debug.h"
#include "RGB_LED.h"

Device::DJiMotorGroup m3508Group_Chassis(&hcan1, 0x201, 0x200);
Device::DJiMotorGroup m3508Group_frib(&hcan2, 0x204, 0x1ff);
Device::DJiMotorGroup m3508Group_triger(&hcan1, 0x204, 0x1ff);
Device::DM4310 YawMotor(&hcan1, 2);
Device::DM4310 PithMotor(&hcan2, 1);

Component::DeltaPID M3508_Speed_PID(9.0f, 0.2f, 30.0f, 0.0f, 16384.0f, -16384.0f);


void app_init_function(void) {

    BSP::CAN_UserInit();

}


void TestTask(void const * argument){

    uint32_t col = 0;

    Device::RGB_LED &hLED = Device::RGB_LED::getInstance();

    hLED.setColor(0xFFFF00FF);


    while (1){

        short refc = (int16_t)M3508_Speed_PID.Output;
        int16_t cur[] = {refc, 0, 0, 0};
        m3508Group_Chassis.setMotorCurrent(cur);

        M3508_Speed_PID.Run(500.f, m3508Group_Chassis.getMotorState(0)->speed);


        if(col)
            hLED.setColor(0xFFFF00FF);
        else
            hLED.setColor(0xFF000000);
        col = !col;

        osDelay(1);

    }

}


void DebugTask(void const * argument){

    while (1){
        int16_t pack[3];
        pack[0] = m3508Group_Chassis.getMotorState(0)->angle;
        pack[1] = m3508Group_Chassis.getMotorState(0)->speed;
        pack[2] = m3508Group_Chassis.getMotorState(0)->current;

        Debug::print_vofa<short>(pack, 3);

        osDelay(50);
    }
}