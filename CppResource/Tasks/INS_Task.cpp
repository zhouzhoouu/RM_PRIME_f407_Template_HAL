#include "TaskList.h"
#include "Debug.h"
#include "BMI088.h"
#include "Simple_IIR3.h"
#include "AHRS/AHRS_middleware.h"
#include "AHRS/AHRS.h"

using namespace Device;
using namespace Component;
//using namespace BSP;

static TickType_t INS_LastWakeTime;

float INS_angle_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};
float INS_accel[3] = {0.0f, 0.0f, 0.0f};
float INS_mag[3] = {0.0f, 0.0f, 0.0f};



BMI088::Measurement tmp;

static const float Filter_num[][4] = {{0.002329458745586203f, 0.f, 0.f, 0.f},{1.f, -1.929454039488895f, 0.93178349823448126f, 0.f}};
Simple_IIR3 acc_filter[3] = {
        Simple_IIR3(Filter_num[0],Filter_num[1]),
            Simple_IIR3(Filter_num[0],Filter_num[1]),
            Simple_IIR3(Filter_num[0],Filter_num[1]),
};

void INS_Task(void const * argument){

    INS_LastWakeTime = xTaskGetTickCount();

    AHRS_init(INS_angle_quat, INS_accel, INS_mag);
    BMI088 &hIMU = BMI088::getInstance();



    while (1){

        tmp = hIMU.GetMeasurement();

        INS_accel[0] = acc_filter[0].processSample(tmp.acc[0]);
        INS_accel[1] = acc_filter[1].processSample(tmp.acc[1]);
        INS_accel[2] = acc_filter[2].processSample(tmp.acc[2]);



        AHRS_update(INS_angle_quat, 0.001f, tmp.gyro, INS_accel, INS_mag);
        get_angle(INS_angle_quat, INS_angle, INS_angle + 1, INS_angle + 2);


        vTaskDelayUntil(&INS_LastWakeTime, 1);
    }
}


//void DebugTask(void const * argument){
//
//    while (1){
//
//        float pack[3];
//        pack[0] = acc_filter[0].getOutput();
//        pack[1] = acc_filter[1].getOutput();
//        pack[2] = acc_filter[2].getOutput();
//
//        Debug::print_vofa(INS_angle, 3);
//        //Debug::print_vofa(tmp.acc, 3);
//
//        osDelay(50);
//    }
//}