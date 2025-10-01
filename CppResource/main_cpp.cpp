#include "main_cpp.h"
#include "UART.h"
#include "InterfaceCAN.h"
#include "BMI088.h"


void app_init_function(void) {

    BSP::CAN_UserInit();
    BSP::UART_UserInit();

    Device::BMI088 &hIMU = Device::BMI088::getInstance();
    hIMU.init();

}

