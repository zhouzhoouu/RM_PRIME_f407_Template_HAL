#include "TaskList.h"
#include "RGB_LED.h"


using namespace Device;


void TestTask(void const * argument){

    uint32_t col = 0;

    RGB_LED &hLED = RGB_LED::getInstance();
    hLED.setColor(0xFFFF00FF);


    while (1){


        if(col)
            hLED.setColor(0xFFFF00FF);
        else
            hLED.setColor(0xFF000000);
        col = !col;

        osDelay(500);

    }

}
