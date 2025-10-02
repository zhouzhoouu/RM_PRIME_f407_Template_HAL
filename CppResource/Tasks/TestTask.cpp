#include "TaskList.h"
#include "RGB_LED.h"
#include "Buzzer.h"
#include "usbd_cdc_if.h"

using namespace Device;


void TestTask(void const * argument){


    Buzzer &hBuzzer = Buzzer::getInstance();
    RGB_LED &hLED = RGB_LED::getInstance();


    uint32_t col = 0;
    hLED.setColorPhase(col);


    hBuzzer.setFrequency(Buzzer::ToneList[0]);

    //hBuzzer.toggle();


    while (1){


        //hBuzzer.toggle();
        //uint8_t msg[] = "Hello World!\r\n";
        //CDC_Transmit_FS(msg, sizeof(msg)-1);

        col = (col + 1) % 360;
        hLED.setColorPhase(col);

        // col = (col + 1) % 20;
        // hBuzzer.setFrequency(Buzzer::ToneList[2+col]);

        osDelay(50);

    }

}
