#include "TaskList.h"
#include "Referee.h"
#include "VT03.h"
#include "ui.h"

using namespace Device;


void ClientUI_Task(void const * argument){

    Referee &hreferee = Referee::getInstance();
    auto &hVT03 = VT03::getInstance();

    static bool last_G = false;

    while (1){

        bool G = BITMASK(hVT03.getState()->key_code, 10);
        if(G && !last_G)
        {
            ui_init_g_Ungroup();
        }
        last_G = G;


        ui_update_g_Ungroup();

        osDelay(50);
    }

}