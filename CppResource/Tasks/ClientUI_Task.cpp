#include "TaskList.h"
#include "Referee.h"
#include "ui.h"

using namespace Device;


void ClientUI_Task(void const * argument){

    Referee &hreferee = Referee::getInstance();
    bool last_referee_exist = hreferee.RefereeExist();

    while (1){

        bool ref_exist = hreferee.RefereeExist();
        if(last_referee_exist != ref_exist) {
            last_referee_exist = ref_exist;
            if(ref_exist){
                osDelay(100);
                ui_remove_g_Ungroup();
                osDelay(100);
                ui_init_g_Ungroup();
            }
        }

//        ui_remove_g_Ungroup();
        ui_init_g_Ungroup();
//        if(ui_g_Ungroup_engbar->end_x > ui_g_Ungroup_engbar->start_x)
//            ui_g_Ungroup_engbar->end_x--;
//        ui_g_Ungroup_engbar->start_x = 500;
//        ui_update_g_Ungroup();

        osDelay(50);
    }

}