#include "TaskList.h"
#include "Chassis_M2006.h"
#include "DBus.h"
#include "VT03.h"

static constexpr int MAX_SPEED = 6000;

bool isHigh = false;
bool triggered = false;

bool last_fn2 = false;

constexpr Chassis_M2006::RodSet HighPos = {{-3250000,-3250000,-3250000,-3250000}};

[[noreturn]] void HeightControlTask(void const * argument){

    auto &Dbus = Device::DBus::getInstance();
    auto &hVT03 = Device::VT03::getInstance();

    using namespace Chassis_M2006;

    RodSet target_rod_pos = {{0,0,0,0}};

    uint32_t zero_cnt = 0;
    osDelay(5000);
    while (zero_cnt < 200){
        zero_cnt += RodLenSetZero();
        osDelay(1);
    }
    setRodLen({{800000,800000,800000,800000}});

    while (true){

        short whel = -Dbus.getState()->ch[4] + hVT03.getState()->wheel;

        if(triggered){
            if(whel > -300){
                triggered = false;
            }
        } else{
            if(whel < -600){
                isHigh = !isHigh;
                triggered = true;
                if(isHigh) target_rod_pos = HighPos;
                else target_rod_pos = {{0,0,0,0}};
            }
        }

//        bool fn2_s = hVT03.getState()->fn_2;
//        if(last_fn2 != fn2_s && fn2_s){
//            isHigh = !isHigh;
//            if(isHigh) target_rod_pos = HighPos;
//            else target_rod_pos = {{0,0,0,0}};
//        }
//
//        last_fn2 = fn2_s;

        RodSet rod_spd = {{0,0,0,0}};
        auto rod_pos = getRodLen();
        for (int i = 0; i < 4; ++i) {
            int dx = target_rod_pos.datas[i] - rod_pos.datas[i];
            int spd = dx >> 5;
            if (spd > MAX_SPEED) spd = MAX_SPEED;
            else if (spd < -MAX_SPEED) spd = -MAX_SPEED;
            rod_spd.datas[i] = (int) spd;
        }

        setSpeed(rod_spd);

        osDelay(1);
    }
}


//[[noreturn]] void DebugTask(void const * argument){
//
//    using namespace Chassis_M2006;
//
//    while (true){
//
//
//        float pack[6];
//
//        for (int i = 0; i < 4; ++i) {
//            pack[i] = getRodLen().datas[i];
//        }
//        pack[4] = isHigh;
//        pack[5] = triggered;
//
////        Debug::print_vofa(pack, 6);
//
//        osDelay(50);
//    }
//}
