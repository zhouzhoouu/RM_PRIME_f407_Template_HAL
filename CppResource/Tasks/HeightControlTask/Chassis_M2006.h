#ifndef F407_RM_TMPLATE_HAL_CHASSIS_M2006_H
#define F407_RM_TMPLATE_HAL_CHASSIS_M2006_H

namespace Chassis_M2006 {

    union RodSet{
        struct {
            int FL;
            int FR;
            int BL;
            int BR;
        };
        int datas[4];
    };

    void setRodLen(RodSet rods);
    void setSpeed(RodSet rods);
    bool RodLenSetZero();
    RodSet getRodLen();

}

#endif
