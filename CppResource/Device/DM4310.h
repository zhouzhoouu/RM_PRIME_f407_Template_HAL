#ifndef F407_RM_TMPLATE_HAL_DM4310_H
#define F407_RM_TMPLATE_HAL_DM4310_H

#include "InterfaceCAN.h"

namespace Device{

class DM4310 : public BSP::InterfaceCAN{
    public:
        DM4310(CAN_HandleTypeDef *hc, uint32_t motorid);
        void init() override;
        bool receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len) override;


    };

}



#endif
