#include "DM4310.h"

namespace Device{

    DM4310::DM4310(CAN_HandleTypeDef *hc, uint32_t motorid):
            InterfaceCAN(hc, 0x010+motorid, motorid)
    {

    }


    void DM4310::init(){
        uint8_t enable_motor_cmd[] =
                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        uint8_t len = 0x08;
        sendMessage(enable_motor_cmd, len);
    }

    bool DM4310::receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len){
        return true;
    }



}