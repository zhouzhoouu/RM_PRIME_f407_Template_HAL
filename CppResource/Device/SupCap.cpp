#include "SupCap.h"

namespace Device{

    SupCap::SupCap(CAN_HandleTypeDef *hc):
    InterfaceCAN(hc,0x641,0x640){

    }

    void SupCap::init() {}

    bool SupCap::receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len){
        for (int i = 0; i < 8; ++i) {
            rxData.datas[i] = pdata[i];
        }

        return true;
    }

    float SupCap::getPower() const{
        return rxData.power;
    }

    void SupCap::SetRestEng(float energy){
        TxData txData{};
        txData.restEng = energy;
        uint8_t len = 0x08;
        sendMessage(txData.datas, len);
    }

}