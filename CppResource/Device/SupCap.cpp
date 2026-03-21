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
        return rxData.P_measured;
    }

    uint32_t SupCap::getRest() const{
        return rxData.Rest_energy;
    }


    float SupCap::getMaxOutPower() const{
        return 0.1f*(float)rxData.Max_Power;
    }

    uint8_t SupCap::getErrcode() const{
        return rxData.Err_code;
    }

    bool SupCap::EnableDischarge() const{
        return supcap_enable;
    }

    void SupCap::SetParameter(float energy, float power_limit,bool power_on){
        TxData txData{};
        txData.E_buffer = energy;
        txData.Ref_Power_Limit = (uint8_t)power_limit;
        txData.disChargeEnable = supcap_enable;
        txData.chassis_Power_on = power_on?1:0;
        uint8_t len = 0x08;
        sendMessage(txData.datas, len);
    }

    void SupCap::SetState(bool enable){
        supcap_enable = enable;

    }

}