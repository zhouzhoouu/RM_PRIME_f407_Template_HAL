#ifndef F407_RM_TMPLATE_HAL_SUPCAP_H
#define F407_RM_TMPLATE_HAL_SUPCAP_H

#include "InterfaceCAN.h"

namespace Device{

class SupCap: public BSP::InterfaceCAN{
    public:
        SupCap(CAN_HandleTypeDef *hc);

        void init() override;
        bool receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len) override;

        float getPower() const;
        void SetRestEng(float energy);

        ~SupCap() override = default;

    private:
        union RxData{
            struct {
                float power;
                uint32_t reserved;
            };
            uint8_t datas[8];
        };

        union TxData{
            struct {
                float restEng;
                uint32_t reserved;
            };
            uint8_t datas[8];
        };

        RxData rxData;


    };

}

#endif //F407_RM_TMPLATE_HAL_SUPCAP_H
