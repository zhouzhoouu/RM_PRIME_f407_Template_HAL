#ifndef F407_RM_TMPLATE_HAL_DBUS_H
#define F407_RM_TMPLATE_HAL_DBUS_H

#include "main_cpp.h"

namespace Device {
    class DBus {
    public:
        struct RCState {
            int16_t ch[5];       // -660~660
            uint8_t s[2];       // 1 or 3
            int16_t mouse_x;    // -32768~32768
            int16_t mouse_y;    // -32768~32768
            int16_t mouse_z;    // -32768~32768
            uint8_t press_l;    // 0 or 1
            uint8_t press_r;    // 0 or 1
            uint16_t key_code;  // bit field
        };

        DBus();
        void init();
        bool receiveMessage(uint8_t *pdata, uint32_t len);
        const RCState* getState() const;

    private:
        UART_HandleTypeDef *hUART;
        RCState state;
        uint8_t buffer[18];
        uint8_t bufferIndex;
        uint8_t frameLength;
        uint16_t crc;
        uint16_t calcCRC(const uint8_t* data, uint32_t len);
    };
}

#endif
