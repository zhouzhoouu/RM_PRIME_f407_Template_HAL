#ifndef F407_RM_TMPLATE_HAL_REFEREE_H
#define F407_RM_TMPLATE_HAL_REFEREE_H

#include "main.h"
#include "etl/queue.h"

namespace Device{

    class Referee {
    public:



        static Referee& getInstance();

        void init();
        void Send(uint8_t* data, uint8_t len);
        bool PushData(uint8_t* data, uint32_t len);
        void ProcessData();

    private:
        Referee() = default;
        Referee(const Referee&) = delete;
        Referee& operator=(const Referee&) = delete;

        etl::queue<uint8_t, 256> DataFifo;

    };


}

#endif
