#ifndef F407_RM_TMPLATE_HAL_REFEREE_H
#define F407_RM_TMPLATE_HAL_REFEREE_H

#include "main_cpp.h"
#include "etl/queue.h"
#include "RefereeType.h"

namespace Device{

    class Referee {
    public:

        static Referee& getInstance();

        void init();
        void Send(uint8_t* data, uint8_t len);

        /// 将数据存入裁判系统串口解析FIFO
        /// @param data ///数据指针
        /// @param len 数据长度
        /// @return 返回实际存入的数据长度
        uint32_t pushData(uint8_t* data, uint32_t len);

        template<typename T>
        std::enable_if_t<std::is_base_of_v<RefereeType::RefereeCMD<T::CMD_ID>, T>,T&>
        getRefereeInfo() {
            return std::get<T>(RefereeTuple);
        }

        void ProcessData();

    private:

        etl::queue<uint8_t, 256> DataFifo;
        RefereeType::RefereeTupleType RefereeTuple;

        Referee() = default;
        Referee(const Referee&) = delete;
        Referee& operator=(const Referee&) = delete;

        void packetWrite(uint16_t ID, uint8_t* data, uint16_t len);
    };


}

#endif
