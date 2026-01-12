#ifndef F407_RM_TMPLATE_HAL_IST8310_H
#define F407_RM_TMPLATE_HAL_IST8310_H

#include "main_cpp.h"

namespace Device{

    class IST8310 {
    public:
        static IST8310& getInstance();
        void init();

    private:
        IST8310() = default;
        ~IST8310() = default;
        IST8310(const IST8310&) = delete;
        IST8310& operator=(const IST8310&) = delete;

        static constexpr uint8_t IST8310_IIC_ADDRESS = (0x0E << 1);  //IST8310的IIC地址
        static constexpr uint8_t IST8310_IIC_READ_MSB = (0x80); //IST8310的SPI读取发送第一个bit为1

        static constexpr float MAG_SEN = 0.3f; //转换成 uT

        static constexpr uint8_t IST8310_WHO_AM_I = 0x00;       //ist8310 who am I 寄存器
        static constexpr uint8_t IST8310_WHO_AM_I_VALUE = 0x10; //设备 ID
        static constexpr uint8_t IST8310_WRITE_REG_NUM = 4; //IST8310需要设置的寄存器数目

        static constexpr uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] =
                {
                        {0x0B, 0x08, 0x01},
                        {0x41, 0x09, 0x02},
                        {0x42, 0xC0, 0x03},
                        {0x0A, 0x0B, 0x04}
                };

        uint8_t read_single_reg(uint8_t reg);

    };

}

#endif
