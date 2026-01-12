#include "IST8310.h"

using namespace Device;

IST8310& IST8310::getInstance(){
    static IST8310 instance;
    return instance;
}

uint8_t IST8310::read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}

void IST8310::init() {

    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    read_single_reg(IST8310_WHO_AM_I);

}

