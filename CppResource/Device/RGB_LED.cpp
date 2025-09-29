#include "RGB_LED.h"

namespace Device{
    RGB_LED::RGB_LED()
    {
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

        setColor(0x00000000);
    }

    void RGB_LED::setColor(uint32_t aRGB){
        uint8_t alpha;
        uint16_t red,green,blue;

        alpha = (aRGB & 0xFF000000) >> 24;
        red = ((aRGB & 0x00FF0000) >> 16) * alpha;
        green = ((aRGB & 0x0000FF00) >> 8) * alpha;
        blue = ((aRGB & 0x000000FF) >> 0) * alpha;

        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);

        color = aRGB;
    }

    uint32_t RGB_LED::getColor() {
        return color;
    }

    RGB_LED& RGB_LED::getInstance() {
        static RGB_LED instance;
        return instance;
    }
}