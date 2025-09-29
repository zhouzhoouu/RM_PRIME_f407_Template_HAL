#include "DBus.h"



extern "C" void USART3_IDEL_IRQHandler(UART_HandleTypeDef *huart){

    HAL_UART_DMAStop(huart);
//    uint32_t len = huart->RxXferSize - huart->hdmarx->Instance->NDTR;
//    DBus* dbus = reinterpret_cast<DBus*>(huart->pUserData);
//    dbus->receiveMessage(dbus->buffer, len);

//    HAL_UART_Receive_DMA(huart, dbus->buffer, dbus->frameLength);

}