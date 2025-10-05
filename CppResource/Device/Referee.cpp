#include "Referee.h"
using namespace Device;

Referee &Referee::getInstance(){
    static Referee instance;
    return instance;
}

void Referee::init() {
    DataFifo.clear();
}

uint32_t Referee::PushData(uint8_t* data, uint32_t len){

    if (DataFifo.available()<len) {
        len = DataFifo.available();
    }

    for (uint32_t i = 0; i < len; i++){
        DataFifo.push(data[i]);
    }

    return len;
}
