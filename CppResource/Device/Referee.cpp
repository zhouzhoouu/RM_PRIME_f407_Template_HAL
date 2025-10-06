#include "Referee.h"
using namespace Device;

Referee &Referee::getInstance(){
    static Referee instance;
    return instance;
}

void Referee::init() {
    DataFifo.clear();
    RefereeTuple = {};
}

uint32_t Referee::pushData(uint8_t* data, uint32_t len){

    if (DataFifo.available()<len) {
        len = DataFifo.available();
    }

    for (uint32_t i = 0; i < len; i++){
        DataFifo.push(data[i]);
    }

    return len;
}

void Referee::packetWrite(uint16_t ID, uint8_t* data, uint16_t len){
    packetWriteTemplate(ID, data, len, RefereeTuple);
}

void Referee::ProcessData(){
    if (DataFifo.available()>0) {DataFifo.push(0);}
}
