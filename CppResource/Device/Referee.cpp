#include "Referee.h"
using namespace Device;

Referee &Referee::getInstance(){
    static Referee instance;
    return instance;
}

void Referee::init() {
    DataFifo.clear();
}
