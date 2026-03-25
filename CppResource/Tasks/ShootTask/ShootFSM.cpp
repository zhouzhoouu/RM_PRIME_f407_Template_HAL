#include "ShootTask.h"

namespace ShootFSM{

    bool isFribOpened = false;
    bool isZeroCross = false;
    uint32_t FirbCounter[2] = {0,0};


    bool getIsFribOpened(){
        return isFribOpened;
    }

    bool getIsZeroCross(){
        return isZeroCross;
    }


    void ShootFSMLoop(short tin){

        if(isFribOpened){
            if(tin < -300){
                FirbCounter[1]++;
                if(FirbCounter[1] >= TOPEN){
                    isFribOpened = false;

                    FirbCounter[0] = 0;
                    FirbCounter[1] = 0;
                }
            } else{
                FirbCounter[1] = 0;
            }

        } else{
            if(tin > 580){
                FirbCounter[0]++;
                if(FirbCounter[0] >= TOPEN){
                    isFribOpened = true;
                    isZeroCross = false;

                    FirbCounter[0] = 0;
                    FirbCounter[1] = 0;
                }
            } else{
                FirbCounter[0] = 0;
            }
        }

        if(isZeroCross == false){
            if(tin > -50 && tin < 50){
                isZeroCross = true;
            }
        }

    }




}
