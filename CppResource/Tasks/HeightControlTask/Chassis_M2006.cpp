#include "Chassis_M2006.h"
#include "MotionControl.h"

namespace Chassis_M2006 {

    constexpr float MAX_TOR_CUR = 8000.0f;
    constexpr short ZEROING_TOR = 3500;
    static constexpr short PI_DIG = (4096);

    short angleMod(short angle){
        using namespace MotionParameter;
        while (angle >= PI_DIG) angle -= 2 * PI_DIG;
        while (angle <= -PI_DIG) angle += 2 * PI_DIG;
        return angle;
    }

//    DJiMotorGroup GroupM2006(&hcan2, 0x205, 0x1FF);
    DJiMotorGroup GroupM2006(&hcan2, 0x201, 0x200);

    DeltaPID Pos_PID[4] = {
            DeltaPID(2.f, 0.02f, .1f, .0f, MAX_TOR_CUR, -MAX_TOR_CUR),
            DeltaPID(2.f, 0.02f, .1f, .0f, MAX_TOR_CUR, -MAX_TOR_CUR),
            DeltaPID(2.f, 0.02f, .1f, .0f, MAX_TOR_CUR, -MAX_TOR_CUR),
            DeltaPID(2.f, 0.02f, .1f, .0f, MAX_TOR_CUR, -MAX_TOR_CUR)
    };

    RodSet RodLen{{0,0,0,0}};
    int angle[4] = {0,0,0,0};
    int last_angle[4] = {0,0,0,0};


    void setSpeed(RodSet rods){

        short tmp_cur[]{0,0,0,0};
        for (int i = 0; i < 4; ++i) {
            auto  sta = GroupM2006.getMotorState(i);
            short omega = sta.speed;
            short ang = sta.angle;

            short d_angle = angleMod((short)(ang - last_angle[i]));
            angle[i] += d_angle;
            last_angle[i] = ang;

            Pos_PID[i].Run(rods.datas[i],omega);
            tmp_cur[i] = (short)Pos_PID[i].Output;
            //tmp_cur[i] = rods.datas[i];

        }


        GroupM2006.setMotorCurrent(tmp_cur);
    }


    bool RodLenSetZero(){

        short tmp_cur[]{0,0,0,0};

        bool rel = false;

        for (int i = 0; i < 4; ++i) {
            auto  sta = GroupM2006.getMotorState(i);
            short omega = sta.speed;

            Pos_PID[i].Run(300,omega);
            tmp_cur[i] = (short)Pos_PID[i].Output;
            if(tmp_cur[i] > ZEROING_TOR) tmp_cur[i] = ZEROING_TOR;
            else if(tmp_cur[i] < -ZEROING_TOR) tmp_cur[i] = -ZEROING_TOR;
            rel |= (omega > 10);
        }
        GroupM2006.setMotorCurrent(tmp_cur);
        return !rel;
    }

    void setRodLen(RodSet rods){
        for(int i = 0; i < 4; ++i){
            angle[i] = rods.datas[i];
        }
    }


    RodSet getRodLen(){
        for(int i = 0; i < 4; ++i){
            RodLen.datas[i] = angle[i];
        }
        return RodLen;
    }

}

