#include "ShootTask.h"
#include "Referee.h"

namespace TriggerControl{

    static constexpr short PI_DIG = (4096);
    static constexpr int dir = 1;
    static constexpr uint32_t trig_id = 0;

    static bool isPowerOn;
    int TriggerCounter;
    short last_tri_ang;
    uint32_t timeout;
    bool backing;

    DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);
    DeltaPID Triger_PID = DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Tri_CUR, -MAX_Tri_CUR);
    auto &hreferee = Referee::getInstance();

    static short angleMod(short angle){
        while (angle >= PI_DIG) angle -= 2 * PI_DIG;
        while (angle <= -PI_DIG) angle += 2 * PI_DIG;
        return angle;
    }

    void NotifyPowerSate(bool s){
        if(isPowerOn && !s){
            TriggerCounter = 0;
            last_tri_ang = 0;
            timeout = 0;
            backing = false;
            Triger_PID.Reset();
        }
        isPowerOn = s;
    }

    void setSpeed(float target_speed){

        if(!isPowerOn)return;

        //
        short tmp_cur[] = {0, 0, 0, 0};
        auto sta = m3508Group_triger.getMotorState(trig_id);
        float motor_speed = sta.speed;

        //多圈计数
        short ang = sta.angle;
        short d_angle = angleMod((short)(ang - last_tri_ang));
        TriggerCounter += d_angle;
        last_tri_ang = (short)ang;

        //卡弹检测
        if(backing)
            target_speed = -Tri_BACKING_SPEED;
        float error = target_speed - dir*motor_speed;
        if(error > TRIG_STUCK_RATE * MAX_TRIG_SPEED){
            timeout++;
            if(timeout > Tri_BACKING_TIME){
                backing = true;
            }
        }
        else{
            if(timeout > 0)
                timeout--;
            else if(timeout == 0)
                backing = false;

        }

        Triger_PID.Run(dir*target_speed, motor_speed);
        tmp_cur[trig_id] = (short)Triger_PID.Output;
        m3508Group_triger.setMotorCurrent(tmp_cur);
    }

    void Loop(){
        //热量限制
        using namespace RefereeType;
        uint16_t shooterHeat = hreferee.getRefereeInfo<RefereeType::PowerHeatData>().shooter_id1_17mm_cooling_heat;
        uint16_t shooterHeatMax = hreferee.getRefereeInfo<GameRobotState>().shooter_barrel_heat_limit;

        if(shooterHeatMax - shooterHeat < 30){
            RestStep();
        }

        float tspd = -0.1f*(float)TriggerCounter;
        if (tspd > MAX_TRIG_SPEED) tspd = MAX_TRIG_SPEED;
        else if (tspd < -MAX_TRIG_SPEED) tspd = -MAX_TRIG_SPEED;

        setSpeed(tspd);
    }

    void AddStep(){
        if(TriggerCounter > -2*SHOOT_STEP)
            TriggerCounter -= SHOOT_STEP;
    }

    void RestStep(){
        while (TriggerCounter < -SHOOT_STEP) TriggerCounter += SHOOT_STEP;
//        TriggerCounter = 0;
    }

}


//[[noreturn]] void DebugTask(void const * argument){
//
//    while (true){
//
//        float pack[8];
//        pack[0] = TriggerControl::TriggerCounter;
//        Debug::print_vofa(pack, 8);
//
//        osDelay(50);
//    }
//}