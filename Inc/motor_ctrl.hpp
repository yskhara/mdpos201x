/*
 * motor_ctrl.hpp
 *
 *  Created on: Dec 23, 2018
 *      Author: yusaku
 */

#pragma once
#include <cmath>
#include "stm32f1xx_hal.h"
#include "main.h"
#include "SerialClass.hpp"

#include "conf.h"
#include "led.hpp"

extern SerialClass serial;

class MotorCtrl
{
    using Float_Type = float;

public:
    void SetTarget(Float_Type target);
    void ResetPosition(Float_Type offset = 0.0);

    void Control(void);

    void Print(void);

    inline void Shutdown(void)
    {
        //GPIOB->BSRR = GPIO_BSRR_BR15;
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;

        this->shutdown = true;
        this->homing = false;

        led::mode = led::lighting_mode::shutdown;

        this->ResetState();
    }

    inline void Recover(void)
    {
        if(this->homing)
        {
            return;
        }

        _recover();
    }

    inline void _recover(void)
    {

        if ((GPIO_EMS->IDR & GPIO_IDR_EMS) != 0)
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            this->ResetState();

            //GPIOB->BSRR = GPIO_BSRR_BS15;
            TIM1->BDTR |= TIM_BDTR_MOE;

            this->shutdown = false;

            led::mode = led::lighting_mode::operational;
        }
    }

    void ReadConfig(void);
    void WriteConfig(void);

    void LimitSwitch0Handler(void);
    void LimitSwitch1Handler(void);

    void Home(void);
    inline bool IsHoming(void)
    {
        return this->homing;
    }

private:

    uint16_t ccr_arr = 1440 - 1;
    uint16_t ccr_max = 1400 - 1;

    // @param d duty ratio multiplied by 1000, where 1000 represents 100% duty ratio.
    inline void SetDuty(int d)
    {
        if (d < -1000 || 1000 < d)
        {
            return;
        }

        if (0 < d)
        {
            TIM1->CCR1 = d * ccr_max / 1000;
            TIM1->CCR2 = 0;
        }
        else if (d < 0)
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = -d * ccr_max / 1000;
        }
        else
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
        }
    }

    inline void ResetState(void)
    {
        //TIM2->CNT = 0;
        //this->enc_cnt = 0;
        //this->pulse = 0;
        //this->velocity = 0;
        this->error = 0;
        this->error_prev = 0;
        this->u_p = 0;
        this->u_i = 0;

#ifdef CTRL_POS
        this->target_position_pulse = this->current_position_pulse;
#endif
        this->target_velocity = 0;
        this->target_torque = 0;
        this->target_voltage = 0;
    }

    bool shutdown = true;
    bool homing = false;

    int pulse = 0;
    Float_Type velocity = 0;                            // current velocity in [rad/s]

    Float_Type error = 0;                               // error in [rad/s]
    Float_Type error_prev = 0;                          // previous error in [rad/s]

    Float_Type u_p = 0;
    Float_Type u_i = 0;

#ifdef CTRL_POS
    int current_position_pulse = 0;
    int target_position_pulse = 0;
#endif

    Float_Type target_velocity = 0;                     // target angular velocity [rad/sec]
    Float_Type target_torque = 0;                       // target torque [N.m]
    Float_Type target_voltage = 0;                      // target voltage [V]

    Float_Type Kr = 1.0;                                // 入力である速度指令を，[rad]に変換する係数．
                                                    // degで指令するなら， M_PI/180．rad/secで指令するなら 1．

    //Float_Type Ki = 0;
    Float_Type Kp = 0;                                  // 比例ゲイン
    Float_Type KiTc = 0;                                // 積分ゲインと制御周期の積
    Float_Type Tc = 0.001;                              // 制御周期 [sec]

    Float_Type Kv = 20;                                  // 位置偏差比例ゲイン
                                                      // 775, 385では40にした．
                                                        // 380では20にしてみたけど，もう少し低くても良さそう．

    //Float_Type Ppr = 2000;
    Float_Type Kh = 2 * M_PI / (2000 * Tc);             // エンコーダ入力[pulse/ctrl]を[rad/s]に変換する係数．kg / Tc．

    //double Ra = 0.1384615;                          // 巻線抵抗 [Ohm]
    //double Kt = 0.0083762 * 24;                     // トルク定数 [N.m/A]

    Float_Type Ke = 0.0080087 * 17;                     // 速度起電力定数 [V/(rad/sec)]
    //double Ke = 0.0080087 * 40;                     // 速度起電力定数 [V/(rad/sec)]
    Float_Type Kg = 0.1384615 / (0.0083762 * 24);       // トルクから電圧への係数．定義は 巻線抵抗 / トルク定数．
    //double Kg = 0.1384615 / (0.0083762 * 61);       // トルクから電圧への係数．定義は 巻線抵抗 / トルク定数．

    int MaximumPosition_pulse = 20 * M_PI / (Kh * Tc);   // 回転角制限 [pulse]
    Float_Type MaximumVelocity = 100;                   // 回転数制限 [rad/s]
    Float_Type MaximumTorque = 10 * (0.0083762 * 24);    // 吐ける電流量を基準に，トルク制限を定める．
                                                     //電気ではなく機械を基準にしてもよかろう．
    Float_Type MaximumVoltage = 22;                     // デューティ最大のときの出力電圧で，電圧制限を定める．

    Float_Type SupplyVoltage = 24;

    Float_Type HomingVelocity = 1;                      // ホーミングの際の速度．
                                                     // TODO: UARTから変更できるようにする．→できるようになった？

public:

    // set proportional gain Kp
    inline int SetKp(Float_Type kp)
    {
        if (kp < 0)
            return -1;

        this->Kp = kp;
        return 0;
    }

    inline Float_Type GetKp(void)
    {
        return this->Kp;
    }

    // integral gain
    inline int SetKi(Float_Type ki)
    {
        if (ki < 0)
            return -1;

        this->KiTc = ki * Tc;
        return 0;
    }

    inline Float_Type GetKi(void)
    {
        return this->KiTc / Tc;
    }

    // emf constant
    inline int SetKe(Float_Type ke)
    {
        if (ke < 0)
            return -1;

        this->Ke = ke;
        return 0;
    }

    inline Float_Type GetKe(void)
    {
        return this->Ke;
    }

    // winding resistance Ra divided by torque constant Kt, i.e., Ra/Kt.
    inline int SetKg(Float_Type kg)
    {
        if (kg < 0)
            return -1;

        this->Kg = kg;
        return 0;
    }

    inline Float_Type GetKg(void)
    {
        return this->Kg;
    }

    // set ppr: [pulse] -> [rad]
    inline int SetPPR(Float_Type ppr)
    {
        //if (ppr < 0)
        //    return -1;

        this->Kh = 2 * M_PI / (ppr * Tc);

        // TODO: make MaximumPosition_pulse configurable
        this->MaximumPosition_pulse = M_PI / (Kh * Tc);
        return 0;
    }

    inline Float_Type GetPPR(void)
    {
        return 2 * M_PI / (this->Kh * Tc);
    }

    // coefficient: (unit of reference) -> [rad/s]
    inline int SetKr(Float_Type kr)
    {
        // Kr is allowed to be negative value.
        //if (kr < 0)
        //    return -1;

        this->Kr = kr;
        return 0;
    }

    inline Float_Type GetKr(void)
    {
        return this->Kr;
    }

    // coefficient: (position error [rad]) -> (target velocity [rad/s])
    inline int SetKv(Float_Type kv)
    {
        // Kv is NOT allowed to be negative value.
        if (kv < 0)
            return -1;

        this->Kv = kv;
        return 0;
    }

    inline Float_Type GetKv(void)
    {
        return this->Kv;
    }

    inline int SetMaximumVelocity(Float_Type om)
    {
        if (om < 0)
            return -1;

        this->MaximumVelocity = om;
        return 0;
    }

    inline Float_Type GetMaximumVelocity(void)
    {
        return this->MaximumVelocity;
    }

    inline int SetHomingVelocity(Float_Type val)
    {
        //if (val < 0)
        //    return -1;

        this->HomingVelocity = val;
        return 0;
    }

    inline Float_Type GetHomingVelocity(void)
    {
        return this->HomingVelocity;
    }

    inline int SetMaximumTorque(Float_Type tm)
    {
        if (tm < 0)
            return -1;

        this->MaximumTorque = tm;
        return 0;
    }

    inline Float_Type GetMaximumTorque(void)
    {
        return this->MaximumTorque;
    }

    inline int SetSupplyVoltage(Float_Type vs)
    {
        if (vs < 0)
            return -1;

        this->SupplyVoltage = vs;
        this->MaximumVoltage = vs * (ccr_max + 1) / (ccr_arr + 1);
        return 0;
    }

    inline Float_Type GetSupplyVoltage(void)
    {
        return this->SupplyVoltage;
    }

    inline Float_Type GetCurrentVelocity(void)
    {
        return this->velocity;
    }

    inline uint8_t GetStatusCode(void)
    {
        if(this->homing)
        {
            return 0x10;
        }

        if(this->shutdown)
        {
            return 0x00;
        }

        return 0x01;
    }
};

extern MotorCtrl control;

