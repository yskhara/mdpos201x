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

//#define IGNORE_EMS

extern SerialClass serial;

class MotorCtrl
{
public:
    void SetTarget(double target);
    void ResetPosition(double offset = 0.0);

    void Control(void);

    void Print(void);

    inline void Shutdown(void)
    {
        GPIOB->BSRR = GPIO_BSRR_BR15;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;

        this->shutdown = true;

        this->ResetState();
    }

    inline void Recover(void)
    {
#ifdef IGNORE_EMS
#warning "ignore me if you know what you are doing."
        if (true)
#else
        if ((GPIOC->IDR & GPIO_IDR_IDR14) != 0)
#endif
        {
            TIM1->CCR1 = 0;
            TIM1->CCR2 = 0;
            this->ResetState();

            GPIOB->BSRR = GPIO_BSRR_BS15;

            this->shutdown = false;
        }
    }

    void ReadConfig(void);
    void WriteConfig(void);

private:

    uint16_t ccr_arr = 720 - 1;
    uint16_t ccr_max = 685 - 1;

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
        this->pulse = 0;
        this->velocity = 0;
        this->error = 0;
        this->error_prev = 0;
        this->u_p = 0;
        this->u_i = 0;

        this->target_position_pulse = this->current_position_pulse;
        this->target_velocity = 0;
        this->target_torque = 0;
        this->target_voltage = 0;
    }

    bool shutdown = true;

    int pulse = 0;
    double velocity = 0;                            // current velocity in [rad/s]

    double error = 0;                               // error in [rad/s]
    double error_prev = 0;                          // previous error in [rad/s]

    double u_p = 0;
    double u_i = 0;

    int current_position_pulse = 0;
    int target_position_pulse = 0;

    double target_velocity = 0;                     // target angular velocity [rad/sec]
    double target_torque = 0;                       // target torque [N.m]
    double target_voltage = 0;                      // target voltage [V]

    double Kr = 1.0;                                // 入力である速度指令を，[rad]に変換する係数．
                                                    // degで指令するなら， M_PI/180．rad/secで指令するなら 1．

    double Kp = 0;                                  // 比例ゲイン
    double KiTc = 0;                                // 積分ゲインと制御周期の積
    double Tc = 0.001;                              // 制御周期 [sec]

    double Kv = 40;                                  // 位置偏差比例ゲイン
                                                      // 775, 385では40にした．
                                                        // 380では20にしてみたけど，もう少し低くても良さそう．

    double Kh = 2 * M_PI / (2000 * Tc);             // エンコーダ入力[pulse/ctrl]を[rad/s]に変換する係数．kg / Tc．

    //double Ra = 0.1384615;                          // 巻線抵抗 [Ohm]
    //double Kt = 0.0083762 * 24;                     // トルク定数 [N.m/A]

    double Ke = 0.0080087 * 17;                     // 速度起電力定数 [V/(rad/sec)]
    //double Ke = 0.0080087 * 40;                     // 速度起電力定数 [V/(rad/sec)]
    double Kg = 0.1384615 / (0.0083762 * 24);       // トルクから電圧への係数．定義は 巻線抵抗 / トルク定数．
    //double Kg = 0.1384615 / (0.0083762 * 61);       // トルクから電圧への係数．定義は 巻線抵抗 / トルク定数．

    int MaximumPosition_pulse = 20 * M_PI / (Kh * Tc);   // 回転角制限 [pulse]
    double MaximumVelocity = 100;                   // 回転数制限 [rad/s]
    double MaximumTorque = 10 * (0.0083762 * 24);    // 吐ける電流量を基準に，トルク制限を定める．
                                                     //電気ではなく機械を基準にしてもよかろう．
    double MaximumVoltage = 22;                     // デューティ100%のときの出力電圧で，電圧制限を定める．

    double SupplyVoltage = 24;

public:

    // set proportional gain Kp
    inline int SetKp(double kp)
    {
        if (kp < 0)
            return -1;

        this->Kp = kp;
        return 0;
    }

    inline double GetKp(void)
    {
        return this->Kp;
    }

    // integral gain
    inline int SetKi(double ki)
    {
        if (ki < 0)
            return -1;

        this->KiTc = ki * Tc;
        return 0;
    }

    inline double GetKi(void)
    {
        return this->KiTc / Tc;
    }

    // emf constant
    inline int SetKe(double ke)
    {
        if (ke < 0)
            return -1;

        this->Ke = ke;
        return 0;
    }

    inline double GetKe(void)
    {
        return this->Ke;
    }

    // winding resistance Ra divided by torque constant Kt, i.e., Ra/Kt.
    inline int SetKg(double kg)
    {
        if (kg < 0)
            return -1;

        this->Kg = kg;
        return 0;
    }

    inline double GetKg(void)
    {
        return this->Kg;
    }

    // set ppr: [pulse] -> [rad]
    inline int SetPPR(double ppr)
    {
        //if (ppr < 0)
        //    return -1;

        this->Kh = 2 * M_PI / (ppr * Tc);

        // TODO: make MaximumPosition_pulse configurable
        this->MaximumPosition_pulse = M_PI / (Kh * Tc);
        return 0;
    }

    inline double GetPPR(void)
    {
        return 2 * M_PI / (this->Kh * Tc);
    }

    // coefficient: (unit of reference) -> [rad/s]
    inline int SetKr(double kr)
    {
        if (kr < 0)
            return -1;

        this->Kr = kr;
        return 0;
    }

    inline double GetKr(void)
    {
        return this->Kr;
    }

    inline int SetMaximumVelocity(double om)
    {
        if (om < 0)
            return -1;

        this->MaximumVelocity = om;
        return 0;
    }

    inline double GetMaximumVelocity(void)
    {
        return this->MaximumVelocity;
    }

    inline int SetMaximumTorque(double tm)
    {
        if (tm < 0)
            return -1;

        this->MaximumTorque = tm;
        return 0;
    }

    inline double GetMaximumTorque(void)
    {
        return this->MaximumTorque;
    }

    inline int SetSupplyVoltage(double vs)
    {
        if (vs < 0)
            return -1;

        this->SupplyVoltage = vs;
        this->MaximumVoltage = vs * (ccr_max + 1) / (ccr_arr + 1);
        return 0;
    }

    inline double GetSupplyVoltage(void)
    {
        return this->SupplyVoltage;
    }

    inline double GetCurrentVelocity(void)
    {
        return this->velocity;
    }
};

extern MotorCtrl control;

