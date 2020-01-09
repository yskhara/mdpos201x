/*
 * motor_ctrl.cpp
 *
 *  Created on: Dec 23, 2018
 *      Author: yusaku
 */

#include "motor_ctrl.hpp"

void MotorCtrl::Control(void)
{
    int pulse = -static_cast<int16_t>(TIM2->CNT);
    TIM2->CNT = 0;

#ifdef CTRL_POS
    // update current position
    this->current_position_pulse += pulse;
#endif
    this->velocity = pulse * Kh;

    if ((GPIO_EMS->IDR & GPIO_IDR_EMS) == 0)
    {
        this->Shutdown();
    }

    if (this->shutdown)
    {
        // disable master output
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;

        // turn red led on, yellow off
        //GPIO_LED1->BSRR = GPIO_BSRR_BR_LED1;
        //GPIO_LED2->BSRR = GPIO_BSRR_BS_LED2;

        this->ResetState();

        return;
    }

    // flash yellow led, red off
    GPIO_LED1->BSRR = GPIO_BSRR_BS_LED1;
    //GPIO_LED2->BSRR = GPIO_BSRR_BR_LED2;

#ifdef CTRL_POS
    Float_Type tmp_vel;

    if (this->homing)
    {
        tmp_vel = this->HomingVelocity;
    }
    else
    {
        tmp_vel = (this->target_position_pulse - this->current_position_pulse) * Kh * Tc * Kv;
    }

// limit target velocity
    if (MaximumVelocity < tmp_vel)
    {
        this->target_velocity = MaximumVelocity;
    }
    else if (tmp_vel < -MaximumVelocity)
    {
        this->target_velocity = -MaximumVelocity;
    }
    else
    {
        this->target_velocity = tmp_vel;
    }
#endif



    this->error_prev = this->error;
    this->error = this->target_velocity - this->velocity;

    this->u_p = Kp * (this->error - this->error_prev);
    this->u_i = KiTc * this->error;

    this->target_torque += (u_p + u_i);

// limit torque
    if (MaximumTorque < target_torque)
    {
        target_torque = MaximumTorque;
    }
    else if (target_torque < -MaximumTorque)
    {
        target_torque = -MaximumTorque;
    }

    target_voltage = (target_torque * Kg) + (this->velocity * Ke);

    if (MaximumVoltage < target_voltage)
    {
        target_voltage = MaximumVoltage;
    }
    else if (target_voltage < -MaximumVoltage)
    {
        target_voltage = -MaximumVoltage;
    }

// apply output voltage
    SetDuty(target_voltage * 1000 / MaximumVoltage);

    GPIO_LED1->BSRR = GPIO_BSRR_BR_LED1;
}

void MotorCtrl::SetTarget(Float_Type target)
{
#ifdef CTRL_POS
    int tmp = (target * Kr / (Kh * Tc)) + 0.5;

#ifdef LIMIT_POS

    if (MaximumPosition_pulse < tmp)
    {
        this->target_position_pulse = MaximumPosition_pulse;
    }
    else if (tmp < -MaximumPosition_pulse)
    {
        this->target_position_pulse = -MaximumPosition_pulse;
    }
    else
    {
#endif
    this->target_position_pulse = tmp;
#ifdef LIMIT_POS
}
#endif
#else
    double tmp = target * Kr;

    if (MaximumVelocity < tmp)
    {
        this->target_velocity = MaximumVelocity;
    }
    else if (tmp < -MaximumVelocity)
    {
        this->target_velocity = -MaximumVelocity;
    }
    else
    {
        this->target_velocity = tmp;
    }
#endif
}

#ifdef CTRL_POS
void MotorCtrl::ResetPosition(Float_Type offset)
{
    if (!this->shutdown)
    {
        return;
    }

    this->current_position_pulse = offset * Kh * Tc;
    this->target_position_pulse = this->current_position_pulse;
}

void MotorCtrl::LimitSwitch0Handler(void)
{
    if (!this->homing)
    {
        return;
    }

    this->Shutdown();
    this->ResetPosition();
}

void MotorCtrl::LimitSwitch1Handler(void)
{
    if (!this->homing)
    {
        return;
    }

    this->Shutdown();
    this->ResetPosition();
}

void MotorCtrl::Home(void)
{
    if (!this->shutdown)
    {
        return;
    }

    if (((GPIO_DIN0->IDR & GPIO_IDR_DIN0) == 0))    // || ((GPIO_DIN1->IDR & GPIO_IDR_DIN1) == 0))
    {
        this->Shutdown();
        this->ResetPosition();
        this->homing = false;
        return;
    }

    this->homing = true;
    this->_recover();

    led::mode = led::lighting_mode::error_0;
}

#endif

void MotorCtrl::Print(void)
{
    char buf[128];
    int ret;
#ifdef CTRL_POS
    ret = sprintf(buf, "%06lu,%+03d,%+03d,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f\r\n", HAL_GetTick(),
            this->current_position_pulse, this->target_position_pulse, (float) this->velocity, (float) this->target_velocity,
            (float) this->error, (float) this->error_prev, (float) this->u_p, (float) this->u_i, (float) this->target_torque,
            (float) this->target_voltage);
#else
    ret = sprintf(buf, "%06lu,%+03d,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f\r\n", HAL_GetTick(), this->pulse,
            (float) this->velocity, (float) this->target_velocity, (float) this->error, (float) this->error_prev,
            (float) this->u_p, (float) this->u_i, (float) this->target_torque, (float) this->target_voltage);
#endif

    if (ret < 0)
    {
        return;
    }

    serial.write((const uint8_t *) buf, ret);
}

void MotorCtrl::ReadConfig(void)
{
    this->Kp = confStruct.Kp;
    this->KiTc = confStruct.KiTc;
    this->Ke = confStruct.Ke;
    this->Kg = confStruct.Kg;
    this->Kh = confStruct.Kh;
    this->Kr = confStruct.Kr;
    this->Kv = confStruct.Kv;
    this->MaximumVelocity = confStruct.MaxVel;
    this->HomingVelocity = confStruct.HomVel;
    this->MaximumTorque = confStruct.MaxTrq;
    this->SetSupplyVoltage(confStruct.Vsup);
}

void MotorCtrl::WriteConfig(void)
{
    confStruct.Kp = this->Kp;
    confStruct.KiTc = this->KiTc;
    confStruct.Ke = this->Ke;
    confStruct.Kg = this->Kg;
    confStruct.Kh = this->Kh;
    confStruct.Kr = this->Kr;
    confStruct.Kv = this->Kv;
    confStruct.MaxVel = this->MaximumVelocity;
    confStruct.HomVel = this->HomingVelocity;
    confStruct.MaxTrq = this->MaximumTorque;
    confStruct.Vsup = this->SupplyVoltage;
}

