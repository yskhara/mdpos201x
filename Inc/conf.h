/*
 * conf.h
 *
 *  Created on: Dec 27, 2018
 *      Author: yusaku
 */

#ifndef CONF_H_
#define CONF_H_

#include "stm32f1xx_hal.h"

extern "C"
{
    typedef struct
    {
        uint16_t can_id_cmd;
        //uint16_t can_id_vel;
        //uint16_t can_id_stat;
        float Kp;
        float Ki;
        float Ke;
        float Kg;
        float Ppr;
        float Kr;
        float MaxVel;
        float MaxTrq;
        float Vsup;
        float HomVel;
    } ConfStruct;
}

extern ConfStruct confStruct;

void readConf(void);
void writeConf(void);

inline ConfStruct * getConf(void)
{
    return &confStruct;
}

#endif /* CONF_H_ */
