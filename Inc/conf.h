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
        uint16_t can_id_vel;
        uint16_t can_id_stat;
        double Kp;
        double KiTc;
        double Ke;
        double Kg;
        double Kh;
        double Kr;
        double MaxVel;
        double MaxTrq;
        double Vsup;
    } ConfStruct;
}

extern ConfStruct confStruct;

void readConf(void);
void writeConf(void);

#endif /* CONF_H_ */
