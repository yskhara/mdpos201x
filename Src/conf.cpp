/*
 * conf.c
 *
 *  Created on: Dec 27, 2018
 *      Author: yusaku
 */

#include "flash.h"
#include "conf.h"

ConfStruct confStruct;

void readConf(void)
{
    loadFlash(DATA_PAGE_ADDR, (uint8_t*)&confStruct, sizeof(ConfStruct));
}

void writeConf(void)
{
    writeFlash(DATA_PAGE_ADDR, (uint16_t*)&confStruct, sizeof(ConfStruct));
}


