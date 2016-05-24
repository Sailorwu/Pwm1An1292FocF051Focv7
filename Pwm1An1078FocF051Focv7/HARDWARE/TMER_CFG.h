#ifndef _TMER_CFG_H_
#define _TMER_CFG_H_

#include "system_define.h"

extern uint16_t Duty_Temp;
extern uint16_t T2ms_Temp,T100ms_Temp;
extern uint8_t T2ms_Flag,T100ms_Flag;

void TMER_Iinitialization(void);
void Delay(uint32_t nCount);

#endif

