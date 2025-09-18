#ifndef _ZF_DEVICE_SYSTICK_H_
#define _ZF_DEVICE_SYSTICK_H_

#include "zf_common_headfile.h"

void systick_init(void);
uint32 systick_get_time_ms(void);
void systick_delay_ms(uint32 ms);


#endif

