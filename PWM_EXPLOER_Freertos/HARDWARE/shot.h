#ifndef _SHOT_
#define _SHOT_

#include "sys.h"
#include "gpio.h"

void charge(void);
void wait_for_shot(void);

extern uint8_t charge_flag;
#endif 