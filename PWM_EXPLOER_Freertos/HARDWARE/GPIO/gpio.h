#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"
#include "gpio.h"

void control_gpio_init(void);

#define con_charge PDout(2)
#define con_shot   PDout(3)
#endif

