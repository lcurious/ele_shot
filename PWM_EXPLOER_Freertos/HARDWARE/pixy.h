#ifndef _PIXY_H
#define _PIXY_H

#include "sys.h"
void USAR4_Init(u32 bound);

#define image_center_offset 78

extern float image_x;

float get_image_ave_angle(uint16_t time_delay);

#endif 
