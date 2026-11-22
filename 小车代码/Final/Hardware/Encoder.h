#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"

void Encoder_Init(void);

int16_t Encoder_GetLeft(void);    // ×óÂÖ£¬TIM3£¨PB6/PB7£©
int16_t Encoder_GetRight(void);   // ÓÒÂÖ£¬TIM4£¨PA6/PA7£©

#endif


