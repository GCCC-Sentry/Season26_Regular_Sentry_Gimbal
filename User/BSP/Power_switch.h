#ifndef __POWER_SWITCH_H
#define __POWER_SWITCH_H

#include "stm32h7xx_hal.h"

typedef enum
{
    power1,
    power2,
} powertyp;

void Power_Set(powertyp power, GPIO_PinState status);
void Power_TurnOff(powertyp power);
void Power_TurnOn(powertyp power);


#endif // !__POWER_SWITCH_H

