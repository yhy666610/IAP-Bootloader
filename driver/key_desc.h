#ifndef __KEY_DESC_H__
#define __KEY_DESC_H__

#include <stdint.h>
#include "stm32f4xx.h"
#include "key.h"

struct key_desc
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIOPuPd_TypeDef pupd;
    BitAction press_level;
};

#endif /* __KEY_H__ */
