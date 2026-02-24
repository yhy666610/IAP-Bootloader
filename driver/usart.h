#ifndef __USART_H__
#define __USART_H__

#include <stdbool.h>
#include "stm32f4xx.h"

void usart_write(USART_TypeDef* USARTx, const char str[], uint32_t length);
void usart_init(void);

#endif /* __USART_H__ */
