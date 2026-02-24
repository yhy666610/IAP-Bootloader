#include "stm32f4xx.h" // Device header
#include <string.h>
#include "usart.h"
#include <stdio.h>

static void usart_io_init(void)
{
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void usart_lowlevel_init(void)
{
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}
void usart_init(void)
{
    usart_io_init();
    usart_lowlevel_init();
}

void usart_write(USART_TypeDef *USARTx, const char str[], uint32_t length)
{
	for (uint32_t i = 0; i < length; i++)
	{
		USART_ClearFlag(USARTx, USART_FLAG_TC);
		USART_SendData(USARTx, str[i]);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	}
}

