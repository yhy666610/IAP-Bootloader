#include <stdio.h>
#include "stm32f4xx.h"
#include "key.h"
#include "key_desc.h"
#include "led.h"
#include "led_desc.h"

void board_lowlevel_init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}

int fputc(int ch, FILE *f)
{

    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}

static struct led_desc _led1 = { GPIOE, GPIO_Pin_9, Bit_RESET, Bit_SET};
static struct led_desc _led2 = { GPIOB, GPIO_Pin_0, Bit_RESET, Bit_SET};
static struct led_desc _led3 = { GPIOB, GPIO_Pin_1, Bit_RESET, Bit_SET};
led_desc_t led1 = &_led1;
led_desc_t led2 = &_led2;
led_desc_t led3 = &_led3;

static struct key_desc _key1 = { GPIOA, GPIO_Pin_0, GPIO_PuPd_UP, Bit_RESET};
static struct key_desc _key2 = { GPIOC, GPIO_Pin_4, GPIO_PuPd_UP, Bit_RESET};
static struct key_desc _key3 = { GPIOC, GPIO_Pin_5, GPIO_PuPd_UP, Bit_RESET};
key_desc_t key1 = &_key1;
key_desc_t key2 = &_key2;
key_desc_t key3 = &_key3;
