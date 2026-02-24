#ifndef __STM32_FLASH_H__
#define __STM32_FLASH_H__
#include <stdint.h>


#define STM32_FLASH_BASE    0x08000000
#define STM32_FLASH_SIZE    (512 * 1024)
void stm32_flash_lock(void);
void stm32_flash_unlock(void);
void stm32_flash_erase(uint32_t addr, uint32_t size);
void stm32_flash_program(uint32_t addr, const uint8_t *data, uint32_t size);


#endif /* __STM32_FLASH_H__ */
