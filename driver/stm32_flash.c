#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32_flash.h"
#include "utils.h"

#define LOG_TAG      "flash"
#define LOG_LVL      ELOG_LVL_INFO
#include "elog.h"

#define FLASH_BASE_ADDRESS 0x08000000

typedef struct
{
    uint32_t sector;
    uint32_t size;
} sector_desc_t;

static const sector_desc_t sector_descs[] =
{
    {FLASH_Sector_0, 16 * 1024},   // 0x08000000-0x08003FFF
    {FLASH_Sector_1, 16 * 1024},   // 0x08004000-0x08007FFF
    {FLASH_Sector_2, 16 * 1024},   // 0x08008000-0x0800BFFF
    {FLASH_Sector_3, 16 * 1024},   // 0x0800C000-0x0800FFFF
    {FLASH_Sector_4, 64 * 1024},   // 0x08010000-0x0801FFFF
    {FLASH_Sector_5, 128 * 1024},  // 0x08020000-0x0803FFFF
    {FLASH_Sector_6, 128 * 1024},  // 0x08040000-0x0805FFFF
    {FLASH_Sector_7, 128 * 1024},  // 0x08060000-0x0807FFFF
    {FLASH_Sector_8, 128 * 1024},  // 0x08080000-0x0809FFFF
    {FLASH_Sector_9, 128 * 1024},  // 0x080A0000-0x080BFFFF
    {FLASH_Sector_10, 128 * 1024}, // 0x080C0000-0x080DFFFF
    {FLASH_Sector_11, 128 * 1024}  // 0x080E0000-0x080FFFFF
};


void stm32_flash_lock(void)
{
    FLASH_Lock();
}
void stm32_flash_unlock(void)
{
    FLASH_Unlock();
}

void stm32_flash_erase(uint32_t address, uint32_t size)
{
    uint32_t addr = FLASH_BASE_ADDRESS;
    for (uint32_t i = 0; i < ARRAY_SIZE(sector_descs); i++)
    {
        if (addr >= address && addr < address + size)
        {
            log_i("Erasing sector %lu at address 0x%08lX size %lu", i, addr, sector_descs[i].size);
            if (FLASH_EraseSector(sector_descs[i].sector, VoltageRange_3) != FLASH_COMPLETE)
            {
                log_e("Error erasing sector %lu", i);
            }
        }
        addr += sector_descs[i].size;
    }
}

void stm32_flash_program(uint32_t addr, const uint8_t *data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i += 4)
    {
        if (FLASH_ProgramWord(addr + i, *(uint32_t *)(data + i)) != FLASH_COMPLETE)
        {
            log_e("Error programming word at address 0x%08lX", addr + i);
            break; //作用是停止编程过程，防止继续写入可能导致错误的数据，用return也是可以的
        }
    }
}
