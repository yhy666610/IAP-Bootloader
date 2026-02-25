#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "board.h"
#include "bl_usart.h"
#include "tim_delay.h"
#include "stm32_flash.h"
#include "magic_header.h"
#include "crc16.h"
#include "crc32.h"
#include "ringbuffer.h"
#include "utils.h"

#define LOG_TAG      "bootload"
#define LOG_LVL      ELOG_LVL_INFO
#include "elog.h"

#define BL_VERSION       "v0.9.0"
#define BL_ADDRESS       0x08000000
#define BL_SIZE          (48 * 1024)   // 48KB
#define APP_BASE_ADDRESS 0x08010000
#define RX_TIMEOUT_MS    20
#define RX_BUFFER_SIZE   (5 * 1024)
#define PAYLOAD_SIZE_MAX (4096 + 8) //4096为Program数据最大长度，8为Program指令的地址(4)和大小(4)字段长度
#define PACKET_SIZE_MAX  (4 + PAYLOAD_SIZE_MAX + 2)  // header(1) + opcode(1) + length(2) + payload + crc16(2)
#define BOOT_DELAY       3000

// 协议常数
#define PACKET_HEADER_REQUEST 0xAA
#define PACKET_HEADER_RESPONSE 0x55

// 数据包结构常数
#define PACKET_HEADER_SIZE 1
#define PACKET_OPCODE_SIZE 1
#define PACKET_LENGTH_SIZE 2
#define PACKET_CRC_SIZE 2
#define PACKET_HEADER_OFFSET 0
#define PACKET_OPCODE_OFFSET 1
#define PACKET_LENGTH_OFFSET 2
#define PACKET_PAYLOAD_OFFSET 4
#define PACKET_MIN_SIZE (PACKET_HEADER_SIZE + PACKET_OPCODE_SIZE + PACKET_LENGTH_SIZE + PACKET_CRC_SIZE)

// 参数长度常数
#define ADDR_SIZE_PARAM_LENGTH 8  // uint addr + uint size
#define ADDR_SIZE_CRC_PARAM_LENGTH 12  // uint addr + uint size + uint crc

typedef enum
{
    PACKET_STATE_HEADER,
    PACKET_STATE_OPCODE,
    PACKET_STATE_LENGTH,
    PACKET_STATE_PAYLOAD,
    PACKET_STATE_CRC16
} packet_state_machine_t;
typedef enum
{
    PACKET_OPCODE_INQUERY = 0x01,
    PACKET_OPCODE_ERASE = 0x81,
    PACKET_OPCODE_PROGRAM = 0x82,
    PACKET_OPCODE_VERIFY = 0x83,
    PACKET_OPCODE_RESET = 0x21,
    PACKET_OPCODE_BOOT = 0x22
} packet_opcode_t;
typedef enum
{
    PACKET_ERRCODE_ERR_OK = 0,
    PACKET_ERRCODE_ERR_OPCODE,
    PACKET_ERRCODE_ERR_OVERFLOW,
    PACKET_ERRCODE_ERR_TIMEOUT,
    PACKET_ERRCODE_ERR_FORMAT,
    PACKET_ERRCODE_ERR_VERIFY,
    PACKET_ERRCODE_ERR_PARAM = 0xFF
} PACKET_errcode_t;
typedef enum
{
    INQUERY_SUBCODE_VERSION = 0x00,
    INQUERY_SUBCODE_MTU = 0x01
} inquery_subcode_t;

static packet_state_machine_t packet_state = PACKET_STATE_HEADER;
static uint8_t packet_buf[PACKET_SIZE_MAX];
static uint16_t packet_index;
static packet_opcode_t packet_opcode;
static uint16_t packet_payload_length;

static uint8_t rb_buf[RX_BUFFER_SIZE];
static rb_t rxrb;

/**
 * @brief Validate the application firmware.
 *
 * @return true if the application is valid.
 * @return false if the application is invalid.
 */
static bool application_validate(void)
{
    if (!magic_header_validate())
    {
        log_e("Magic header validation failed.");
        return false;
    }

    uint32_t app_address = magic_header_get_address();
    uint32_t app_length = magic_header_get_length();
    uint32_t app_crc32 = magic_header_get_crc32();
    uint32_t calculated_crc = crc32((uint8_t *)app_address, app_length);
    if (calculated_crc != app_crc32)
    {
        log_e("Application CRC32 mismatch: calculated 0x%08X, expected 0x%08X", calculated_crc, app_crc32);
        return false;
    }

    return true;
}

static void boot_application(void)
{
    if (!application_validate())
    {
        log_e("Application validation failed, cannot boot.");
        return;
    }

    log_w("booting application ... ");
    tim_delay_ms(2);

    led_off(led1);
    TIM_DeInit(TIM6);
    USART_DeInit(USART1);
    USART_DeInit(USART3);
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_DisableIRQ(USART3_IRQn);

    SCB->VTOR = APP_BASE_ADDRESS;
    extern void JumpApp(uint32_t appAddress);
    JumpApp(APP_BASE_ADDRESS);
}
static void bl_response(packet_opcode_t opcode, PACKET_errcode_t errcode, const uint8_t *data, uint16_t length)
{
    uint8_t *response = packet_buf, *prsp = response;
    put_u8_inc(&prsp, PACKET_HEADER_RESPONSE);
    put_u8_inc(&prsp, (uint8_t)opcode);
    put_u8_inc(&prsp, (uint8_t)errcode);
    put_u16_inc(&prsp, length);
    put_bytes_inc(&prsp, data, length);
    uint16_t crc = crc16(response, prsp - response);
    put_u16_inc(&prsp, crc);

    bl_usart_write(response, prsp - response);
}

// static inline void bl_opcode_response_ack(packet_opcode_t opcode, PACKET_errcode_t errcode)
// {
//     bl_response(opcode, errcode, NULL, 0);
// }

static void bl_opcode_inquery_handler(void)
{
    log_i("Inquary handler.");
    if (packet_payload_length != 1)
    {
        log_e("Inquary packet length error");
        return;
    }

    uint8_t subcode = get_u8(packet_buf + PACKET_PAYLOAD_OFFSET);
    switch (subcode)
    {
        case INQUERY_SUBCODE_VERSION:
		{
			bl_response(PACKET_OPCODE_INQUERY, PACKET_ERRCODE_ERR_OK, (const uint8_t *)BL_VERSION, strlen(BL_VERSION));
            break;
		}
        case INQUERY_SUBCODE_MTU:
		{
			uint8_t mtu[2];
            put_u16(mtu, PAYLOAD_SIZE_MAX);
            bl_response(PACKET_OPCODE_INQUERY, PACKET_ERRCODE_ERR_OK, (const uint8_t *)&mtu, sizeof(mtu));
            break;
		}
        default:
		{
            log_w("Inquary unknown subcode: %02X", subcode);
            break;
        }
    }
}

static void bl_opcode_erase_handler(void)
{
    log_i("Erase handler.");

    if (packet_payload_length != ADDR_SIZE_PARAM_LENGTH)
    {
        log_e("Erase packet length error: %d", packet_payload_length);
        bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_ERR_FORMAT, NULL, 0);
        return;
    }

    uint8_t *payload = packet_buf + PACKET_PAYLOAD_OFFSET;
    uint32_t address = get_u32_inc(&payload);// 从payload中提取出地址字段，address表示要擦除的flash内存起始地址
    uint32_t size = get_u32_inc(&payload);   // 从payload中提取出大小字段，size表示要擦除的数据长度

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        log_e("Erase address: 0x%08X, size: %u out of range", address, size);
        bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    if (address >= BL_ADDRESS && address < BL_ADDRESS + BL_SIZE)
    {
        log_e("address 0x%08X is protected", address);
        bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    log_d("Erase address: 0x%08X, size: %u", address, size);

    stm32_flash_unlock();
    stm32_flash_erase(address, size);
    stm32_flash_lock();

    bl_response(PACKET_OPCODE_ERASE, PACKET_ERRCODE_ERR_OK, NULL, 0);
}

static void bl_opcode_program_handler(void)
{
    log_i("Program handler.");

    if (packet_payload_length <= ADDR_SIZE_PARAM_LENGTH)
    {
        log_e("Program packet length error: %d", packet_payload_length);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_FORMAT, NULL, 0);
        return;
    }

    uint8_t *payload = packet_buf + PACKET_PAYLOAD_OFFSET;
    uint32_t address = get_u32_inc(&payload);// 从payload中提取出地址字段，address表示要写入的flash内存起始地址
    uint32_t size = get_u32_inc(&payload);   // 从payload中提取出大小字段，size表示要写入的数据长度
    uint8_t *data = payload;                 // data指向剩余的payload数据，即要写入flash的数据

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        log_e("Program address: 0x%08X, size: %u out of range", address, size);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    if (address >= BL_ADDRESS && address < BL_ADDRESS + BL_SIZE)
    {
        log_e("address 0x%08X is in protected bootloader region", address);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    if (size !=packet_payload_length - ADDR_SIZE_PARAM_LENGTH)
    {
        log_e("Program size %u does not match packet payload length %u", size, packet_payload_length - ADDR_SIZE_PARAM_LENGTH);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_FORMAT, NULL, 0);
        return;
    }

    log_i("Program address: 0x%08X, size: %u", address, size);

    stm32_flash_unlock();
    stm32_flash_program(address, data, size);
    stm32_flash_lock();

    bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_OK, NULL, 0);
}

static void bl_opcode_verify_handler(void)
{
    log_i("Verify handler.");

    if (packet_payload_length != ADDR_SIZE_CRC_PARAM_LENGTH)
    {
        log_e("Verify packet length error: %d", packet_payload_length);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_ERR_FORMAT, NULL, 0);
        return;
    }

    uint8_t *payload = packet_buf + PACKET_PAYLOAD_OFFSET;
    uint32_t address = get_u32_inc(&payload);// 从payload中提取出地址字段，address表示要验证的flash内存起始地址
    uint32_t size = get_u32_inc(&payload);   // 从payload中提取出大小字段，size表示要验证的数据长度
    uint32_t crc = get_u32_inc(&payload);    // 从payload中提取出CRC32字段，crc表示预期的CRC32校验值

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        log_e("Verify address: 0x%08X, size: %u out of range", address, size);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    log_d("Verify address: 0x%08X, size: %u, CRC: 0x%08X", address, size, crc);

    uint32_t calculated_crc = crc32((uint8_t *)address, size);//计算指定地址和大小范围内的数据的CRC32校验值，指定地址为flash内存起始地址
    if (calculated_crc != crc)
    {
        log_e("Verify CRC mismatch: calculated 0x%08X, expected 0x%08X", calculated_crc, crc);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_ERR_VERIFY, NULL, 0);
        return;
    }

    bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_ERR_OK, NULL, 0);
}

static void bl_opcode_reset_handler(void)
{
    log_i("Reset handler.");
    bl_response(PACKET_OPCODE_RESET, PACKET_ERRCODE_ERR_OK, NULL, 0);
    log_w("system resetting...");
    tim_delay_ms(2);
    // 执行系统复位
    NVIC_SystemReset();
}

static void bl_opcode_boot_handler(void)
{
    log_i("Boot handler.");
    bl_response(PACKET_OPCODE_BOOT, PACKET_ERRCODE_ERR_OK, NULL, 0);
    boot_application();
}

static void bl_packet_handler(void)
{
    switch (packet_opcode)
    {
        case PACKET_OPCODE_INQUERY:
            bl_opcode_inquery_handler();
            break;
        case PACKET_OPCODE_ERASE:
             bl_opcode_erase_handler();
            break;
        case PACKET_OPCODE_PROGRAM:
             bl_opcode_program_handler();
            break;
        case PACKET_OPCODE_VERIFY:
             bl_opcode_verify_handler();
            break;
        case PACKET_OPCODE_RESET:
             bl_opcode_reset_handler();
            break;
        case PACKET_OPCODE_BOOT:
             bl_opcode_boot_handler();
            break;
        default:
            log_w("Unknown opcode: %02X", packet_opcode);
            break;
    }
}

static bool bl_byte_handler(uint8_t byte)
{
    bool full_packet = false;
    // 处理字节数据超时接受
    static uint64_t last_byte_ms;
    uint64_t now_ms = tim_get_ms();
    if (now_ms - last_byte_ms > RX_TIMEOUT_MS)
    {
        if (packet_state != PACKET_STATE_HEADER)
            log_w("last packet rx timeout");
        packet_index = 0;
        packet_state = PACKET_STATE_HEADER;
    }
    last_byte_ms = now_ms;

    //字节接收状态机处理
    //printf("recv: %02X", byte); // 调试输出接收字节,耗时,后期要关闭
    log_v("recv: %02X", byte); // 详细日志级别输出接收字节
    packet_buf[packet_index++] = byte;
    switch (packet_state)
    {
        case PACKET_STATE_HEADER:
            if (packet_buf[PACKET_HEADER_OFFSET] == PACKET_HEADER_REQUEST)
                {
			log_d("header ok");
                    packet_state = PACKET_STATE_OPCODE;
                }
                else
                {
                    log_w("header error: %02X", packet_buf[PACKET_HEADER_OFFSET]);
                    // 如果不是头部字节，重置状态机
                    packet_index = 0;
                    packet_state = PACKET_STATE_HEADER;
                }
                break;
            case PACKET_STATE_OPCODE:
                if (packet_buf[PACKET_OPCODE_OFFSET] == PACKET_OPCODE_INQUERY ||
                    packet_buf[PACKET_OPCODE_OFFSET] == PACKET_OPCODE_ERASE ||
                    packet_buf[PACKET_OPCODE_OFFSET] == PACKET_OPCODE_PROGRAM ||
                    packet_buf[PACKET_OPCODE_OFFSET] == PACKET_OPCODE_VERIFY ||
                    packet_buf[PACKET_OPCODE_OFFSET] == PACKET_OPCODE_RESET ||
                    packet_buf[PACKET_OPCODE_OFFSET] == PACKET_OPCODE_BOOT)
                {
			log_d("opcode ok: %02X", packet_buf[PACKET_OPCODE_OFFSET]);
                    packet_opcode = (packet_opcode_t)packet_buf[PACKET_OPCODE_OFFSET];
                    packet_state = PACKET_STATE_LENGTH;
                }
                else
                {
                    log_w("opcode error: %02X", packet_buf[PACKET_OPCODE_OFFSET]);
                    // 如果不是有效的操作码，重置状态机
                    packet_index = 0;
                    packet_state = PACKET_STATE_HEADER;
                }
                break;
            case PACKET_STATE_LENGTH:
                if (packet_index == PACKET_PAYLOAD_OFFSET)
                {
                    uint16_t payload_length = get_u16(packet_buf + PACKET_LENGTH_OFFSET); // 另一种写法：&packet_buf[PACKET_LENGTH_OFFSET]
                    if (payload_length <= PAYLOAD_SIZE_MAX)
			{
				log_d("length ok: %u", payload_length);
                        packet_payload_length = payload_length;
                        if (packet_payload_length > 0)
                        packet_state = PACKET_STATE_PAYLOAD;
                        else
                        packet_state = PACKET_STATE_CRC16;
                    }
                    else
                    {
                        log_w("length error: %u, max is %u", payload_length, PAYLOAD_SIZE_MAX);
                        // 如果长度超过最大值，重置状态机
                        packet_index = 0;
                        packet_state = PACKET_STATE_HEADER;
                    }
                }
                break;
            case PACKET_STATE_PAYLOAD:
                if (packet_index == PACKET_PAYLOAD_OFFSET + packet_payload_length)
                {
			log_d("payload receive ok");
                    packet_state = PACKET_STATE_CRC16;
                }
                break;
                case PACKET_STATE_CRC16:
                if (packet_index == PACKET_MIN_SIZE + packet_payload_length)
                {
                    uint16_t crc = get_u16(packet_buf + PACKET_PAYLOAD_OFFSET + packet_payload_length);
                    uint16_t calculated_crc = crc16(packet_buf, PACKET_PAYLOAD_OFFSET + packet_payload_length);
                    if (crc == calculated_crc)
                    {
                        full_packet = true;
                        log_d("crc16 ok: %04X", crc);
                        log_d("packet received: optcode=%02X, length=%u", packet_opcode, packet_payload_length);
                        if (LOG_LVL >=ELOG_LVL_VERBOSE)
                            elog_hexdump("payload", 16, packet_buf, PACKET_MIN_SIZE + packet_payload_length);

                            //耗时的调试输出,后期要关闭
                        // printf("payloda: ");
                        // for (uint32_t i = 0; i < packet_payload_length; i++)
                        // {
                        //     printf("%02X ", packet_buf[4 + i]);
                        // }
                        // printf("");
                    }
                    else
                    {
                        log_w("crc16 error: expected %04X, got %04X", crc, calculated_crc);
                    }

                    packet_index = 0;
                    packet_state = PACKET_STATE_HEADER;
                }
                break;
            default:
            break;
    }
    return full_packet;
}

static void bl_rx_handler(const uint8_t *data, uint32_t length)
{
    rb_puts(rxrb, data, length);
}

static bool key_trap_check(void)
{   // 检测按键是否在BOOT_DELAY时间内被按下
    for (uint32_t t = 0; t < BOOT_DELAY; t += 10)
    {
        tim_delay_ms(10);
        if (!key_read(key3))
            return false;
    }
    log_w("Key pressed, entering bootloader mode.");
    return true;
}

static void wait_key_release(void)
{
    while (key_read(key3))
        tim_delay_ms(10);
}

static bool key_press_check(void)
{
    if (!key_read(key3))
        return false;

    tim_delay_ms(10);
    if (!key_read(key3))
        return false;

    return true;
}

bool magic_header_trap_boot(void)
{
    if (!magic_header_validate())
    {
        log_w("Magic header invalid, entering bootloader mode.");
        return true;
    }

    if (!application_validate())
    {
        log_w("Application invalid, entering bootloader mode.");
        return true;
    }

    return false;
}

bool rx_trap_boot(void)
{
    for (uint32_t t = 0; t < BOOT_DELAY; t += 10)
    {
        tim_delay_ms(10);
        if (!rb_empty(rxrb))
        {
            log_w("Data received, entering bootloader mode.");
            return true;
        }
    }

    return false;
}

void bootloader_main(void)
{
    log_i("Bootloader started.");

    key_init(key3);

    rxrb = rb_new(rb_buf, RX_BUFFER_SIZE);
    bl_usart_init();
    bl_usart_register_rx_callback(bl_rx_handler);


    bool trap_boot = false;

    if (!trap_boot)
        trap_boot = magic_header_trap_boot();

    if (!trap_boot)
        trap_boot = key_trap_check();

    if (!trap_boot)
        trap_boot = rx_trap_boot();

    if (!trap_boot)
        boot_application();

    led_init(led1);
    led_on(led1);
    wait_key_release();

    while (1)
    {
        if (key_press_check())
        {
            log_w("key pressed, rebooting...");
            tim_delay_ms(2);
            NVIC_SystemReset();
        }
        if (!rb_empty(rxrb))
        {
            uint8_t byte;
            rb_get(rxrb, &byte);
            if (bl_byte_handler(byte))
            {
                bl_packet_handler();
            }
        }
    }
}