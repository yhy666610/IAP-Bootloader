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

// LED 闪烁周期定义（单位 ms）
// 正常升级模式：LED 常亮
// 固件损坏模式：LED 快速闪烁，每 200ms 翻转一次（即 400ms 一个完整周期）
#define LED_BLINK_ERROR_INTERVAL_MS  200

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
 * @brief 标志位：固件验证失败导致进入升级模式。
 *        true  -> 固件损坏，LED 快速闪烁提示用户需要重新烧写固件。
 *        false -> 正常/主动进入升级模式，LED 常亮。
 */
static bool boot_validate_failed = false;
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
        /* 置位标志，通知主循环以 LED 闪烁区分\"固件损坏\"状态 */
        boot_validate_failed = true;
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

    if (packet_payload_length < ADDR_SIZE_PARAM_LENGTH)
    {
        log_e("Program packet length error: %d", packet_payload_length);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_FORMAT, NULL, 0);
        return;
    }

    uint8_t *payload = packet_buf + PACKET_PAYLOAD_OFFSET;
    uint32_t address = get_u32_inc(&payload);
    uint32_t size = get_u32_inc(&payload);

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        log_e("Program address: 0x%08X, size: %u out of range", address, size);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    if (address >= BL_ADDRESS && address < BL_ADDRESS + BL_SIZE)
    {
        log_e("address 0x%08X is protected", address);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    if (packet_payload_length != ADDR_SIZE_PARAM_LENGTH + size)
    {
        log_e("Program packet data length mismatch: expected %lu, got %d",
              ADDR_SIZE_PARAM_LENGTH + size, packet_payload_length);
        bl_response(PACKET_OPCODE_PROGRAM, PACKET_ERRCODE_ERR_FORMAT, NULL, 0);
        return;
    }

    log_d("Program address: 0x%08X, size: %lu", address, size);

    stm32_flash_unlock();
    stm32_flash_program(address, payload, size);
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
    uint32_t address = get_u32_inc(&payload);
    uint32_t size = get_u32_inc(&payload);
    uint32_t expected_crc = get_u32_inc(&payload);

    if (address < STM32_FLASH_BASE || address + size > STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
        log_e("Verify address: 0x%08X, size: %u out of range", address, size);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_ERR_PARAM, NULL, 0);
        return;
    }

    uint32_t calculated_crc = crc32((uint8_t *)address, size);
    if (calculated_crc != expected_crc)
    {
        log_e("Verify CRC32 mismatch: calculated 0x%08X, expected 0x%08X", calculated_crc, expected_crc);
        bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_ERR_VERIFY, NULL, 0);
        return;
    }

    log_i("Verify OK.");
    bl_response(PACKET_OPCODE_VERIFY, PACKET_ERRCODE_ERR_OK, NULL, 0);
}

static void bl_opcode_reset_handler(void)
{
    log_w("Reset handler, rebooting...");
    tim_delay_ms(2);
    NVIC_SystemReset();
}

static void bl_opcode_boot_handler(void)
{
    log_i("Boot handler.");
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
            log_w("Unknown opcode: 0x%02X", packet_opcode);
            bl_response(packet_opcode, PACKET_ERRCODE_ERR_OPCODE, NULL, 0);
            break;
    }
}

static bool bl_byte_handler(uint8_t byte)
{
    static uint16_t rx_length_remaining = 0;
    static uint64_t last_rx_time = 0;

    uint64_t now = tim_get_ms();

    // 超时重置状态机
    if (packet_state != PACKET_STATE_HEADER && (now - last_rx_time) > RX_TIMEOUT_MS)
    {
        log_w("RX timeout, reset state machine.");
        packet_state = PACKET_STATE_HEADER;
        packet_index = 0;
    }
    last_rx_time = now;

    switch (packet_state)
    {
        case PACKET_STATE_HEADER:
            if (byte == PACKET_HEADER_REQUEST)
            {
                packet_index = 0;
                packet_buf[packet_index++] = byte;
                packet_state = PACKET_STATE_OPCODE;
            }
            break;

        case PACKET_STATE_OPCODE:
            packet_buf[packet_index++] = byte;
            packet_opcode = (packet_opcode_t)byte;
            packet_state = PACKET_STATE_LENGTH;
            rx_length_remaining = PACKET_LENGTH_SIZE;
            break;

        case PACKET_STATE_LENGTH:
            packet_buf[packet_index++] = byte;
            rx_length_remaining--;
            if (rx_length_remaining == 0)
            {
                packet_payload_length = get_u16(packet_buf + PACKET_LENGTH_OFFSET);
                if (packet_payload_length > PAYLOAD_SIZE_MAX)
                {
                    log_e("Payload length overflow: %d", packet_payload_length);
                    bl_response(packet_opcode, PACKET_ERRCODE_ERR_OVERFLOW, NULL, 0);
                    packet_state = PACKET_STATE_HEADER;
                    packet_index = 0;
                    break;
                }
                if (packet_payload_length == 0)
                {
                    packet_state = PACKET_STATE_CRC16;
                    rx_length_remaining = PACKET_CRC_SIZE;
                }
                else
                {
                    packet_state = PACKET_STATE_PAYLOAD;
                    rx_length_remaining = packet_payload_length;
                }
            }
            break;

        case PACKET_STATE_PAYLOAD:
            packet_buf[packet_index++] = byte;
            rx_length_remaining--;
            if (rx_length_remaining == 0)
            {
                packet_state = PACKET_STATE_CRC16;
                rx_length_remaining = PACKET_CRC_SIZE;
            }
            break;

        case PACKET_STATE_CRC16:
            packet_buf[packet_index++] = byte;
            rx_length_remaining--;
            if (rx_length_remaining == 0)
            {
                // 校验 CRC16
                uint16_t received_crc = get_u16(packet_buf + PACKET_MIN_SIZE - PACKET_CRC_SIZE + packet_payload_length);
                uint16_t calculated_crc = crc16(packet_buf, PACKET_MIN_SIZE - PACKET_CRC_SIZE + packet_payload_length);
                packet_state = PACKET_STATE_HEADER;
                packet_index = 0;
                if (received_crc != calculated_crc)
                {
                    log_e("CRC16 mismatch: calculated 0x%04X, received 0x%04X", calculated_crc, received_crc);
                    bl_response(packet_opcode, PACKET_ERRCODE_ERR_FORMAT, NULL, 0);
                    break;
                }
                return true; // 完整且合法的数据包
            }
            break;

        default:
            packet_state = PACKET_STATE_HEADER;
            packet_index = 0;
            break;
    }
    return false;
}

static void bl_rx_handler(const uint8_t *data, uint32_t length)
{
    rb_puts(rxrb, data, length);
}

static bool key_trap_check(void)
{
    bool pressed = key_read(key3);
    if (pressed)
        log_w("Key trap: key3 is pressed, entering bootloader.");
    return pressed;
}

static void wait_key_release(void)
{
    while (key_read(key3));
}

static bool key_press_check(void)
{
    return key_read(key3);
}

static bool magic_header_trap_boot(void)
{
    // 如果 magic header 无效（未烧写过固件），不触发 trap
    if (!magic_header_validate())
        return false;

    // 仅当 data_type 为特殊触发类型时才 trap（此处扩展预留，当前 APP 类型不触发）
    magic_header_type_t type = magic_header_get_type();
    if (type != MAGIC_HEADER_TYPE_APP)
    {
        log_w("Magic header trap: type=%d, entering bootloader.", type);
        return true;
    }
    return false;
}

static bool rx_trap_boot(void)
{
    log_i("Waiting for rx trap boot (%d ms)...", BOOT_DELAY);
    uint64_t start = tim_get_ms();
    while (tim_get_ms() - start < BOOT_DELAY)
    {
        if (!rb_empty(rxrb))
        {
            uint8_t byte;
            rb_get(rxrb, &byte);
            if (bl_byte_handler(byte))
            {
                if (packet_opcode == PACKET_OPCODE_INQUERY)
                {
                    log_w("RX trap: received INQUERY, entering bootloader.");
                    bl_packet_handler();
                    return true;
                }
            }
        }
    }
    return false;
}

/**
 * @brief 固件损坏错误指示：LED 快速闪烁。
 *        每调用一次检测是否到达翻转时刻，到达则翻转 LED。
 *        该函数应在主循环中非阻塞地周期性调用。
 */
static void led_blink_error(void)
{
    static uint64_t last_toggle_ms = 0;
    uint64_t now = tim_get_ms();
    if (now - last_toggle_ms >= LED_BLINK_ERROR_INTERVAL_MS)
    {
        last_toggle_ms = now;
        /* 利用 led_set 配合静态状态变量实现翻转 */
        static bool led_state = false;
        led_state = !led_state;
        led_set(led1, led_state);
    }
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

    if (boot_validate_failed)
    {
        /* 固件损坏：打印明确提示，LED 初始关闭（由闪烁逻辑控制） */
        log_e("!!! Firmware corrupted or missing, please re-flash the application !!!");
        led_off(led1);
    }
    else
    {
        /* 正常/主动进入升级模式：LED 常亮 */
        log_i("Bootloader upgrade mode, LED on.");
        led_on(led1);
    }

    wait_key_release();

    while (1)
    {
        if (key_press_check())
        {
            log_w("key pressed, rebooting...");
            tim_delay_ms(2);
            NVIC_SystemReset();
        }

        /* 固件损坏时驱动 LED 快速闪烁，正常模式下 LED 常亮无需处理 */
        if (boot_validate_failed)
        {
            led_blink_error();
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