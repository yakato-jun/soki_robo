#include "modbus.h"
#include "usart.h"

/* Central register store */
uint16_t holding_regs[MODBUS_REG_COUNT];

/* RX buffer (filled by UART0 ISR) */
uint8_t  modbus_rx_buf[MODBUS_RX_BUF_SIZE];
uint16_t modbus_rx_len = 0;
volatile uint8_t modbus_frame_ready = 0;
volatile uint8_t modbus_cmd_written = 0;

/* ---- CRC-16 (Modbus polynomial 0xA001, bit-by-bit) ---- */
uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t  j;

    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ---- Send response via UART0 ---- */
void modbus_send_response(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        USART_SendData(data[i]);
    }
}

/* ---- Send error response ---- */
static void modbus_send_exception(uint8_t fc, uint8_t exception)
{
    uint8_t resp[5];
    uint16_t crc;

    resp[0] = MODBUS_SLAVE_ADDR;
    resp[1] = fc | 0x80;   /* error flag */
    resp[2] = exception;
    crc = modbus_crc16(resp, 3);
    resp[3] = crc & 0xFF;
    resp[4] = (crc >> 8) & 0xFF;
    modbus_send_response(resp, 5);
}

/* ---- FC03: Read Holding Registers ---- */
static void modbus_fc03(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t start_addr, quantity;
    uint8_t  resp[5 + MODBUS_REG_COUNT * 2]; /* max response */
    uint16_t byte_count, crc;
    uint16_t i;

    if (frame_len < 8) return;

    start_addr = ((uint16_t)frame[2] << 8) | frame[3];
    quantity   = ((uint16_t)frame[4] << 8) | frame[5];

    /* Validate range */
    if (quantity == 0 || quantity > 125 ||
        start_addr >= MODBUS_REG_COUNT ||
        (start_addr + quantity) > MODBUS_REG_COUNT) {
        modbus_send_exception(0x03, MODBUS_EX_ILLEGAL_ADDRESS);
        return;
    }

    byte_count = quantity * 2;
    resp[0] = MODBUS_SLAVE_ADDR;
    resp[1] = 0x03;
    resp[2] = (uint8_t)byte_count;

    for (i = 0; i < quantity; i++) {
        resp[3 + i * 2]     = (holding_regs[start_addr + i] >> 8) & 0xFF;
        resp[3 + i * 2 + 1] = holding_regs[start_addr + i] & 0xFF;
    }

    crc = modbus_crc16(resp, 3 + byte_count);
    resp[3 + byte_count]     = crc & 0xFF;
    resp[3 + byte_count + 1] = (crc >> 8) & 0xFF;

    modbus_send_response(resp, 5 + byte_count);
}

/* ---- FC06: Write Single Register ---- */
static void modbus_fc06(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t reg_addr, value;
    uint8_t  resp[8];
    uint16_t crc;

    if (frame_len < 8) return;

    reg_addr = ((uint16_t)frame[2] << 8) | frame[3];
    value    = ((uint16_t)frame[4] << 8) | frame[5];

    /* Check writable range */
    if (reg_addr < REG_WRITE_START || reg_addr > REG_WRITE_END) {
        modbus_send_exception(0x06, MODBUS_EX_ILLEGAL_ADDRESS);
        return;
    }

    holding_regs[reg_addr] = value;
    modbus_cmd_written = 1;

    /* Echo the request as response */
    resp[0] = frame[0];
    resp[1] = frame[1];
    resp[2] = frame[2];
    resp[3] = frame[3];
    resp[4] = frame[4];
    resp[5] = frame[5];
    crc = modbus_crc16(resp, 6);
    resp[6] = crc & 0xFF;
    resp[7] = (crc >> 8) & 0xFF;

    modbus_send_response(resp, 8);
}

/* ---- FC16: Write Multiple Registers ---- */
static void modbus_fc16(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t start_addr, quantity;
    uint8_t  byte_count;
    uint8_t  resp[8];
    uint16_t crc;
    uint16_t i;

    if (frame_len < 9) return;

    start_addr = ((uint16_t)frame[2] << 8) | frame[3];
    quantity   = ((uint16_t)frame[4] << 8) | frame[5];
    byte_count = frame[6];

    /* Validate */
    if (quantity == 0 || quantity > 123 || byte_count != quantity * 2) {
        modbus_send_exception(0x10, MODBUS_EX_ILLEGAL_VALUE);
        return;
    }
    if (start_addr < REG_WRITE_START ||
        (start_addr + quantity - 1) > REG_WRITE_END) {
        modbus_send_exception(0x10, MODBUS_EX_ILLEGAL_ADDRESS);
        return;
    }
    if (frame_len < (uint16_t)(9 + byte_count)) return;

    /* Write registers */
    for (i = 0; i < quantity; i++) {
        holding_regs[start_addr + i] =
            ((uint16_t)frame[7 + i * 2] << 8) | frame[7 + i * 2 + 1];
    }
    modbus_cmd_written = 1;

    /* Response: addr + FC + start_addr + quantity + CRC */
    resp[0] = MODBUS_SLAVE_ADDR;
    resp[1] = 0x10;
    resp[2] = (start_addr >> 8) & 0xFF;
    resp[3] = start_addr & 0xFF;
    resp[4] = (quantity >> 8) & 0xFF;
    resp[5] = quantity & 0xFF;
    crc = modbus_crc16(resp, 6);
    resp[6] = crc & 0xFF;
    resp[7] = (crc >> 8) & 0xFF;

    modbus_send_response(resp, 8);
}

/* ---- Process received Modbus frame ---- */
void modbus_process_frame(void)
{
    uint16_t crc_received, crc_calculated;
    uint8_t  addr, fc;

    /* Minimum frame: addr(1) + fc(1) + crc(2) = 4 bytes */
    if (modbus_rx_len < 4) {
        modbus_rx_len = 0;
        return;
    }

    /* Check slave address */
    addr = modbus_rx_buf[0];
    if (addr != MODBUS_SLAVE_ADDR && addr != 0x00) {
        /* Not for us (and not broadcast) — ignore silently */
        modbus_rx_len = 0;
        return;
    }

    /* CRC check */
    crc_received = ((uint16_t)modbus_rx_buf[modbus_rx_len - 1] << 8) |
                    modbus_rx_buf[modbus_rx_len - 2];
    crc_calculated = modbus_crc16(modbus_rx_buf, modbus_rx_len - 2);

    if (crc_received != crc_calculated) {
        modbus_rx_len = 0;
        return;
    }

    /* Dispatch by function code */
    fc = modbus_rx_buf[1];
    switch (fc) {
        case 0x03:
            modbus_fc03(modbus_rx_buf, modbus_rx_len);
            break;
        case 0x06:
            modbus_fc06(modbus_rx_buf, modbus_rx_len);
            break;
        case 0x10:
            modbus_fc16(modbus_rx_buf, modbus_rx_len);
            break;
        default:
            if (addr != 0x00) {  /* No response to broadcast errors */
                modbus_send_exception(fc, MODBUS_EX_ILLEGAL_FUNCTION);
            }
            break;
    }

    modbus_rx_len = 0;
}
