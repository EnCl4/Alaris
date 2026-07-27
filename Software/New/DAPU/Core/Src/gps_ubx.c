/**
  ******************************************************************************
  * @file    gps_ubx.c
  * @brief   u-blox NEO-M8N on USART2, UBX binary protocol, NAV-PVT at 10 Hz.
  ******************************************************************************
  * The receiver leaves the factory at 9600 baud, where a 100 byte NAV-PVT at
  * 10 Hz needs 10 kbit/s and simply does not fit. gps_init() therefore raises
  * the link to 115200 first (trying both bauds so it works whatever state the
  * module was left in), then applies the message/rate configuration.
  ******************************************************************************
  */

#include "main.h"
#include "usart.h"
#include "dapu_config.h"
#include "dapu_state.h"
#include "gps_ubx.h"
#include <string.h>

/* ==========================================================================
 * DMA receive ring
 * ========================================================================== */

static uint8_t  s_rx_buf[GPS_RX_DMA_SIZE];
static uint16_t s_rx_tail;

/* ==========================================================================
 * UBX framing
 * ========================================================================== */

#define UBX_SYNC1           0xB5
#define UBX_SYNC2           0x62
#define UBX_CLASS_NAV       0x01
#define UBX_ID_NAV_PVT      0x07
#define UBX_NAV_PVT_LEN     92u
#define UBX_MAX_PAYLOAD     128u

typedef enum
{
    ST_SYNC1 = 0, ST_SYNC2, ST_CLASS, ST_ID, ST_LEN1, ST_LEN2,
    ST_PAYLOAD, ST_CKA, ST_CKB
} ubx_state_t;

static ubx_state_t s_state;
static uint8_t     s_class, s_id;
static uint16_t    s_len, s_count;
static uint8_t     s_payload[UBX_MAX_PAYLOAD];
static uint8_t     s_ck_a, s_ck_b;

static uint32_t    s_frames;
static uint32_t    s_ck_errors;

static inline int32_t  le32 (const uint8_t *p) { return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); }
static inline uint32_t leu32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint16_t leu16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

/* ==========================================================================
 * Transmit helpers - the checksum is always computed, never hand written
 * ========================================================================== */

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t head[6] = { UBX_SYNC1, UBX_SYNC2, cls, id,
                        (uint8_t)(len & 0xFFu), (uint8_t)(len >> 8) };
    uint8_t ck[2] = { 0, 0 };

    for (uint16_t i = 2; i < 6; i++)
    {
        ck[0] = (uint8_t)(ck[0] + head[i]);
        ck[1] = (uint8_t)(ck[1] + ck[0]);
    }
    for (uint16_t i = 0; i < len; i++)
    {
        ck[0] = (uint8_t)(ck[0] + payload[i]);
        ck[1] = (uint8_t)(ck[1] + ck[0]);
    }

    HAL_UART_Transmit(&huart2, head, sizeof(head), 100);
    if (len > 0u)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)payload, len, 200);
    }
    HAL_UART_Transmit(&huart2, ck, sizeof(ck), 100);
    HAL_Delay(5);
}

/** Pre-built frame copied verbatim from the validated F4 bench firmware. */
static void ubx_send_raw(const uint8_t *frame, uint16_t len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)frame, len, 200);
    HAL_Delay(5);
}

static void gps_set_baud(uint32_t baud)
{
    HAL_UART_DeInit(&huart2);
    huart2.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(20);
}

/* ==========================================================================
 * Configuration messages
 * ========================================================================== */

/* CFG-PRT, UART1, 8N1, in = UBX+NMEA+RTCM, out = UBX only, baud 115200. */
static const uint8_t cfg_prt_115200[20] = {
    0x01, 0x00, 0x00, 0x00,             /* portID, reserved, txReady        */
    0xD0, 0x08, 0x00, 0x00,             /* mode: 8 bit, no parity, 1 stop   */
    0x00, 0xC2, 0x01, 0x00,             /* baudRate = 115200                */
    0x07, 0x00,                         /* inProtoMask  = UBX|NMEA|RTCM     */
    0x01, 0x00,                         /* outProtoMask = UBX               */
    0x00, 0x00, 0x00, 0x00              /* flags, reserved                  */
};

/* CFG-RATE: measRate 100 ms (10 Hz), navRate 1, timeRef = GPS. */
static const uint8_t cfg_rate_10hz[6] = { 0x64, 0x00, 0x01, 0x00, 0x01, 0x00 };

/* CFG-NAV5: only the dynamic model byte is meaningful here, mask = 0x0001. */
static const uint8_t cfg_nav5_dynmodel[36] = {
    0x01, 0x00,                         /* mask: apply dynModel only        */
    GPS_DYN_MODEL, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* CFG-CFG: save the current configuration to BBR + flash. */
static const uint8_t cfg_save[13] = {
    0x00, 0x00, 0x00, 0x00,             /* clearMask                        */
    0xFF, 0xFF, 0x00, 0x00,             /* saveMask                         */
    0x00, 0x00, 0x00, 0x00,             /* loadMask                         */
    0x17                                /* deviceMask: BBR|flash|EEPROM     */
};

/* CFG-GNSS, GPS + SBAS + Galileo + GLONASS. Verbatim from the F4 firmware. */
static const uint8_t cfg_gnss_frame[] = {
    0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08,
    0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00,
    0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x03, 0x04,
    0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x01, 0x00,
    0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x06, 0x08,
    0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x2D, 0x45
};

/** CFG-MSG, 3 byte form: enable/disable one message on the current port. */
static void ubx_set_message_rate(uint8_t cls, uint8_t id, uint8_t rate)
{
    uint8_t payload[3] = { cls, id, rate };
    ubx_send(0x06, 0x01, payload, sizeof(payload));
}

static void gps_apply_config(void)
{
    /* Silence every standard NMEA sentence - at 10 Hz they would eat the
     * whole link and starve NAV-PVT. */
    for (uint8_t nmea_id = 0x00; nmea_id <= 0x05; nmea_id++)
    {
        ubx_set_message_rate(0xF0, nmea_id, 0);
    }

    ubx_set_message_rate(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1);

    ubx_send(0x06, 0x08, cfg_rate_10hz,      sizeof(cfg_rate_10hz));
    ubx_send(0x06, 0x24, cfg_nav5_dynmodel,  sizeof(cfg_nav5_dynmodel));
    ubx_send_raw(cfg_gnss_frame,             sizeof(cfg_gnss_frame));
    ubx_send(0x06, 0x09, cfg_save,           sizeof(cfg_save));
}

/* ==========================================================================
 * NAV-PVT decoding
 * ========================================================================== */

static void handle_nav_pvt(const uint8_t *p, uint16_t len)
{
    if (len < UBX_NAV_PVT_LEN)
    {
        return;
    }

    uint8_t fix_type = p[20];

    dapu_state_lock();

    g_state.gps_year        = leu16(&p[4]);
    g_state.gps_month       = p[6];
    g_state.gps_day         = p[7];
    g_state.gps_hour        = p[8];
    g_state.gps_min         = p[9];
    g_state.gps_sec         = p[10];
    g_state.gps_fix_type    = fix_type;
    g_state.gps_num_sv      = p[23];
    g_state.gps_lon_1e7     = le32(&p[24]);
    g_state.gps_lat_1e7     = le32(&p[28]);
    g_state.gps_height_mm   = le32(&p[32]);
    g_state.gps_hmsl_mm     = le32(&p[36]);
    g_state.gps_hacc_mm     = leu32(&p[40]);
    g_state.gps_vacc_mm     = leu32(&p[44]);
    g_state.gps_vel_n_mms   = le32(&p[48]);
    g_state.gps_vel_e_mms   = le32(&p[52]);
    g_state.gps_vel_d_mms   = le32(&p[56]);
    g_state.gps_gspeed_mms  = le32(&p[60]);
    g_state.gps_headmot_1e5 = le32(&p[64]);
    g_state.gps_seq++;
    g_state.gps_last_tick   = HAL_GetTick();

    if (fix_type >= 3u)
    {
        g_state.status |= DAPU_ST_GPS_FIX_3D;
    }
    else
    {
        g_state.status &= (uint16_t)~DAPU_ST_GPS_FIX_3D;
    }

    dapu_state_unlock();

    s_frames++;
}

static inline void ck_accumulate(uint8_t b)
{
    s_ck_a = (uint8_t)(s_ck_a + b);
    s_ck_b = (uint8_t)(s_ck_b + s_ck_a);
}

static void ubx_parse_byte(uint8_t b)
{
    switch (s_state)
    {
    case ST_SYNC1:
        if (b == UBX_SYNC1) { s_state = ST_SYNC2; }
        break;

    case ST_SYNC2:
        s_state = (b == UBX_SYNC2) ? ST_CLASS : ST_SYNC1;
        break;

    case ST_CLASS:
        s_class = b;
        s_ck_a = 0; s_ck_b = 0;
        ck_accumulate(b);
        s_state = ST_ID;
        break;

    case ST_ID:
        s_id = b;
        ck_accumulate(b);
        s_state = ST_LEN1;
        break;

    case ST_LEN1:
        s_len = b;
        ck_accumulate(b);
        s_state = ST_LEN2;
        break;

    case ST_LEN2:
        s_len |= (uint16_t)((uint16_t)b << 8);
        ck_accumulate(b);
        if (s_len > UBX_MAX_PAYLOAD)
        {
            s_state = ST_SYNC1;             /* not a frame we care about */
        }
        else if (s_len == 0u)
        {
            s_state = ST_CKA;
        }
        else
        {
            s_count = 0;
            s_state = ST_PAYLOAD;
        }
        break;

    case ST_PAYLOAD:
        s_payload[s_count++] = b;
        ck_accumulate(b);
        if (s_count >= s_len)
        {
            s_state = ST_CKA;
        }
        break;

    case ST_CKA:
        if (b == s_ck_a)
        {
            s_state = ST_CKB;
        }
        else
        {
            s_ck_errors++;
            s_state = ST_SYNC1;
        }
        break;

    case ST_CKB:
        if (b == s_ck_b)
        {
            if (s_class == UBX_CLASS_NAV && s_id == UBX_ID_NAV_PVT)
            {
                handle_nav_pvt(s_payload, s_len);
            }
        }
        else
        {
            s_ck_errors++;
        }
        s_state = ST_SYNC1;
        break;

    default:
        s_state = ST_SYNC1;
        break;
    }
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

void gps_poll(void)
{
    uint16_t head = (uint16_t)(GPS_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx));

    while (s_rx_tail != head)
    {
        ubx_parse_byte(s_rx_buf[s_rx_tail]);
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % GPS_RX_DMA_SIZE);
    }
}

bool gps_init(void)
{
    s_state     = ST_SYNC1;
    s_rx_tail   = 0;
    s_frames    = 0;
    s_ck_errors = 0;

    /* The module may be at either baud depending on whether a previous run
     * already saved the configuration, so push CFG-PRT from both. */
    gps_set_baud(GPS_BAUD_DEFAULT);
    ubx_send(0x06, 0x00, cfg_prt_115200, sizeof(cfg_prt_115200));
    HAL_Delay(100);

    gps_set_baud(GPS_BAUD_TARGET);
    ubx_send(0x06, 0x00, cfg_prt_115200, sizeof(cfg_prt_115200));
    HAL_Delay(100);

    gps_apply_config();

    if (HAL_UART_Receive_DMA(&huart2, s_rx_buf, GPS_RX_DMA_SIZE) != HAL_OK)
    {
        return false;
    }

    /* Give the receiver up to 1 s to emit its first NAV-PVT. Absence of a fix
     * is fine here; absence of any frame means the link is not working. */
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 1000u)
    {
        gps_poll();
        if (s_frames > 0u)
        {
            return true;
        }
        HAL_Delay(10);
    }
    return false;
}

uint32_t gps_frame_count(void)      { return s_frames;    }
uint32_t gps_checksum_errors(void)  { return s_ck_errors; }
