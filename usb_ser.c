/**
 * Copyright 2020 DriveX.Tech. All rights reserved.
 * 
 * Licensed under the License.
 */

#include "usb_ser.h"
#include "packet_buf.h"
#include "packet_queue.h"
#include "serial_num.h"
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_util.h"
#include "app_util_bds.h"
#include "app_util_platform.h"
#include "app_usbd_core.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_string_desc.h"
#include "nrf_strerror.h"
#include "nrf_atomic.h"
#include "nrf_atflags.h"
#include "nrf_delay.h"
#include "slip.h"
#include "crc16.h"
#include <assert.h>



typedef struct _usb_stat_t {
    nrf_atomic_flag_t busy;
    nrf_atomic_flag_t connected;
} usb_stat_t;

static volatile usb_stat_t g_usb_stat = { .busy = false, .connected = false };


static void app_usb_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_usb,
                            app_usb_user_ev_handler,
                            0,
                            1,
                            NRF_DRV_USBD_EPIN8,
                            NRF_DRV_USBD_EPIN4,
                            NRF_DRV_USBD_EPOUT4,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);



static BYTE g_usb_packet_recev_buf[MAX_PACKET_LEN] = {'8'};

static slip_t g_slipper;
static void reset_slipper(slip_t* slipper)
{
    slipper->state = SLIP_STATE_DECODING;
    slipper->current_index = 0;
    slipper->p_buffer = g_usb_packet_recev_buf;
    slipper->buffer_len = sizeof(g_usb_packet_recev_buf);
}

static void process_data_received(const BYTE* data, WORD data_len)
{
    ret_code_t err_code;

    for(WORD i = 0; i < data_len; i++) {
        err_code = slip_decode_add_byte(&g_slipper, data[i]);
        switch(err_code) {
            case NRF_SUCCESS:
                do {
                    if(g_slipper.current_index <= 2) {
                        // SEND_LOG("(%s): Wrong received packet length, SHOULD > 2\r\n", __func__);
                        reset_slipper(&g_slipper);
                        continue;
                    }
                    WORD crc16_checksum = crc16_compute(g_slipper.p_buffer, g_slipper.current_index - 2, NULL);
                    WORD packet_crc16_checksum = g_slipper.p_buffer[g_slipper.current_index - 2] + g_slipper.p_buffer[g_slipper.current_index - 1] * (1 << 8);
                    if(crc16_checksum != packet_crc16_checksum) {
                        // SEND_LOG("(%s): Corrupted received packet, checksum mismath!\r\n", __func__);
                    } else {
                        on_usb_data_received(g_slipper.p_buffer, g_slipper.current_index - 2);
                    }
                    reset_slipper(&g_slipper);
                } while(0);
                break;
            case NRF_ERROR_BUSY:
                break;
            case NRF_ERROR_NO_MEM:
            case NRF_ERROR_INVALID_DATA:
                do {
                    // SEND_LOG("(%s): Error(0x%x), reason: %s\r\n", __func__, err_code, nrf_strerror_get(err_code));
                    reset_slipper(&g_slipper);
                } while(0);
                break;
            default:
                do {
                    // SEND_LOG("(%s): Unknown Error(0x%x), reason: %s\r\n", __func__, err_code, nrf_strerror_get(err_code));
                    reset_slipper(&g_slipper);
                } while(0);
        }
    }
}


static BYTE g_usb_data_recev_buf[NRF_DRV_USBD_EPSIZE * 8] = {'8'};

static void app_usb_user_ev_handler(const app_usbd_class_inst_t* p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    ret_code_t ret;
    int len = 0;

    switch(event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            do {
                // g_usb_stat.connected = true;
                nrf_atomic_flag_set(&g_usb_stat.connected);

                /*Setup first transfer*/
                ret = NRF_SUCCESS;
                while(ret == NRF_SUCCESS) {
                    ret = app_usbd_cdc_acm_read_any(&m_app_usb,
                                                    g_usb_data_recev_buf,
                                                    sizeof(g_usb_data_recev_buf));
                    switch(ret) {
                        case NRF_SUCCESS:
                            len = app_usbd_cdc_acm_rx_size(&m_app_usb);
                            process_data_received(g_usb_data_recev_buf, len);
                            break;
                        case NRF_ERROR_IO_PENDING:
                            break;
                        case NRF_ERROR_BUSY:
                            break;
                        default:
                            break;
                    }
                }
            } while(0);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            do {
                // g_usb_stat.connected = false;
                nrf_atomic_flag_clear(&g_usb_stat.connected);
            } while(0);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            do {
                // g_usb_stat.busy = false;
                nrf_atomic_flag_clear(&g_usb_stat.busy);
                on_usb_data_sent();
            } while(0);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            do {
                /*Get amount of data transfered*/
                len = app_usbd_cdc_acm_rx_size(&m_app_usb);

                process_data_received(g_usb_data_recev_buf, len);

                /*Setup next transfer*/
                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read_any(&m_app_usb,
                                                g_usb_data_recev_buf,
                                                sizeof(g_usb_data_recev_buf));
            } while(ret == NRF_SUCCESS);
            break;
        default:
            break;
    }
}


static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    // CAN NOT use SEND_LOG() in this function, Log Not Ready!
    switch(event) {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;
        case APP_USBD_EVT_DRV_RESUME:
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            if(!nrf_drv_usbd_is_enabled()) {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            // g_usb_stat.connected = false;
            nrf_atomic_flag_clear(&g_usb_stat.connected);
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;
        case APP_USBD_EVT_DRV_RESET:
            // g_usb_stat.connected = false;
            nrf_atomic_flag_clear(&g_usb_stat.connected);
            break;
        default:
            break;
    }
}


void usb_init()
{
    ret_code_t ret;

    app_usbd_config_t app_usbd_config;
    memset(&app_usbd_config, 0, sizeof(app_usbd_config));
    app_usbd_config.ev_state_proc = usbd_user_ev_handler;

    ret = app_usbd_init(&app_usbd_config);
    APP_ERROR_CHECK(ret);

    serial_num_generate();

    const app_usbd_class_inst_t* app_usb_inst = app_usbd_cdc_acm_class_inst_get(&m_app_usb);
    ret = app_usbd_class_append(app_usb_inst);
    APP_ERROR_CHECK(ret);

    if(USBD_POWER_DETECTION) {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    } else {
        app_usbd_enable();
        app_usbd_start();
    }

    reset_slipper(&g_slipper);

    /* Give some time for the host to enumerate and connect to the USB port */
    nrf_delay_ms(800);
}


bool send_usb_data(const BYTE* data, WORD data_len)
{
    static BYTE send_buf[MAX_PACKET_LEN];
    static BYTE checksum16[2];

    ret_code_t err_code;
    uint32_t payload_len = 0, checksum_len = 0;

    err_code = slip_encode((BYTE*)send_buf, (BYTE*)data, data_len, &payload_len);
    assert(err_code == NRF_SUCCESS);

    WORD crc16_checksum = crc16_compute(data, data_len, NULL);
    checksum16[0] = (BYTE)crc16_checksum;
    checksum16[1] = (BYTE)(crc16_checksum >> 8);

    err_code = slip_encode((BYTE*)send_buf + payload_len - 1, checksum16, sizeof(checksum16), &checksum_len);
    assert(err_code == NRF_SUCCESS);

    // g_usb_stat.busy = true;
    nrf_atomic_flag_set(&g_usb_stat.busy);

    //err_code = app_usbd_cdc_acm_write(&m_app_usb, send_buf, payload_len);
    err_code = app_usbd_cdc_acm_write(&m_app_usb, send_buf, payload_len + checksum_len - 1);
    //assert(err_code == NRF_SUCCESS);

    if(err_code == NRF_SUCCESS) {
        return true;
    } else {
        //TODO: log warning
        // g_usb_stat.busy = false;
        nrf_atomic_flag_clear(&g_usb_stat.busy);
        return false;
    }
}


bool check_usb_connected()
{
    return g_usb_stat.connected;
}

bool check_usb_busy()
{
    return g_usb_stat.busy;
}