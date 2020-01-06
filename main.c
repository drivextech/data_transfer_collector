
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "slip.h"
#include "crc16.h"
#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_usbd_core.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_string_desc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_db_discovery.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "ble_nus_c.h"
#include "boards.h"
#include "bsp_btn_ble.h"
#include "protothread/pt.h"
#include "serial_num.h"
#include "tlv_transformer.h"
#include "packet_buf.h"
#include "util.h"



/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


#define APP_BLE_CONN_CFG_TAG      1                                     /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< BLE observer priority of the application. There is no need to modify this value. */

#define SCAN_INTERVAL 840 /**< Determines scan interval in units of 0.625 milliseconds. */
//#define SCAN_WINDOW   48 /**< Determines scan window in units of 0.625 milliseconds. */
//#define SCAN_WINDOW   38 /**< Determines scan window in units of 0.625 milliseconds. */
#define SCAN_WINDOW   32 /**< Determines scan window in units of 0.625 milliseconds. */
#define SCAN_TIMEOUT  0x0    /**< Scan timeout between 0x01 and 0xFFFF in seconds, 0x0 disables timeout. */

#define SCHED_CONN_INTERVAL 28

#define MIN_CONN_INTERVAL               8                                       /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               80                                      /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
// #define MIN_CONN_INTERVAL               18                                      /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
// #define MAX_CONN_INTERVAL               18                                      /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
// #define MIN_CONN_INTERVAL               28                                      /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
// #define MAX_CONN_INTERVAL               28                                      /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
// #define SLAVE_LATENCY                   0                                       /**< Slave latency. */
// #define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define PEER_NUM_LED_MASK         BSP_LED_0_MASK
#define HOST_USB_LED_MASK         BSP_LED_1_MASK

#define USER_BUTTON_NO            BSP_BUTTON_0                          /**< Button for user */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(80)                   /**< Delay for a button is reported as pushed (in number of timer ticks). */


// Log / Data / CMD channel
static packet_buf_t g_log_packet_buf, g_data_packet_buf, g_cmd_packet_buf;
static BYTE packet_buf[MAX_PACKET_LEN];

static BYTE tlv_header_buf[TLV_HEADER_LEN];
//#define SEND_RAWBYTE(...)
#define SEND_RAWBYTE(PACKET_BUF_NAME, DATA_TAG, DATA, DATA_LEN) do {                \
    int header_len = tlv_header_generate(DATA_TAG, DATA_LEN, tlv_header_buf);       \
    assert(header_len >= 0);                                                        \
    int total_len = header_len + DATA_LEN;                                          \
    assert(total_len > 0);                                                          \
    WORD seg_len;                                                                   \
    seg_len = packet_buf_prepare_data(&PACKET_BUF_NAME, total_len);                 \
    assert(seg_len == total_len);                                                   \
    seg_len = packet_buf_put_data(&PACKET_BUF_NAME, tlv_header_buf, header_len);    \
    assert(seg_len == header_len);                                                  \
    seg_len = packet_buf_put_data(&PACKET_BUF_NAME, DATA, DATA_LEN);                \
    assert(seg_len == DATA_LEN);                                                    \
    seg_len = packet_buf_commit_data(&PACKET_BUF_NAME);                             \
    assert(seg_len == total_len);                                                   \
} while(0)


//#define SEND_LOG(...)
#define SEND_LOG(...) do {                                                          \
    int packet_len = sprintf(packet_buf, __VA_ARGS__);                              \
    assert(packet_len >= 0);                                                        \
    SEND_RAWBYTE(g_log_packet_buf, TAG_LOG, packet_buf, packet_len);                \
} while(0)
#define POP_LOG(DATA, DATA_LEN) packet_buf_pop_data(&g_log_packet_buf, DATA, DATA_LEN)
#define PEEK_LOG(DATA, DATA_LEN) packet_buf_peek_data(&g_log_packet_buf, DATA, DATA_LEN)

//#define SEND_USB_DATA(DATA, DATA_LEN)
#define SEND_USB_DATA(DATA, DATA_LEN) SEND_RAWBYTE(g_data_packet_buf, TAG_DATA, DATA, DATA_LEN)
#define POP_USB_DATA(DATA, DATA_LEN) packet_buf_pop_data(&g_data_packet_buf, DATA, DATA_LEN)
#define PEEK_USB_DATA(DATA, DATA_LEN) packet_buf_peek_data(&g_data_packet_buf, DATA, DATA_LEN)


static const char m_target_periph_name[] = "NordicDTS";             /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */


#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

static const ble_uuid_t m_nus_uuid = {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE};



static char g_usb_data_recev_buf[NRF_DRV_USBD_EPSIZE * 16] = {'8'};



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

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_NUS_C_ARRAY_DEF(m_dts, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */



typedef struct _bt_stat_t {
    int peer_num;
} bt_stat_t;

typedef struct _dt_stat_t {
    volatile bool ready_to_send;
    volatile bool connected;
} dt_stat_t;


static volatile bt_stat_t g_bt_stat;

static ble_gap_addr_t g_handle_peeraddr_table[NRF_SDH_BLE_CENTRAL_LINK_COUNT];

void on_ble_data_received(const ble_gap_addr_t* peer_addr, const uint8_t* data, int data_len)
{
    assert(data_len > 0);

    static BYTE recev_buf[MAX_PACKET_LEN];

//    SEND_LOG("(%s): ble data received: %lu B\r\n", __func__, data_len);

    fast_copy(recev_buf, peer_addr->addr, 6);
//    fast_copy(recev_buf + 6, data, data_len);
    fast_copy(recev_buf + 6, data + 5, data_len - 5);

//    SEND_USB_DATA(recev_buf, 6 + data_len);
    SEND_USB_DATA(recev_buf, 1 + data_len);
}

void on_ble_data_sent(const ble_gap_addr_t* peer_addr)
{
}


static volatile dt_stat_t g_usb_stat = { .ready_to_send = true, .connected = false };

void on_usb_data_received(const uint8_t* data, int data_len)
{
//    SEND_LOG("(%s): usb data received: %lu B\r\n", __func__, data_len);
    SEND_LOG("(%s)(buf: %u, %u): usb data received: %lu B\r\n", __func__,
            QUEUE_AVAILABLE(*g_log_packet_buf.packet_queue), QUEUE_AVAILABLE(*g_data_packet_buf.packet_queue), data_len);
}

void on_usb_data_sent()
{
    g_usb_stat.ready_to_send = true;
}



static void app_usb_user_ev_handler(const app_usbd_class_inst_t* p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    ret_code_t ret;
    int len = 0;

    switch(event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            do {
                g_usb_stat.connected = true;

                SEND_LOG("(%s): USB connected!\r\n", __func__);
                /*Setup first transfer*/
                ret = NRF_SUCCESS;
                while(ret == NRF_SUCCESS) {
                    ret = app_usbd_cdc_acm_read_any(&m_app_usb,
                                                    g_usb_data_recev_buf,
                                                    sizeof(g_usb_data_recev_buf));
                    switch(ret) {
                        case NRF_SUCCESS:
                            len = app_usbd_cdc_acm_rx_size(&m_app_usb);
                            on_usb_data_received(g_usb_data_recev_buf, len);
                            break;
                        case NRF_ERROR_IO_PENDING:
                            break;
                        case NRF_ERROR_BUSY:
                            SEND_LOG("(%s): READ Busy Error!\r\n", __func__);
                            break;
                        default:
                            SEND_LOG("(%s): Unknown Error!\r\n", __func__);
                            break;
                    }
                }
            } while(0);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            do {
                g_usb_stat.connected = false;
                SEND_LOG("(%s): USB closed!\r\n", __func__);
            } while(0);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            do {
                on_usb_data_sent();
            } while(0);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            do {
                /*Get amount of data transfered*/
                len = app_usbd_cdc_acm_rx_size(&m_app_usb);

                on_usb_data_received(g_usb_data_recev_buf, len);

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
            g_usb_stat.connected = false;
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;
        case APP_USBD_EVT_DRV_RESET:
            g_usb_stat.connected = false;
            break;
        default:
            break;
    }
}



/**@brief Function for starting scanning. */
static void scan_start(void)
{
    ret_code_t err_code;

    SEND_LOG("(%s): Start scanning for device with name: %s\r\n", __func__, m_target_periph_name);
    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}

static void scan_evt_handler(const scan_evt_t* p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id) {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            do {
                //TODO
                err_code = p_scan_evt->params.connecting_err.err_code;
                SEND_LOG("(%s): Scan connecting error: %u\r\n", __func__, err_code);
                APP_ERROR_CHECK(err_code);
            } while(0);
            break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
             do {
                  ble_gap_evt_connected_t const * p_connected =
                                   p_scan_evt->params.connected.p_connected;
                 // Scan is automatically stopped by the connection.
                 SEND_LOG("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                          p_connected->peer_addr.addr[0],
                          p_connected->peer_addr.addr[1],
                          p_connected->peer_addr.addr[2],
                          p_connected->peer_addr.addr[3],
                          p_connected->peer_addr.addr[4],
                          p_connected->peer_addr.addr[5]
                          );
             } while(0);
             break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
             do {
                 SEND_LOG("Scan timed out.\r\n");
                 scan_start();
             } while(0);
             break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            do {
            } while(0);
            break;

        case NRF_BLE_SCAN_EVT_SCAN_REQ_REPORT:
            do {
            } while(0);
            break;

         default:
             break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    ble_gap_conn_params_t conn_params;
    ble_gap_scan_params_t scan_params;
    nrf_ble_scan_init_t init_scan;

    memset(&conn_params, 0, sizeof(conn_params));
//    conn_params.min_conn_interval = MIN_CONN_INTERVAL;
//    conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    conn_params.min_conn_interval = SCHED_CONN_INTERVAL;
    conn_params.max_conn_interval = SCHED_CONN_INTERVAL;
    conn_params.slave_latency     = SLAVE_LATENCY;
    conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    memset(&scan_params, 0, sizeof(scan_params));
    scan_params.extended = 0;
    scan_params.report_incomplete_evts = 0;
    scan_params.active = 0;
    scan_params.filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL;
    scan_params.scan_phys = BLE_GAP_PHY_1MBPS;
    scan_params.interval = SCAN_INTERVAL;
    scan_params.window = SCAN_WINDOW;
    scan_params.timeout = SCAN_TIMEOUT;
    scan_params.channel_mask[0] = 0;
    scan_params.channel_mask[1] = 0;
    scan_params.channel_mask[2] = 0;
    scan_params.channel_mask[3] = 0;
    scan_params.channel_mask[4] = 0;

    memset(&init_scan, 0, sizeof(init_scan));
    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    init_scan.p_conn_param     = &conn_params;
    init_scan.p_scan_param     = &scan_params;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
//    APP_ERROR_CHECK(err_code);
//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
//    APP_ERROR_CHECK(err_code);
}


static void ble_nus_c_evt_handler(ble_nus_c_t* p_ble_nus_c, const ble_nus_c_evt_t* p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            SEND_LOG("DTS discovered on conn_handle 0x%x\r\n", p_ble_nus_evt->conn_handle);

            g_bt_stat.peer_num++;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            do {
                // SEND_LOG("(%s): conn(0x%x) Received data: %d B\r\n", __func__, p_ble_nus_c->conn_handle, p_ble_nus_evt->data_len);

                // assert `g_handle_peeraddr_table[p_ble_nus_c->conn_handle]` Not Invalid!
                on_ble_data_received(&g_handle_peeraddr_table[p_ble_nus_c->conn_handle], p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            } while(0);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            SEND_LOG("Disconnected.\r\n");

            g_bt_stat.peer_num--;

            break;
    }
}


const char* get_phy_name(BYTE phy_id)
{
    char* phy_name = "NotSet";
    switch(phy_id) {
        case BLE_GAP_PHY_1MBPS:
            phy_name = "Phy1Mb";
            break;
        case BLE_GAP_PHY_2MBPS:
            phy_name = "Phy2Mb";
            break;
        case BLE_GAP_PHY_CODED:
            phy_name = "PhyCoded";
            break;
        case BLE_GAP_PHY_AUTO:
            phy_name = "PhyAuto";
            break;
        case BLE_GAP_PHY_NOT_SET:
        default:
            break;
    }
    return phy_name;
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(const ble_evt_t* p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    uint16_t conn_handle = 0;

    // For readability.
    const ble_gap_evt_t* p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch(p_ble_evt->header.evt_id) {
        // Upon connection, check which peripheral is connected, initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
            do {
                conn_handle = p_gap_evt->conn_handle;

                SEND_LOG("Connection 0x%x established, starting DB discovery.\r\n", conn_handle);

                APP_ERROR_CHECK_BOOL(conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

                ble_gap_phys_t phys = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);

                err_code = ble_nus_c_handles_assign(&m_dts[conn_handle],
                                                    conn_handle,
                                                    NULL);
                APP_ERROR_CHECK(err_code);

                err_code = ble_db_discovery_start(&m_db_disc[conn_handle],
                                                  conn_handle);
                if(err_code != NRF_ERROR_BUSY) {
                    APP_ERROR_CHECK(err_code);
                }

                if(ble_conn_state_central_conn_count() >= NRF_SDH_BLE_CENTRAL_LINK_COUNT) {
                } else {
                    SEND_LOG("(%s): Connected: continue scan!\r\n", __func__);
                    scan_start();
                }

                g_handle_peeraddr_table[conn_handle] = p_gap_evt->params.connected.peer_addr;
            } while(0);
            break;

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
            do {
                conn_handle = p_gap_evt->conn_handle;

                SEND_LOG("Disconnected. conn_handle: 0x%x, reason: 0x%x\r\n",
                             conn_handle,
                             p_gap_evt->params.disconnected.reason);

                memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));

                // Start scanning.
//                scan_start();

            } while(0);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            do {
                ble_gap_phys_t phy = p_ble_evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys;
                const char* phy_s = get_phy_name(phy.tx_phys);
                const char* phy_r = get_phy_name(phy.rx_phys);
                SEND_LOG("(%s): PHY update request params: (%s, %s)\r\n", __func__, phy_s, phy_r);
                ble_gap_phys_t phys = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
            } while(0);
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
            do {
                ble_gap_evt_phy_update_t phy = p_ble_evt->evt.gap_evt.params.phy_update;
                const char* phy_s = get_phy_name(phy.tx_phy);
                const char* phy_r = get_phy_name(phy.rx_phy);
                SEND_LOG("(%s): PHY update params: (%s, %s)\r\n", __func__, phy_s, phy_r);
            } while(0);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            do {
                // Timeout for scanning is not specified, so only the connection requests can time out.
                if(p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
                    SEND_LOG("Connection request timed out.\r\n");

                    memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));
                }
            } while(0);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            do {
                ble_gap_conn_params_t params = p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params;
                uint16_t max_con_int = params.max_conn_interval;
                uint16_t min_con_int = params.min_conn_interval;
                SEND_LOG("(%s): Conn params update request: CI: %i, %i\r\n", __func__, min_con_int, max_con_int);
                int min_sched = min_con_int / SCHED_CONN_INTERVAL;
                int max_sched = max_con_int / SCHED_CONN_INTERVAL;
                int min_diff = min_con_int % SCHED_CONN_INTERVAL;
                if(min_diff == 0) {
                    params.min_conn_interval = params.max_conn_interval = min_con_int;
                } else {
                    uint16_t minp1_con_int = (min_sched + 1) * SCHED_CONN_INTERVAL;
                    if(minp1_con_int <= max_con_int) {
                        params.min_conn_interval = params.max_conn_interval = minp1_con_int;
                    } else {
                        int max_diff = max_con_int - max_sched * SCHED_CONN_INTERVAL;
                        if(min_diff <= max_diff) {
                            params.min_conn_interval = params.max_conn_interval = min_sched * SCHED_CONN_INTERVAL;
                        } else {
                            params.min_conn_interval = params.max_conn_interval = max_sched * SCHED_CONN_INTERVAL;
                        }
                    }
                }
                err_code = sd_ble_gap_conn_param_update(p_ble_evt->evt.gap_evt.conn_handle, &params);
                APP_ERROR_CHECK(err_code);
            } while(0);
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            do {
                ble_gap_conn_params_t params = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params;
                uint16_t max_con_int = params.max_conn_interval;
                uint16_t min_con_int = params.min_conn_interval;
                SEND_LOG("(%s): Conn params update: CI: (%i, %i)\r\n",
                            __func__,
                            p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval,
                            p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval);
            } while(0);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            do {
                // Disconnect on GATT client timeout event.
                SEND_LOG("GATT client timeout.\r\n");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);

                memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));
            } while(0);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            do {
                // Disconnect on GATT server timeout event.
                SEND_LOG("GATT server timeout.\r\n");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);

                memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));
            } while(0);
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
            do {
                ble_gap_data_length_params_t params = p_ble_evt->evt.gap_evt.params.data_length_update.effective_params;
                SEND_LOG("(%s): Data length update: (%hu, %hu) B, (%hu, %hu) us\r\n", __func__,
                                params.max_tx_octets,
                                params.max_rx_octets,
                                params.max_tx_time_us,
                                params.max_rx_time_us);
            } while(0);
            break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
            do {
                ble_gap_data_length_params_t params = p_ble_evt->evt.gap_evt.params.data_length_update_request.peer_params;
                SEND_LOG("(%s): Data length update request: (%hu, %hu) B, (%hu, %hu) us", __func__,
                                params.max_tx_octets,
                                params.max_rx_octets,
                                params.max_tx_time_us,
                                params.max_rx_time_us);
                ble_gap_data_length_params_t dle_param;
                memset(&dle_param, 0, sizeof(dle_param));
                // dle_param.max_tx_octets = dle_param.max_rx_octets = NRF_SDH_BLE_GAP_DATA_LENGTH;
                // err_code = sd_ble_gap_data_length_update(p_dts->conn_handle, &dle_param, NULL);
                // APP_ERROR_CHECK(err_code);
            } while(0);
            break;

        default:
            // No implementation needed.
            break;
    }
}


static void dts_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_nus_c_init(&m_dts[i], &init);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press or release).
 */
static void button_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    if(pin_no == USER_BUTTON_NO) {
    };
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    static app_button_cfg_t button_conf;
    button_conf.pin_no = USER_BUTTON_NO;
    button_conf.pull_cfg = BUTTON_PULL;
    button_conf.active_state = false;
    button_conf.button_handler = button_handler;

    err_code = app_button_init(&button_conf, 1, BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t* p_evt)
{
    SEND_LOG("call to ble_lbs_on_db_disc_evt for conn_handle 0x%x!\r\n",
                  p_evt->conn_handle);

    ble_nus_c_on_db_disc_evt(&m_dts[p_evt->conn_handle], p_evt);
}

/** @brief Database discovery initialization.
 */
static void db_discovery_init()
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t* p_evt)
{
    uint32_t data_length;
//    if(m_dts.conn_handle != p_evt->conn_handle) {
//        return;
//    }
    switch (p_evt->evt_id) {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
            data_length = p_evt->params.att_mtu_effective;
            SEND_LOG("(%s): ATT MTU is set to 0x%X (%d)\r\n", __func__, data_length, data_length);
            break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
            data_length = p_evt->params.att_mtu_effective - 4;
            SEND_LOG("(%s): Data len is set to 0x%X (%d)\r\n", __func__, p_evt->params.data_length, p_evt->params.data_length);
            break;
    
        default:
            SEND_LOG("(%s): Unknown GATT event, FATAL ERROR!\r\n", __func__);
            break;
    }
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    // connection event extenstion
    ble_opt_t  opt;
    memset(&opt, 0, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = 1;

    // This API must be called after sd_ble_enable().
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}


static void usb_init()
{
    ret_code_t ret;

    app_usbd_config_t usbd_config;
    memset(&usbd_config, 0, sizeof(usbd_config));
    usbd_config.ev_state_proc = usbd_user_ev_handler;

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    serial_num_generate();

    const app_usbd_class_inst_t* usb_inst = app_usbd_cdc_acm_class_inst_get(&m_app_usb);
    ret = app_usbd_class_append(usb_inst);
    APP_ERROR_CHECK(ret);

    if(USBD_POWER_DETECTION) {
        ret_code_t ret;
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    } else {
        app_usbd_enable();
        app_usbd_start();
    }

    /* Give some time for the host to enumerate and connect to the USB port */
    nrf_delay_ms(800);

    while(! g_usb_stat.connected) {
#ifdef APP_USBD_CONFIG_EVENT_QUEUE_ENABLE
        app_usbd_event_queue_process();
#endif
    }
}


static void power_init()
{
    ret_code_t err_code;

    nrf_drv_power_config_t power_conf;
    err_code = nrf_drv_power_init(NULL); // Must be called before ble init
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


APP_TIMER_DEF(m_app_timer_id);

void timeout_cb(void* p_context)
{
    static int cached_peer_num = -8;

    if(g_usb_stat.connected) {
//    if(g_usb_stat.ready_to_send) {
        LEDS_ON(HOST_USB_LED_MASK);
    } else {
        LEDS_OFF(HOST_USB_LED_MASK);
    }

    if(g_bt_stat.peer_num == cached_peer_num) {
        if(cached_peer_num != 0) {
            LEDS_INVERT(PEER_NUM_LED_MASK);
        }
        return;
    }

    if(g_bt_stat.peer_num == 0) {
        LEDS_ON(PEER_NUM_LED_MASK);
    } else {
        app_timer_stop(m_app_timer_id);
        if(g_bt_stat.peer_num > 0 && g_bt_stat.peer_num <= 8) {
            app_timer_start(m_app_timer_id, APP_TIMER_TICKS(1000 * g_bt_stat.peer_num), NULL);
        } else {
            app_timer_start(m_app_timer_id, APP_TIMER_TICKS(1000 / 8), NULL);
        }
    }

    cached_peer_num = g_bt_stat.peer_num;
}

/** @brief Function for initializing the timer.
 */
static void timer_init()
{
    ret_code_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timeout_cb);
    // err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_SINGLE_SHOT, timeout_cb);
    APP_ERROR_CHECK(err_code);
}

static void timer_start()
{
    SEND_LOG("in (%s)\r\n", __func__);
    ret_code_t err_code;
    //err_code = app_timer_start(m_app_timer_id, APP_TIMER_TICKS(100), NULL);
    err_code = app_timer_start(m_app_timer_id, APP_TIMER_TICKS(1000), NULL);
    //err_code = app_timer_start(m_app_timer_id, APP_TIMER_TICKS(10000), NULL);
    APP_ERROR_CHECK(err_code);
}




bool g_idle = true;

PT_THREAD(task_app_usb(struct pt* pt))
{
    PT_BEGIN(pt);
#ifdef APP_USBD_CONFIG_EVENT_QUEUE_ENABLE
    while(true) {
        while(app_usbd_event_queue_process()) {
            g_idle = false;
        }
        PT_YIELD(pt);
    }
#endif
    PT_END(pt);
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

    //err_code = app_usbd_cdc_acm_write(&m_app_usb, send_buf, payload_len);
    err_code = app_usbd_cdc_acm_write(&m_app_usb, send_buf, payload_len + checksum_len - 1);
    //assert(err_code == NRF_SUCCESS);

    if(err_code == NRF_SUCCESS) {
        g_usb_stat.ready_to_send = false;
        return true;
    } else {
        //TODO: log warning
        return false;
    }
}
bool process_packet()
{
    static BYTE dt_buff[MAX_PACKET_LEN];
    static int dt_buff_len = 0;
    bool ret;

    dt_buff_len = POP_USB_DATA(dt_buff, MAX_PACKET_LEN);
    if(dt_buff_len > 0) {
        ret = send_usb_data(dt_buff, dt_buff_len);
        return true;
    }

    dt_buff_len = POP_LOG(dt_buff, MAX_PACKET_LEN);
    if(dt_buff_len > 0) {
        ret = send_usb_data(dt_buff, dt_buff_len);
        return true;
    }

    return false;
}
PT_THREAD(task_data_transfer(struct pt* pt))
{
    PT_BEGIN(pt);
    while(true) {
        PT_YIELD_UNTIL(pt, g_usb_stat.ready_to_send);
        //PT_YIELD_UNTIL(pt, buf_not_empty());
        if(process_packet()) {
            g_idle = false;
        }
    }
    PT_END(pt);
}

PT_THREAD(task_pwr_mgmt(struct pt* pt))
{
    PT_BEGIN(pt);
    while(true) {
        PT_YIELD_UNTIL(pt, g_idle);
        nrf_pwr_mgmt_run();
        g_idle = false;
    }
    PT_END(pt);
}



static struct pt pt_task_app_usb, pt_task_data_transfer, pt_task_pwr_mgmt;

int main()
{
    ret_code_t ret;

    init_log_packet_buf(&g_log_packet_buf);
    init_data_packet_buf(&g_data_packet_buf);

    // Initialize.
    power_init();
    timer_init();
    usb_init(); // since using power, must be called be ble init
    ble_stack_init();
    LEDS_CONFIGURE(PEER_NUM_LED_MASK | HOST_USB_LED_MASK);
    LEDS_OFF(PEER_NUM_LED_MASK | HOST_USB_LED_MASK);
    buttons_init();
    gatt_init();
    ble_conn_state_init();
    db_discovery_init();
    dts_init();
    scan_init();

    // Start execution.
    SEND_LOG("Multilink example started.\r\n");
    scan_start();

    timer_start();


    PT_INIT(&pt_task_app_usb);
    PT_INIT(&pt_task_data_transfer);
    PT_INIT(&pt_task_pwr_mgmt);

    while(true) {
        task_app_usb(&pt_task_app_usb);
        task_data_transfer(&pt_task_data_transfer);
        task_pwr_mgmt(&pt_task_pwr_mgmt);
    }
}