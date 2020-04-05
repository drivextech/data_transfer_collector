/**
 * Copyright 2020 DriveX.Tech. All rights reserved.
 * 
 * Licensed under the License.
 */

#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"
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
#include "bsp_btn_ble.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include <assert.h>
#include "bt_collector.h"
#include "util.h"
#include "usb_ser_cacher.h"
#include "dts/ble_data_transfer_service_client.h"


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


static const char m_target_periph_name[] = "NordicDTS";             /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */



typedef struct _bt_stat_t {
    int peer_num;
} bt_stat_t;


static volatile bt_stat_t g_bt_stat;




NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DTS_CLIENTS_DEF(m_dts_clients, MAX_PEER_NUM);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, MAX_PEER_NUM);  /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */


static const bt_cbers BT_CBERS0 = (bt_cbers){NULL, NULL, NULL, NULL, NULL, NULL};
static bt_cbers g_bt_cbers = BT_CBERS0;



static bt_addr_t g_handle_peeraddr_table[MAX_PEER_NUM];

static bool is_btaddr_equal(const bt_addr_t* addr1, const bt_addr_t* addr2) {
    //return memcmp(addr1, addr2, sizeof(bt_addr_t)) == 0;
    return memcmp(addr1->addr1s, addr2->addr1s, sizeof(addr1->addr1s)) == 0;
}

static int find_btaddr(const bt_addr_t* addr) {
    for(int ind = 0; ind < (int)MAX_PEER_NUM; ind++) {
        if(is_btaddr_equal(addr, &g_handle_peeraddr_table[ind])) {
            return ind;
        }
    }
    return -8;
}



/**@brief Function for starting scanning. */
void scan_start()
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
                  const ble_gap_evt_connected_t* p_connected =
                                   p_scan_evt->params.connected.p_connected;
                 // Scan is automatically stopped by the connection.
                 SEND_LOG("(%s): Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                        __func__,
                        p_connected->peer_addr.addr[0],
                        p_connected->peer_addr.addr[1],
                        p_connected->peer_addr.addr[2],
                        p_connected->peer_addr.addr[3],
                        p_connected->peer_addr.addr[4],
                        p_connected->peer_addr.addr[5]);
             } while(0);
             break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
             do {
                 SEND_LOG("(%s): Scan timed out.\r\n", __func__);
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
void scan_init()
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

    //ble_uuid_t dts_uuid = {BLE_UUID_DTS_SERVICE, BLE_UUID_TYPE_UNKNOWN};
//    ble_uuid_t dts_uuid = {BLE_UUID_DTS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN};
//    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &dts_uuid);
//    APP_ERROR_CHECK(err_code);
//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
//    APP_ERROR_CHECK(err_code);

//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_ALL_FILTER, false);
//    APP_ERROR_CHECK(err_code);
}


static const char* get_phy_name(BYTE phy_id)
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

                SEND_LOG("(%s): conn_handle(0x%x) established, starting DB discovery.\r\n", __func__, conn_handle);
                // SEND_LOG("(%s): conn_handle(0x%x), addr: %02x%02x%02x%02x%02x%02x\r\n", __func__,
                //         conn_handle,
                //         p_gap_evt->params.connected.peer_addr.addr[0],
                //         p_gap_evt->params.connected.peer_addr.addr[1],
                //         p_gap_evt->params.connected.peer_addr.addr[2],
                //         p_gap_evt->params.connected.peer_addr.addr[3],
                //         p_gap_evt->params.connected.peer_addr.addr[4],
                //         p_gap_evt->params.connected.peer_addr.addr[5]);

                APP_ERROR_CHECK_BOOL(conn_handle < MAX_PEER_NUM);

                ble_gap_phys_t phys = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);

                err_code = ble_db_discovery_start(&m_db_disc[conn_handle],
                                                  conn_handle);
                if(err_code != NRF_ERROR_BUSY) {
                    APP_ERROR_CHECK(err_code);
                }

                if(ble_conn_state_central_conn_count() >= MAX_PEER_NUM) {
                } else {
                    SEND_LOG("(%s): Connected: continue scan!\r\n", __func__);
                    scan_start();
                }
            } while(0);
            break;

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
            do {
                conn_handle = p_gap_evt->conn_handle;

                SEND_LOG("(%s): Disconnected. conn_handle: 0x%x, reason: 0x%x\r\n",
                            __func__,
                            conn_handle,
                            p_gap_evt->params.disconnected.reason);

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
                    SEND_LOG("(%s): Connection request timed out.\r\n", __func__);

//                    memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));
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
                SEND_LOG("(%s): GATT client timeout.\r\n", __func__);
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);

//                memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));
            } while(0);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            do {
                // Disconnect on GATT server timeout event.
                SEND_LOG("(%s): GATT server timeout.\r\n", __func__);
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);

//                memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));
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


static void on_service_ready_cber(const ble_dts_t* dts_inst)
{
    ret_code_t err_code;

    err_code = ble_dts_client_cccd_configure(dts_inst, true);
    APP_ERROR_CHECK_BOOL(err_code);

    uint16_t conn_handle = dts_inst->conn_handle;
    fast_copy(&g_handle_peeraddr_table[conn_handle], &dts_inst->peer_addr, sizeof(g_handle_peeraddr_table[conn_handle]));

    g_bt_stat.peer_num++;

    SEND_LOG("(%s): DTS discovered on conn_handle(0x%x)\r\n", __func__, conn_handle);

    if(conn_handle < MAX_PEER_NUM) {
        // assert `g_handle_peeraddr_table[conn_handle]` Not Invalid!
        if(g_bt_cbers.service_ready_cber) {
            g_bt_cbers.service_ready_cber(&g_handle_peeraddr_table[conn_handle]);
        }
    }
}

static void on_service_unready_cber(const ble_dts_t* dts_inst)
{
    uint16_t conn_handle = dts_inst->conn_handle;

    g_bt_stat.peer_num--;

    if(conn_handle < MAX_PEER_NUM) {
        // assert `g_handle_peeraddr_table[conn_handle]` Not Invalid!
        if(g_bt_cbers.service_unready_cber) {
            g_bt_cbers.service_unready_cber(&g_handle_peeraddr_table[conn_handle]);
        }
    }
    
    SEND_LOG("(%s): Disconnected on conn_handle(0x%x)\r\n", __func__, conn_handle);

    memset(&g_handle_peeraddr_table[conn_handle], 0, sizeof(g_handle_peeraddr_table[conn_handle]));
}

static void on_data_sent_cber(const ble_dts_t* dts_inst)
{
    uint16_t conn_handle = dts_inst->conn_handle;

    // SEND_LOG("(%s): conn(0x%x) data sent\r\n", __func__, conn_handle);

    // assert `g_handle_peeraddr_table[conn_handle]` Not Invalid!
    if(conn_handle < MAX_PEER_NUM) {
        // assert `g_handle_peeraddr_table[conn_handle]` Not Invalid!
        if(g_bt_cbers.data_sent_cber) {
            g_bt_cbers.data_sent_cber(&g_handle_peeraddr_table[conn_handle]);
        }
    }
}

static void on_data_received_cber(const ble_dts_t* dts_inst, const BYTE* data, WORD data_len)
{
    uint16_t conn_handle = dts_inst->conn_handle;

    // SEND_LOG("(%s): conn(0x%x) data received: %u B\r\n", __func__, conn_handle, data_len);

    if(conn_handle < MAX_PEER_NUM) {
        // assert `g_handle_peeraddr_table[conn_handle]` Not Invalid!
        if(g_bt_cbers.data_received_cber) {
            g_bt_cbers.data_received_cber(&g_handle_peeraddr_table[conn_handle], data, data_len);
        }
    }
}

void dts_init()
{
    ble_dts_cbers dts_cbers;
    memset(&dts_cbers, 0, sizeof(dts_cbers));
    dts_cbers.service_ready_cber = on_service_ready_cber;
    dts_cbers.service_unready_cber = on_service_unready_cber;
    dts_cbers.data_sent_cber = on_data_sent_cber;
    dts_cbers.data_received_cber = on_data_received_cber;
    for(int i = 0; i < (int)MAX_PEER_NUM; i++) {
        bool ret = ble_dts_client_init(&m_dts_clients[i]);
        APP_ERROR_CHECK_BOOL(ret);
        m_dts_clients[i].dts_cbers = dts_cbers;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
void ble_stack_init()
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
//    SEND_LOG("(%s): call to ble_dts_client_on_db_disc_evt for conn_handle 0x%x, addr: %02x%02x%02x%02x%02x%02x!\r\n",
//                __func__, p_evt->conn_handle,
//                m_dts_clients[p_evt->conn_handle].peer_addr.addr1s[0],
//                m_dts_clients[p_evt->conn_handle].peer_addr.addr1s[1],
//                m_dts_clients[p_evt->conn_handle].peer_addr.addr1s[2],
//                m_dts_clients[p_evt->conn_handle].peer_addr.addr1s[3],
//                m_dts_clients[p_evt->conn_handle].peer_addr.addr1s[4],
//                m_dts_clients[p_evt->conn_handle].peer_addr.addr1s[5]);

    ble_dts_client_on_db_disc_evt(&m_dts_clients[p_evt->conn_handle], p_evt);
}

/** @brief Database discovery initialization.
 */
void db_discovery_init()
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, const nrf_ble_gatt_evt_t* p_evt)
{
    uint16_t conn_handle = p_evt->conn_handle;

    uint32_t data_length;
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
void gatt_init()
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


bool bt_collector_init()
{
    ble_stack_init();
    gatt_init();
    ble_conn_state_init();
    db_discovery_init();
    dts_init();
    scan_init();

    return true;
}


bool bt_collector_set_cbers(const bt_cbers* bt_cbers)
{
    if(bt_cbers) {
        g_bt_cbers = *bt_cbers;
    } else {
        g_bt_cbers = BT_CBERS0;
    }
    return true;
}


bool send_bt_data(const bt_addr_t* bt_addr, const BYTE* data, WORD data_len)
{
    assert(bt_addr != NULL && data != NULL && data_len > 0);

    //TODO: multi-packet protocol implement!

    int conn_handle = find_btaddr(bt_addr);
    if(conn_handle < 0) {
        return false;
    }

    DWORD err_code = ble_dts_client_send_data(&m_dts_clients[conn_handle], data, data_len);

    return err_code == NRF_SUCCESS;
}


int get_connected_peer_num()
{
    return g_bt_stat.peer_num;
}