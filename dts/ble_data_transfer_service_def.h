#pragma once
#ifndef _BLE_DATA_TRANSFER_SERVICE_DEF_H_
#define _BLE_DATA_TRANSFER_SERVICE_DEF_H_

#include "sdk_config.h"
#include "ble.h"
#include "nrf_sdh_ble.h"
#include "../types.h"


#define DTS_VENDOR_UUID {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88} /**< Used vendor specific UUID. */

#define BLE_UUID_DTS_SERVICE 0x8888
#define DTS_SENDER_CHAR_UUID 0x8484
#define DTS_RECEIVER_CHAR_UUID 0x4848


/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the DTS module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_DTS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic DTS service module. */
#else
    #define BLE_DTS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic DTS service module. */
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif


typedef struct _bt_addr_t {
    BYTE addr1s[BLE_GAP_ADDR_LEN];
} bt_addr_t;

struct _ble_dts_t;
typedef struct _ble_dts_t ble_dts_t;
struct _ble_dts_cbers;
typedef struct _ble_dts_cbers ble_dts_cbers;

typedef void (*data_sent_cber)(const ble_dts_t* dts_inst);
typedef void (*data_received_cber)(const ble_dts_t* dts_inst, const BYTE* data, WORD data_len);
typedef void (*conn_state_cber)(const ble_dts_t* dts_inst);

struct _ble_dts_cbers {
    data_sent_cber data_sent_cber;
    data_received_cber data_received_cber;
    conn_state_cber service_ready_cber;
    conn_state_cber service_unready_cber;
};

struct _ble_dts_t {
    BYTE                        uuid_type;
    WORD                        service_handle;                 /**< Handle of Nordic DTS Service (as provided by the SoftDevice). */
    WORD                        sender_char_handle;
    WORD                        receiver_char_handle;
    WORD                        cccd_char_handle;
    WORD                        conn_handle;                    /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                        is_notification_enabled;
    bt_addr_t                   peer_addr;
    ble_dts_cbers               dts_cbers;
};


#endif