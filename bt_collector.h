// Copyright 2020 DriveX.Tech. All rights reserved.
// 
// Licensed under the License.

#ifndef _BT_COLLECTOR_H_
#define _BT_COLLECTOR_H_

#include "ble.h"
#include "types.h"
#include "dts/ble_data_transfer_service_def.h"


#define MAX_PEER_NUM NRF_SDH_BLE_CENTRAL_LINK_COUNT


typedef void (*bt_data_sent_cber)(const bt_addr_t* bt_addr);
typedef void (*bt_data_received_cber)(const bt_addr_t* bt_addr, const BYTE* data, WORD data_len);
typedef void (*bt_conn_state_cber)(const bt_addr_t* bt_addr);

typedef struct _bt_cbers {
    bt_data_sent_cber data_sent_cber;
    bt_data_received_cber data_received_cber;
    bt_conn_state_cber service_ready_cber;
    bt_conn_state_cber service_unready_cber;
    bt_conn_state_cber bt_connected_cber; // Not impl now
    bt_conn_state_cber bt_disconnected_cber; // Not impl now
} bt_cbers;


void scan_init();

void scan_start();

void dts_init();

void ble_stack_init();

void db_discovery_init();

void gatt_init();


bool bt_collector_init();

bool bt_collector_set_cbers(const bt_cbers* bt_cbers);


bool send_bt_data(const bt_addr_t* bt_addr, const BYTE* data, WORD data_len);

int get_connected_peer_num();


#endif