#ifndef _BT_COLLECTOR_H_
#define _BT_COLLECTOR_H_

#include "ble.h"
#include "types.h"
#include "dts/ble_data_transfer_service_def.h"


void scan_init();

void scan_start();

void dts_init();

void ble_stack_init();

void db_discovery_init();

void gatt_init();


bool bt_collector_init();


extern void on_ble_data_received(const bt_addr_t* peer_addr, const BYTE* data, WORD data_len);

extern void on_ble_data_sent(const bt_addr_t* peer_addr);


bool send_bt_data(const bt_addr_t* bt_addr, const BYTE* data, WORD data_len);

int get_connected_peer_num();


#endif