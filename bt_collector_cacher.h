// Copyright 2020 DriveX.Tech. All rights reserved.
// 
// Licensed under the License.

#ifndef _BT_COLLECTOR_CACHER_H_
#define _BT_COLLECTOR_CACHER_H_

#include "types.h"
#include "protothread/pt.h"
#include "bt_collector.h"


//#define SEND_BT_DATA(P_BT_ADDR, DATA, DATA_LEN)
#define SEND_BT_DATA(P_BT_ADDR, DATA, DATA_LEN) send_bt_rawbytes(P_BT_ADDR, DATA, DATA_LEN)
#define POP_BT_DATA(P_BT_ADDR, DATA, DATA_LEN) pop_queued_bt_data(, DATA, DATA_LEN)
#define PEEK_BT_DATA(P_BT_ADDR, DATA, DATA_LEN) peek_queued_bt_data(, DATA, DATA_LEN)



int get_cached_bt_packet_num(); // Note: NOT thread safe

PT_THREAD(process_cached_bt_packet(struct pt* pt, int* num_packet_processed));

bool bt_collector_cacher_init();



extern void on_ble_data_received(const bt_addr_t* peer_addr, const BYTE* data, WORD data_len);

extern void on_ble_data_sent(const bt_addr_t* peer_addr);


bool send_bt_rawbytes(const bt_addr_t* bt_addr, const BYTE* data, WORD data_len);


#endif