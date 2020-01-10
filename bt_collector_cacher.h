#ifndef _BT_COLLECTOR_CACHER_H_
#define _BT_COLLECTOR_CACHER_H_

#include "types.h"
#include "packet_buf.h"
#include "packet_queue.h"
#include "protothread/pt.h"



// Log / Data / CMD channel
extern packet_queue_t g_bt_data_packet_buf;


//#define SEND_BT_RAWBYTE(...)
#define SEND_BT_RAWBYTE(PACKET_BUF_NAME, DATA, DATA_LEN) do {                       \
    QUEUE_ELEM_TYPE elem = packet_queue_prepare_data(&PACKET_BUF_NAME, DATA_LEN);   \
    assert(elem.elem_len == DATA_LEN);                                              \
    int seg_len = packet_queue_put_data(&elem, 0, DATA, DATA_LEN);                  \
    assert(seg_len == DATA_LEN);                                                    \
    int total_len = packet_queue_commit_data(&PACKET_BUF_NAME, &elem, DATA_LEN);    \
    assert(total_len == DATA_LEN);                                                  \
} while(0)

//#define SEND_BT_DATA(DATA, DATA_LEN)
#define SEND_BT_DATA(DATA, DATA_LEN) SEND_BT_RAWBYTE(g_bt_data_packet_buf, DATA, DATA_LEN)
#define POP_BT_DATA(DATA, DATA_LEN) packet_queue_pop_data(&g_bt_data_packet_buf, DATA, DATA_LEN)
#define PEEK_BT_DATA(DATA, DATA_LEN) packet_queue_peek_data(&g_bt_data_packet_buf, DATA, DATA_LEN)



int get_cached_bt_packet_num();

PT_THREAD(process_cached_bt_packet(struct pt* pt, int* num_packet_processed));

bool bt_collector_cacher_init();


#endif