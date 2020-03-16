// Copyright 2020 DriveX.Tech. All rights reserved.
// 
// Licensed under the License.

#ifndef _USB_SER_CACHER_H_
#define _USB_SER_CACHER_H_

#include "types.h"
#include "tlv_transformer.h"
#include "packet_buf.h"
#include "packet_queue.h"
#include "protothread/pt.h"



// Log / Data / CMD channel
extern packet_queue_t g_usb_log_packet_buf, g_usb_data_packet_buf, g_usb_cmd_packet_buf;


//#define SEND_RAWBYTE(...)
#define SEND_RAWBYTE(PACKET_BUF_NAME, DATA_TAG, DATA, DATA_LEN) do {                \
    static BYTE tlv_header_buf[TLV_HEADER_LEN];                                     \
    int header_len = tlv_header_generate(DATA_TAG, DATA_LEN, tlv_header_buf);       \
    assert(header_len >= 0);                                                        \
    int total_len = header_len + DATA_LEN;                                          \
    assert(total_len > 0);                                                          \
    QUEUE_ELEM_TYPE elem = packet_queue_prepare_data(&PACKET_BUF_NAME, total_len);  \
    assert(elem.elem_len == total_len);                                             \
    WORD seg_len = 0;                                                               \
    seg_len = packet_queue_put_data(&elem, seg_len, tlv_header_buf, header_len);    \
    assert(seg_len == header_len);                                                  \
    seg_len = packet_queue_put_data(&elem, seg_len, DATA, DATA_LEN);                \
    assert(seg_len == total_len);                                                   \
    seg_len = packet_queue_commit_data(&PACKET_BUF_NAME, &elem, total_len);         \
    assert(seg_len == total_len);                                                   \
} while(0)


//#define SEND_LOG(...)
#define SEND_LOG(...) do {                                                          \
    static BYTE packet_buf[MAX_PACKET_LEN];                                         \
    int packet_len = sprintf(packet_buf, __VA_ARGS__);                              \
    assert(packet_len >= 0);                                                        \
    SEND_RAWBYTE(g_usb_log_packet_buf, TAG_LOG, packet_buf, packet_len);            \
} while(0)
#define POP_LOG(DATA, DATA_LEN) packet_queue_pop_data(&g_usb_log_packet_buf, DATA, DATA_LEN)
#define PEEK_LOG(DATA, DATA_LEN) packet_queue_peek_data(&g_usb_log_packet_buf, DATA, DATA_LEN)

//#define SEND_USB_DATA(DATA, DATA_LEN)
#define SEND_USB_DATA(DATA, DATA_LEN) SEND_RAWBYTE(g_usb_data_packet_buf, TAG_DATA, DATA, DATA_LEN)
#define POP_USB_DATA(DATA, DATA_LEN) packet_queue_pop_data(&g_usb_data_packet_buf, DATA, DATA_LEN)
#define PEEK_USB_DATA(DATA, DATA_LEN) packet_queue_peek_data(&g_usb_data_packet_buf, DATA, DATA_LEN)


int get_cached_usb_packet_num();

PT_THREAD(process_cached_usb_packet(struct pt* pt, int* num_packet_processed));

bool usb_ser_cacher_init();


#endif