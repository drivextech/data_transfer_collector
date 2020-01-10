#include "bt_collector_cacher.h"
#include "bt_collector.h"
#include "util.h"
#include <assert.h>


// Log / Data / CMD channel
packet_queue_t g_bt_data_packet_buf;


int get_cached_bt_packet_num()
{
    return packet_queue_used(&g_bt_data_packet_buf);
}

static bool process_packet()
{
    static BYTE dt_buff[MAX_PACKET_LEN];
    static int dt_buff_len = 0;
    bool ret = false;

    dt_buff_len = PEEK_BT_DATA(dt_buff, MAX_PACKET_LEN);
    if(dt_buff_len > 0) {
        bt_addr_t peer_addr;
        assert(dt_buff_len > sizeof(peer_addr.addr1s));
        fast_copy(peer_addr.addr1s, dt_buff, sizeof(peer_addr.addr1s));
        ret = send_bt_data(&peer_addr, dt_buff + sizeof(peer_addr.addr1s), dt_buff_len - sizeof(peer_addr.addr1s));
        if(ret) {
            dt_buff_len = POP_BT_DATA(dt_buff, MAX_PACKET_LEN);
        }
    }

    return ret;
}
PT_THREAD(process_cached_bt_packet(struct pt* pt, int* num_packet_processed))
{
    PT_BEGIN(pt);
    while(true) {
        // PT_YIELD_UNTIL(pt, !check_bt_busy());
        //PT_WAIT_UNTIL(pt, !check_bt_busy());
        //PT_WAIT_UNTIL(pt, (num_packet_processed && (*num_packet_processed = 0), !check_bt_busy()));
        
        bool ret = process_packet();
        if(num_packet_processed) {
            *num_packet_processed = (int)ret;
        }

        PT_YIELD(pt);
    }
    PT_END(pt);
}


bool bt_collector_cacher_init()
{
    init_data_packet_queue(&g_bt_data_packet_buf);

    return true;
}