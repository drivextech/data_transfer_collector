/**
 * Copyright 2020 DriveX.Tech. All rights reserved.
 * 
 * Licensed under the License.
 */

#include "usb_ser_cacher.h"
#include "usb_ser.h"


// Log / Data / CMD channel
packet_queue_t g_usb_log_packet_buf, g_usb_data_packet_buf, g_usb_cmd_packet_buf;



int get_cached_usb_packet_num()
{
    return packet_queue_used(&g_usb_log_packet_buf) + packet_queue_used(&g_usb_data_packet_buf) + packet_queue_used(&g_usb_cmd_packet_buf);
}

//TODO: check succesful
static bool process_packet()
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
PT_THREAD(process_cached_usb_packet(struct pt* pt, int* num_packet_processed))
{
    PT_BEGIN(pt);
    while(true) {
        //PT_YIELD_UNTIL(pt, !check_usb_busy());
        //PT_WAIT_UNTIL(pt, !check_usb_busy());
        PT_WAIT_UNTIL(pt, (num_packet_processed && (*num_packet_processed = 0), !check_usb_busy()));
        
        bool ret = process_packet();
        if(num_packet_processed) {
            *num_packet_processed = (int)ret;
        }

        PT_YIELD(pt);
    }
    PT_END(pt);
}


bool usb_ser_cacher_init()
{
    init_log_packet_queue(&g_usb_log_packet_buf);
    init_data_packet_queue(&g_usb_data_packet_buf);
    init_cmd_packet_queue(&g_usb_cmd_packet_buf);

    return true;
}