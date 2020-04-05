/**
 * Copyright 2020 DriveX.Tech. All rights reserved.
 * 
 * Licensed under the License.
 */

#include "bt_collector_cacher.h"
#include "bt_collector.h"
#include "usb_ser_cacher.h"
#include "packet_buf.h"
#include "packet_queue.h"
#include "util.h"
#include "uthash/uthash.h"
#include "uthash/utlist.h"
#include "uthash/utarray.h"
#include "app_util.h"
#include "app_util_platform.h"
#include <assert.h>



typedef struct _bt_data_queue_elem_t {
    BUF_ELEM_TYPE elem_info;
    BUF_ELEM_TYPE buf_elem;
    struct _bt_data_queue_elem_t* next;
} bt_data_queue_elem_t;

typedef struct _bt_pool_elem_t {
    BUF_ELEM_TYPE elem_info;
    bt_addr_t bt_addr;
    int ref_counter; // used for dealing with case when peer hardware reset then re-connect immediately, which will cause event: new CONNECT -> Outdated DISCONNECT
    bt_data_queue_elem_t* bt_data_queue_header;
    UT_hash_handle hh; /* makes this structure hashable */
} bt_pool_elem_t;

static bool init_pool_elem(bt_pool_elem_t* pool_elem)
{
    memset(pool_elem, 0, sizeof(bt_pool_elem_t));
    memset(&pool_elem->bt_addr, 0, sizeof(pool_elem->bt_addr));
    pool_elem->ref_counter = 0;
    pool_elem->bt_data_queue_header = NULL;

    return true;
}



static bt_pool_elem_t *g_bt_pool = NULL;

BUF_DEF(g_bt_pool_elem_buf, sizeof(bt_pool_elem_t), MAX_PEER_NUM);

BUF_DEF(g_bt_data_queue_elem_buf, sizeof(bt_data_queue_elem_t), (MAX_PEER_NUM * 8));

BUF_DEF(g_bt_data_queue_elem_data_buf, BLE_DTS_MAX_DATA_LEN, (MAX_PEER_NUM * 8));


static UT_icd bt_addr_icd = { sizeof(bt_addr_t), NULL, NULL, NULL };
static UT_array* g_ready_peers = NULL;
static UT_array* g_unready_peers = NULL;

static UT_icd pool_elem_icd = { sizeof(bt_pool_elem_t*), NULL, NULL, NULL };
static UT_array* g_toproess_elems = NULL;




int get_cached_bt_packet_num()
{
    int total_head_packet = 0;
    bt_pool_elem_t* pool_elem = NULL;
    bt_pool_elem_t* pool_elem_tmp = NULL;
    CRITICAL_REGION_ENTER();
    HASH_ITER(hh, g_bt_pool, pool_elem, pool_elem_tmp) {
        if(pool_elem->bt_data_queue_header != NULL) {
            total_head_packet++;
        }
    }
    CRITICAL_REGION_EXIT();
    return total_head_packet;
}

static int process_packet()
{
//    static BYTE dt_buff[BLE_DTS_MAX_DATA_LEN];
//    static int dt_buff_len = 0;

    int num_packet_processed = 0;

    bt_pool_elem_t* pool_elem = NULL;
    bt_pool_elem_t* pool_elem_tmp = NULL;
    bt_data_queue_elem_t* queue_elem = NULL;
    bt_data_queue_elem_t* queue_elem_tmp = NULL;
    BUF_ELEM_TYPE _elem;

    // Part 1. remove unready peer
    if(utarray_len(g_unready_peers) > 0) {
        CRITICAL_REGION_ENTER();
        for(bt_addr_t* addr = (bt_addr_t*)utarray_front(g_unready_peers); addr != NULL; addr = (bt_addr_t*)utarray_next(g_unready_peers, addr)) {
            //HASH_FIND(hh, g_bt_pool, addr, sizeof(bt_addr_t), pool_elem);
            HASH_FIND(hh, g_bt_pool, addr->addr1s, sizeof(addr->addr1s), pool_elem);
            if(pool_elem == NULL) {
                SEND_LOG("(%s): WARNNING: remove non-existing peer error!\r\n", __func__);
                continue;
            }

            pool_elem->ref_counter--;
            if(pool_elem->ref_counter > 0) {
                SEND_LOG("(%s): WARNNING: remove duplicated peer, new counter: %d!\r\n", __func__, pool_elem->ref_counter);
                continue;
            }

            HASH_DEL(g_bt_pool, pool_elem);

            LL_FOREACH_SAFE(pool_elem->bt_data_queue_header, queue_elem, queue_elem_tmp) {
                LL_DELETE(pool_elem->bt_data_queue_header, queue_elem);
                _elem = queue_elem->buf_elem;
                BUF_ELEM_FREE(g_bt_data_queue_elem_data_buf, _elem);
                _elem = queue_elem->elem_info;
                BUF_ELEM_FREE(g_bt_data_queue_elem_buf, _elem);
            }

            _elem = pool_elem->elem_info;
            BUF_ELEM_FREE(g_bt_pool_elem_buf, _elem);
        }
        utarray_clear(g_unready_peers);
        CRITICAL_REGION_EXIT();
    }

    // Part 2. add ready peer
    if(utarray_len(g_ready_peers) > 0) {
        CRITICAL_REGION_ENTER();
        for(bt_addr_t* addr = (bt_addr_t*)utarray_front(g_ready_peers); addr != NULL; addr = (bt_addr_t*)utarray_next(g_ready_peers, addr)) {
            //unsigned key_hashval;
            //HASH_VALUE(addr->addr1s, sizeof(addr->addr1s), key_hashval);;
            //HASH_FIND(hh, g_bt_pool, addr, sizeof(bt_addr_t), pool_elem);
            HASH_FIND(hh, g_bt_pool, addr->addr1s, sizeof(addr->addr1s), pool_elem);
            if(pool_elem != NULL) {
                SEND_LOG("(%s): WARNNING: add duplicated peer(0x%02x%02x%02x%02x%02x%02x), old counter: %d!\r\n", __func__,
                    addr->addr1s[0], addr->addr1s[1], addr->addr1s[2], addr->addr1s[3], addr->addr1s[4], addr->addr1s[5],
                    pool_elem->ref_counter);
                pool_elem->ref_counter++;
                continue;
            }

            BUF_ELEM_ALLOC(g_bt_pool_elem_buf, sizeof(bt_pool_elem_t), _elem);
            if(_elem.elem_data == NULL) {
                SEND_LOG("(%s): ERROR: alloc memory for bt_pool_elem FAILED!\r\n", __func__);
                continue;
            }

            pool_elem = (bt_pool_elem_t*)_elem.elem_data;
            init_pool_elem(pool_elem);
            pool_elem->elem_info = _elem;
            pool_elem->bt_addr = *addr;
            pool_elem->ref_counter = 1;

            //HASH_ADD(hh, g_bt_pool, bt_addr, sizeof(bt_addr_t), pool_elem);
            HASH_ADD(hh, g_bt_pool, bt_addr.addr1s, sizeof(addr->addr1s), pool_elem);
            //HASH_ADD_BYHASHVALUE(hh, g_bt_pool, bt_addr.addr1s, sizeof(addr->addr1s), key_hashval, pool_elem);
        }
        utarray_clear(g_ready_peers);
        CRITICAL_REGION_EXIT();
    }

    // Part 3. process bt data
    utarray_clear(g_toproess_elems);
    CRITICAL_REGION_ENTER();
    HASH_ITER(hh, g_bt_pool, pool_elem, pool_elem_tmp) {
        if(pool_elem->bt_data_queue_header != NULL) {
            utarray_push_back(g_toproess_elems, &pool_elem);
        }
    }
    CRITICAL_REGION_EXIT();
    for(bt_pool_elem_t** p_elem = (bt_pool_elem_t**)utarray_front(g_toproess_elems);
            p_elem != NULL;
            p_elem = (bt_pool_elem_t**)utarray_next(g_toproess_elems, p_elem)) {
        pool_elem = *p_elem;
        if(pool_elem == NULL) {
            continue;
        }
        while(pool_elem->bt_data_queue_header) {
            CRITICAL_REGION_ENTER();
            queue_elem = pool_elem->bt_data_queue_header;
            if(queue_elem != NULL) {
                LL_DELETE(pool_elem->bt_data_queue_header, queue_elem);
            }
            CRITICAL_REGION_EXIT();
            if(queue_elem == NULL) {
                break;
            } else {
                _elem = queue_elem->buf_elem;
                if(_elem.elem_data != NULL && _elem.elem_len > 0) {
                    bt_addr_t peer_addr;
                    assert(_elem.elem_len > sizeof(peer_addr.addr1s));
                    fast_copy(peer_addr.addr1s, pool_elem->bt_addr.addr1s, sizeof(peer_addr.addr1s));
                    bool ret = send_bt_data(&peer_addr, _elem.elem_data, _elem.elem_len);
                    if(ret) {
                        num_packet_processed++;
                    }
                    BUF_ELEM_FREE(g_bt_data_queue_elem_data_buf, _elem);
                }
                _elem = queue_elem->elem_info;
                BUF_ELEM_FREE(g_bt_data_queue_elem_buf, _elem);
            }
        }
    }

    return num_packet_processed;
}
PT_THREAD(process_cached_bt_packet(struct pt* pt, int* num_packet_processed))
{
    PT_BEGIN(pt);
    while(true) {
        // PT_YIELD_UNTIL(pt, !check_bt_busy());
        //PT_WAIT_UNTIL(pt, !check_bt_busy());
        //PT_WAIT_UNTIL(pt, (num_packet_processed && (*num_packet_processed = 0), !check_bt_busy()));
        
        int sent_packet_sent = process_packet();
        if(num_packet_processed) {
            *num_packet_processed = sent_packet_sent;
        }

        PT_YIELD(pt);
    }
    PT_END(pt);
}


static void on_service_ready_cber(const bt_addr_t* bt_addr) {
    CRITICAL_REGION_ENTER();
    utarray_push_back(g_ready_peers, bt_addr);
    CRITICAL_REGION_EXIT();
}

static void on_service_unready_cber(const bt_addr_t* bt_addr) {
    CRITICAL_REGION_ENTER();
    utarray_push_back(g_unready_peers, bt_addr);
    CRITICAL_REGION_EXIT();
}

bool bt_collector_cacher_init()
{
    g_bt_pool = NULL;

    BUF_INIT(g_bt_pool_elem_buf);
    BUF_INIT(g_bt_data_queue_elem_buf);
    BUF_INIT(g_bt_data_queue_elem_data_buf);

    utarray_new(g_ready_peers, &bt_addr_icd);
    utarray_new(g_unready_peers, &bt_addr_icd);

    utarray_new(g_toproess_elems, &pool_elem_icd);
    utarray_reserve(g_toproess_elems, 8);

    bt_cbers bt_cbers;
    memset(&bt_cbers, 0, sizeof(bt_cbers));
    bt_cbers.data_sent_cber = on_ble_data_sent;
    bt_cbers.data_received_cber = on_ble_data_received;
    bt_cbers.service_ready_cber = on_service_ready_cber;
    bt_cbers.service_unready_cber = on_service_unready_cber;
    bt_collector_set_cbers(&bt_cbers);

    return true;
}



bool send_bt_rawbytes(const bt_addr_t* bt_addr, const BYTE* data, WORD data_len)
{
    bt_pool_elem_t* pool_elem = NULL;
    BUF_ELEM_TYPE _elem;
    
    CRITICAL_REGION_ENTER();
    //HASH_FIND(hh, g_bt_pool, bt_addr, sizeof(bt_addr_t), pool_elem);
    HASH_FIND(hh, g_bt_pool, bt_addr->addr1s, sizeof(bt_addr->addr1s), pool_elem);
    //HASH_FIND_BYHASHVALUE(hh, g_bt_pool, bt_addr->addr1s, sizeof(bt_addr->addr1s), key_hashval, pool_elem);
    CRITICAL_REGION_EXIT();

    if(pool_elem == NULL) {
        return false;
    }

    BUF_ELEM_TYPE _elem2;
    BUF_ELEM_ALLOC(g_bt_data_queue_elem_data_buf, data_len, _elem2);
    if(_elem2.elem_data == NULL) {
        return false;
    }
    fast_copy(_elem2.elem_data, data, data_len);
    
    bt_data_queue_elem_t* queue_elem = NULL;
    BUF_ELEM_ALLOC(g_bt_data_queue_elem_buf, sizeof(bt_data_queue_elem_t), _elem);
    if(_elem.elem_data == NULL) {
        BUF_ELEM_FREE(g_bt_data_queue_elem_data_buf, _elem2);
        return false;
    }
    queue_elem = (bt_data_queue_elem_t*)_elem.elem_data;
    queue_elem->elem_info = _elem;
    queue_elem->buf_elem = _elem2;

    CRITICAL_REGION_ENTER();
    LL_APPEND(pool_elem->bt_data_queue_header, queue_elem);
    CRITICAL_REGION_EXIT();

    return true;
}