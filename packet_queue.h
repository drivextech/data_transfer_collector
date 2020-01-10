
#pragma once
#ifndef _PACKET_BUF_H_
#define _PACKET_BUF_H_

#include <stdbool.h>
#include "sdk_config.h"
#include "nrf_balloc.h"
#include "nrf_memobj.h"
#include "nrf_queue.h"
#include "types.h"
#include "util.h"



#define MAX_LOG_PACKET_LEN 280
#define MAX_LOG_PACKET_NUM 28

#define MAX_DATA_PACKET_LEN 1280
#define MAX_DATA_PACKET_NUM 18

#define MAX_CMD_PACKET_LEN 1280
#define MAX_CMD_PACKET_NUM 8

#define MAX_PACKET_LEN MAX(MAX_LOG_PACKET_LEN, MAX_DATA_PACKET_LEN)
#define MAX_PACKET_NUM 8



typedef struct {                                                                    \
    WORD elem_len;                                                                  \
    nrf_memobj_t* elem_obj;                                                         \
} QUEUE_ELEM_TYPE;
#define QUEUE_ELEM_NULL (QUEUE_ELEM_TYPE){ 0, NULL };
#define QUEUE_ELEM_LEN(ELEM_OBJ) (ELEM_OBJ.elem_len)
#define QUEUE_ELEM_ALLOC(ELEM_POOL_NAME, ELEM_LEN, ELEM_OBJ) do {                   \
    ELEM_OBJ.elem_obj = nrf_memobj_alloc(&ELEM_POOL_NAME, ELEM_LEN);                \
    assert(ELEM_OBJ.elem_obj != NULL);                                              \
    nrf_memobj_get(ELEM_OBJ.elem_obj);                                              \
    ELEM_OBJ.elem_len = ELEM_LEN;                                                   \
} while(0)
#define QUEUE_ELEM_FREE(ELEM_OBJ) do {                                              \
    if(ELEM_OBJ.elem_obj) {                                                         \
        nrf_memobj_put(ELEM_OBJ.elem_obj);                                          \
        ELEM_OBJ.elem_obj = NULL;                                                   \
        ELEM_OBJ.elem_len = 0;                                                      \
    }                                                                               \
} while(0)
#define QUEUE_ELEM_WRITE(ELEM_OBJ, ELEN_OBJ_OFF, DATA, DATA_LEN) do {               \
    assert(ELEM_OBJ.elem_obj != NULL && QUEUE_ELEM_LEN(ELEM_OBJ) >= DATA_LEN);      \
    nrf_memobj_write(ELEM_OBJ.elem_obj, DATA, DATA_LEN, ELEN_OBJ_OFF);              \
} while(0)
#define QUEUE_ELEM_READ(ELEM_OBJ, DATA, DATA_LEN) do {                              \
    assert(ELEM_OBJ.elem_obj != NULL);                                              \
    DATA_LEN = QUEUE_ELEM_LEN(ELEM_OBJ);                                            \
    nrf_memobj_read(ELEM_OBJ.elem_obj, DATA, DATA_LEN, 0);                          \
} while(0)

#define QUEUE_AVAILABLE(QUEUE_NAME) nrf_queue_available_get(&QUEUE_NAME)
#define QUEUE_POP(QUEUE_NAME, ELEM_OBJ) nrf_queue_pop(&QUEUE_NAME, &ELEM_OBJ)
#define QUEUE_PEEK(QUEUE_NAME, ELEM_OBJ) nrf_queue_peek(&QUEUE_NAME, &ELEM_OBJ)
#define QUEUE_PUSH(QUEUE_NAME, ELEM_OBJ) nrf_queue_push(&QUEUE_NAME, &ELEM_OBJ)


// NOT Thread SAFE!!!
// Maybe every thread can keep a private instance ?
typedef struct _packet_buf_t {
    const nrf_memobj_pool_t* packet_pool;
    const nrf_queue_t* packet_queue;
    QUEUE_ELEM_TYPE cur_elem_obj;
    WORD cur_elem_obj_write_ind;
} packet_buf_t;


bool init_log_packet_buf(packet_buf_t* packet_buf);
bool init_data_packet_buf(packet_buf_t* packet_buf);
bool init_cmd_packet_buf(packet_buf_t* packet_buf);

WORD packet_buf_prepare_data(packet_buf_t* packet_buf, WORD max_data_len);
WORD packet_buf_put_data(packet_buf_t* packet_buf, const BYTE* data, WORD data_len);
WORD packet_buf_commit_data(packet_buf_t* packet_buf);

WORD packet_buf_pop_data(packet_buf_t* packet_buf, BYTE* data, WORD data_len);
WORD packet_buf_peek_data(packet_buf_t* packet_buf, BYTE* data, WORD data_len);


#endif