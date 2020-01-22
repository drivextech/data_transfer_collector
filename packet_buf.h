
#pragma once
#ifndef _PACKET_BUF_H_
#define _PACKET_BUF_H_

#include <assert.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "nrf_balloc.h"
#include "types.h"
#include "util.h"


typedef struct {                                                                    \
    WORD elem_len;                                                                  \
    BYTE* elem_data;                                                                \
} BUF_ELEM_TYPE;
#define BUF_ELEM_NULL (BUF_ELEM_TYPE){ 0, NULL };
#define BUF_ELEM_LEN(ELEM_OBJ) ((ELEM_OBJ).elem_len)
#define BUF_ELEM_ALLOC(ELEM_POOL_NAME, ELEM_LEN, ELEM_OBJ) do {                     \
    (ELEM_OBJ).elem_data = nrf_balloc_alloc(&ELEM_POOL_NAME);                       \
    (ELEM_OBJ).elem_len = (ELEM_OBJ).elem_data ? ELEM_LEN : 0;                      \
} while(0)
#define BUF_ELEM_FREE(ELEM_POOL_NAME, ELEM_OBJ) do {                                \
    if((ELEM_OBJ).elem_data) {                                                      \
        nrf_balloc_free(&ELEM_POOL_NAME, (ELEM_OBJ).elem_data);                     \
        (ELEM_OBJ).elem_data = NULL;                                                \
        (ELEM_OBJ).elem_len = 0;                                                    \
    }                                                                               \
} while(0)

#define BUF_DEF(BUF_NAME, MAX_ELEM_SIZE, MAX_ELEM_NUM) NRF_BALLOC_DEF(BUF_NAME, MAX_ELEM_SIZE, MAX_ELEM_NUM)
#define BUF_INIT(BUF_NAME) nrf_balloc_init(&BUF_NAME)
#define BUF_AVAILABLE(BUF_NAME) nrf_balloc_utilization_get(&BUF_NAME)


typedef struct _packet_buf_t {
    const nrf_balloc_t* packet_pool;
} packet_buf_t;


#endif