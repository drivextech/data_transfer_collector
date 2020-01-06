#pragma once
#ifndef _UTIL__TYPES_H_
#define _UTIL__TYPES_H_

#include <stdint.h>
#include <stddef.h>


typedef int8_t INT8;
typedef uint8_t UINT8;
typedef UINT8 BYTE;

typedef int16_t INT16;
typedef uint16_t UINT16;
typedef UINT16 WORD;

typedef int32_t INT32;
typedef uint32_t UINT32;
typedef UINT32 DWORD;

typedef int64_t INT64;
typedef uint64_t UINT64;
typedef UINT64 QWORD;

//typedef int128_t int128;
//typedef uint128_t uint128;
//typedef UINT128 OWORD;

typedef float FLOAT32;

typedef double FLOAT64;

typedef long double FLOAT128;



typedef struct _data_buf_hder_t {
    UINT16 data_len;
    BYTE* data;
} data_buf_hder_t;

typedef struct _data_buf_const_hder_t {
    UINT16 data_len;
    const BYTE* data;
} data_buf_const_hder_t;

typedef struct _data_buf_slice_t {
    UINT16 data_off;
    UINT16 data_len;
    BYTE* data;
} data_buf_slice_t;

typedef struct _data_buf_const_slice_t {
    UINT16 data_off;
    UINT16 data_len;
    const BYTE* data;
} data_buf_const_slice_t;


// common callbacker func
typedef void cber_func_t(void* data);
typedef void cber0_func_t(void* user_data);
typedef void cber1_func_t(void* user_data, void* cber_data);

// common callbacker struct
typedef struct _cber_t {
    cber_func_t* cber_func;
    void* cber_data;
    // void* user_data;
} cber_t; // to be deprecated, use `cber0_t` or `cber1_t` instead
#define NULL_CBER (cber_t){NULL, NULL}
typedef struct _cber0_t {
    cber0_func_t* cber_func;
    void* user_data;
} cber0_t;
#define NULL_CBER0 (cber0_t){NULL, NULL}
typedef struct _cber1_t {
    cber1_func_t* cber_func;
    void* user_data;
} cber1_t;
#define NULL_CBER1 (cber1_t){NULL, NULL}

#define IS_CBER_NULL(cber) (cber.cber_func == NULL)
#define IS_PCBER_NULL(pcber) (pcber == NULL || pcber->cber_func == NULL)

#define RUN_CBER(cber) \
if(cber.cber_func) { \
    cber.cber_func(cber.cber_data); \
}
#define RUN_CBER0(cber) \
if(cber.cber_func) { \
    cber.cber_func(cber.user_data); \
}
#define RUN_CBER1(cber, cber_data) \
if(cber.cber_func) { \
    cber.cber_func(cber.user_data, cber_data); \
}

#define RUN_PCBER(pcber) \
if(! IS_PCBER_NULL(pcber)) { \
    pcber->cber_func(pcber->cber_data); \
}
#define RUN_PCBER0(pcber) \
if(! IS_PCBER_NULL(pcber)) { \
    pcber->cber_func(pcber->user_data); \
}
#define RUN_PCBER1(pcber, cber_data) \
if(! IS_PCBER_NULL(pcber)) { \
    pcber->cber_func(pcber->user_data, cber_data); \
}

// alias, to be deprecated
#define RUN_CBER_IF_NOT_NULL(pcber) RUN_PCBER(pcber)
#define RUN_CBER0_IF_NOT_NULL(pcber) RUN_PCBER0(pcber)
#define RUN_CBER1_IF_NOT_NULL(pcber, cber_data) RUN_PCBER1(pcber, cber_data)


#endif