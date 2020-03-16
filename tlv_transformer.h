// Copyright 2020 DriveX.Tech. All rights reserved.
// 
// Licensed under the License.

#pragma once
#ifndef _TLV_TRANSFORMER_H_
#define _TLV_TRANSFORMER_H_

#include <stdbool.h>
#include "types.h"


#ifdef __cplusplus
extern "C" {
#endif


#define TLV_HEADER_LEN 3

typedef enum _tag_t {
    TAG_DATA = 2,
    TAG_LOG = 4,
    TAG_CMD = 8,
} tag_t;


int tlv_parse(const BYTE* data, tag_t* tag, WORD* len, BYTE* payload);

int tlv_generate(const tag_t tag, const WORD len, const BYTE* payload, BYTE* data);

int tlv_header_parse(const BYTE* data, tag_t* tag, WORD* len);

int tlv_header_generate(const tag_t tag, const WORD len, BYTE* data);


#ifdef __cplusplus
}
#endif


#endif