#include <string.h>
#include <assert.h>
#include "tlv_transformer.h"

int tlv_parse(const BYTE* data, tag_t* tag, WORD* len, BYTE* payload)
{
    assert(data != NULL);

    WORD _len;

    int total_len = tlv_header_parse(data, tag, &_len);

    if(len) {
        *len = _len;
    }

    if(payload && _len > 0) {
        memcpy(payload, data + 3, _len);
        total_len += _len;
    }

    return total_len;
}

int tlv_generate(const tag_t tag, const WORD len, const BYTE* payload, BYTE* data)
{
    assert(data != NULL);

    int total_len = tlv_header_generate(tag, len, data);

    if(payload && len > 0) {
        memcpy(data + total_len, payload, len);
        total_len += len;
    }

    return total_len;
}


int tlv_header_parse(const BYTE* data, tag_t* tag, WORD* len)
{
    assert(data != NULL);

    int total_len = -8;

    if(tag) {
        *tag = data[0];
    }

    WORD _len = data[1] + (data[2] << 8);
    if(len) {
        *len = _len;
    }

    return 3;
}

int tlv_header_generate(const tag_t tag, const WORD len, BYTE* data)
{
    assert(data != NULL);

    data[0] = tag;

    data[1] = len;
    data[2] = len >> 8;

    return 3;
}