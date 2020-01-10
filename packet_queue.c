
#include <assert.h>
#include "packet_buf.h"



#define PACKET_POOL_NAME(PACKET_TYPE_NAME) CONCAT3(m_, PACKET_TYPE_NAME, _buf)
#define PACKET_QUEUE_NAME(PACKET_TYPE_NAME) CONCAT3(m_, PACKET_TYPE_NAME, _qun)

#define DEFINE_PACKET_BUF(PACKET_TYPE_NAME, PACKET_LEN, PACKET_NUM)                 \
NRF_MEMOBJ_POOL_DEF(PACKET_POOL_NAME(PACKET_TYPE_NAME),                             \
                    PACKET_LEN, PACKET_NUM);                                        \
NRF_QUEUE_DEF(QUEUE_ELEM_TYPE, PACKET_QUEUE_NAME(PACKET_TYPE_NAME),                 \
                PACKET_NUM, NRF_QUEUE_MODE_NO_OVERFLOW);


//#define INIT_PACKET_BUF(PACKET_TYPE_NAME)                                           \
//do {                                                                                \
//    ret_code_t err_code;                                                            \
//    err_code = nrf_memobj_pool_init(&PACKET_POOL_NAME(PACKET_TYPE_NAME));           \
//    APP_ERROR_CHECK(err_code);                                                      \
//    nrf_queue_reset(&PACKET_QUEUE_NAME(PACKET_TYPE_NAME));                          \
//} while(0)

inline static bool init_packet_buf(packet_buf_t* packet_buf)
{
    ret_code_t err_code;
    err_code = nrf_memobj_pool_init(packet_buf->packet_pool);
    //APP_ERROR_CHECK(err_code);
    nrf_queue_reset(packet_buf->packet_queue);

    return err_code == NRF_SUCCESS;
}

DEFINE_PACKET_BUF(log, MAX_LOG_PACKET_LEN, MAX_LOG_PACKET_NUM);
bool init_log_packet_buf(packet_buf_t* packet_buf)
{
#define LOG_POOL_NAME PACKET_POOL_NAME(log)
#define LOG_QUEUE_NAME PACKET_QUEUE_NAME(log)

    packet_buf->packet_pool = &LOG_POOL_NAME;
    packet_buf->packet_queue = &LOG_QUEUE_NAME;

    //INIT_PACKET_BUF(log);
    //return true;

    return init_packet_buf(packet_buf);
}

DEFINE_PACKET_BUF(data, MAX_DATA_PACKET_LEN, MAX_DATA_PACKET_NUM);
bool init_data_packet_buf(packet_buf_t* packet_buf)
{
#define DATA_POOL_NAME PACKET_POOL_NAME(data)
#define DATA_QUEUE_NAME PACKET_QUEUE_NAME(data)

    packet_buf->packet_pool = &DATA_POOL_NAME;
    packet_buf->packet_queue = &DATA_QUEUE_NAME;

    //INIT_PACKET_BUF(data);
    //return true;

    return init_packet_buf(packet_buf);
}

DEFINE_PACKET_BUF(cmd, MAX_CMD_PACKET_LEN, MAX_CMD_PACKET_NUM);
bool init_cmd_packet_buf(packet_buf_t* packet_buf)
{
#define CMD_POOL_NAME PACKET_POOL_NAME(cmd)
#define CMD_QUEUE_NAME PACKET_QUEUE_NAME(cmd)

    packet_buf->packet_pool = &CMD_POOL_NAME;
    packet_buf->packet_queue = &CMD_QUEUE_NAME;

    //INIT_PACKET_BUF(cmd);
    //return true;

    return init_packet_buf(packet_buf);
}


WORD packet_buf_prepare_data(packet_buf_t* packet_buf, WORD max_data_len)
{
    assert(max_data_len > 0 && max_data_len <= MAX_PACKET_LEN);

    QUEUE_ELEM_ALLOC(*packet_buf->packet_pool, max_data_len, packet_buf->cur_elem_obj);
    //TODO: check is successful
    packet_buf->cur_elem_obj_write_ind = 0;

    return packet_buf->cur_elem_obj.elem_len;
}

WORD packet_buf_put_data(packet_buf_t* packet_buf, const BYTE* data, WORD data_len)
{
    assert(data != NULL && data_len > 0 && data_len <= MAX_PACKET_LEN);

    QUEUE_ELEM_WRITE(packet_buf->cur_elem_obj, packet_buf->cur_elem_obj_write_ind, (BYTE*)data, (WORD)data_len);
    packet_buf->cur_elem_obj_write_ind += data_len;

    return data_len;
}

WORD packet_buf_commit_data(packet_buf_t* packet_buf)
{
    WORD data_put = 0;

    if(QUEUE_AVAILABLE(*packet_buf->packet_queue) == 0) {
        //TODO: log warning
        QUEUE_ELEM_TYPE obj_rub;
        QUEUE_POP(*packet_buf->packet_queue, obj_rub);
        QUEUE_ELEM_FREE(obj_rub);
    }

    QUEUE_ELEM_TYPE obj = packet_buf->cur_elem_obj;
    assert(obj.elem_len >= packet_buf->cur_elem_obj_write_ind);
    obj.elem_len = packet_buf->cur_elem_obj_write_ind;
    packet_buf->cur_elem_obj = QUEUE_ELEM_NULL;
    packet_buf->cur_elem_obj_write_ind = 0;

    ret_code_t err_code = QUEUE_PUSH(*packet_buf->packet_queue, obj);
    if(err_code == NRF_SUCCESS) {
        data_put = obj.elem_len;
    } else {
        //TODO: log warning
        QUEUE_ELEM_FREE(obj);
    }

    return data_put;
}


WORD packet_buf_pop_data(packet_buf_t* packet_buf, BYTE* data, WORD data_len)
{
    assert(data != NULL && data_len > 0 && data_len <= MAX_PACKET_LEN);

    WORD data_pop = 0;

    QUEUE_ELEM_TYPE obj;

    ret_code_t err_code = QUEUE_POP(*packet_buf->packet_queue, obj);
    if(err_code != NRF_SUCCESS) {
        return data_pop;
    }
    QUEUE_ELEM_READ(obj, data, data_len);
    QUEUE_ELEM_FREE(obj);
    data_pop = data_len;

    return data_pop;
}

WORD packet_buf_peek_data(packet_buf_t* packet_buf, BYTE* data, WORD data_len)
{
    assert(data != NULL && data_len > 0 && data_len <= MAX_PACKET_LEN);

    WORD data_peek = 0;

    QUEUE_ELEM_TYPE obj;

    ret_code_t err_code = QUEUE_PEEK(*packet_buf->packet_queue, obj);
    if(err_code != NRF_SUCCESS) {
        return data_peek;
    }
    QUEUE_ELEM_READ(obj, data, data_len);
    data_peek = data_len;

    return data_peek;
}