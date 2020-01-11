
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"
#include "boards.h"
#include "bsp_btn_ble.h"
#include "protothread/pt.h"
#include "util.h"
#include "packet_buf.h"
#include "packet_queue.h"
#include "bt_collector.h"
#include "bt_collector_cacher.h"
#include "usb_ser.h"
#include "usb_ser_cacher.h"


#define PEER_NUM_LED_MASK         BSP_LED_0_MASK
#define HOST_USB_LED_MASK         BSP_LED_1_MASK

#define USER_BUTTON_NO            BSP_BUTTON_0                          /**< Button for user */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(80)                   /**< Delay for a button is reported as pushed (in number of timer ticks). */


typedef struct _bt_data_t {
    bt_addr_t bt_addr;
    BUF_ELEM_TYPE buf_elem;
} bt_data_t;


typedef struct _usb_data_t {
    BUF_ELEM_TYPE buf_elem;
} usb_data_t;



#define MAX_PACKET_BUF_LENGTH 18

BUF_DEF(g_bt_buf, 256, MAX_PACKET_BUF_LENGTH);
BUF_DEF(g_usb_buf, 256, MAX_PACKET_BUF_LENGTH);



PT_THREAD(task_pwr_mgmt(struct pt* pt, volatile bool* idle))
{
    PT_BEGIN(pt);
    while(true) {
        //PT_YIELD_UNTIL(pt, *idle);
        PT_WAIT_UNTIL(pt, *idle);
        nrf_pwr_mgmt_run();
        if(SOFTDEVICE_PRESENT) {
            sd_app_evt_wait();
        } else {
            __WFI();
        }
        PT_YIELD(pt);
    }
    PT_END(pt);
}


static struct pt pt_task_bt, pt_task_usb, pt_task_pwr_mgmt;

void scheduler_handler()
{
    volatile bool idle = false;
    const int max_num_packet_cur = get_cached_bt_packet_num() + get_cached_usb_packet_num();
    int num_packet_cur = 0;
    while(! idle && num_packet_cur < max_num_packet_cur) {
        int num_bt_packet_processed = 0, num_usb_packet_processed = 0;
        process_cached_bt_packet(&pt_task_bt, &num_bt_packet_processed);
        process_cached_usb_packet(&pt_task_usb, &num_usb_packet_processed);

        num_packet_cur += num_bt_packet_processed + num_usb_packet_processed;

        if(num_bt_packet_processed == 0 && num_usb_packet_processed == 0) {
            idle = true;
        }

        task_pwr_mgmt(&pt_task_pwr_mgmt, &idle);
    }
}


void usb_data_scheduler_handler(void *p_event_data, uint16_t event_size)
{
    if(p_event_data != NULL && event_size > 0) {
        static BYTE recev_buf[MAX_PACKET_LEN];

        usb_data_t* usb_data = p_event_data;

        // SEND_LOG("(%s): usb data received: %lu B\r\n", __func__, usb_data->buf_elem.elem_len);
        // SEND_LOG("(%s)(buf: %u, %u): usb data received: %lu B\r\n", __func__,
        //         QUEUE_AVAILABLE(*g_log_packet_buf.packet_queue), QUEUE_AVAILABLE(*g_data_packet_buf.packet_queue), usb_data->buf_elem.elem_len);
        SEND_LOG("(%s)(usb_buf: %d): usb data received: %lu B\r\n", __func__,
                get_cached_usb_packet_num(), usb_data->buf_elem.elem_len);

        //TODO

        BUF_ELEM_FREE(g_usb_buf, usb_data->buf_elem);
    }

    scheduler_handler();
}

void on_usb_data_received(const BYTE* data, WORD data_len)
{
    assert(data != NULL && data_len > 0);

    ret_code_t err_code;

    usb_data_t usb_data;
    BUF_ELEM_ALLOC(g_bt_buf, data_len, usb_data.buf_elem);
    if(usb_data.buf_elem.elem_data == NULL) {
        SEND_LOG("(%s): BUF_ELEM_ALLOC failed!\r\n", __func__);
        return;
    }

    fast_copy(usb_data.buf_elem.elem_data, data, data_len);

    err_code = app_sched_event_put(&usb_data, sizeof(usb_data), usb_data_scheduler_handler);
    if(err_code != NRF_SUCCESS) {
        BUF_ELEM_FREE(g_usb_buf, usb_data.buf_elem);
        SEND_LOG("(%s): app_sched_event_put failed!\r\n", __func__);
    }
}

void on_usb_data_sent(const bt_addr_t* peer_addr)
{
    ret_code_t err_code;

    err_code = app_sched_event_put(NULL, 0, usb_data_scheduler_handler);
    if(err_code != NRF_SUCCESS) {
        SEND_LOG("(%s): app_sched_event_put failed!\r\n", __func__);
    }
}


void ble_data_scheduler_handler(void *p_event_data, uint16_t event_size)
{
    if(p_event_data != NULL && event_size > 0) {
        static BYTE recev_buf[MAX_PACKET_LEN];

        bt_data_t* bt_data = p_event_data;

        // SEND_LOG("(%s): ble data received: %lu B\r\n", __func__, bt_data->buf_elem.elem_len);
        // SEND_LOG("(%s)(bt_buf: %d, usb_buf: %d): ble data received: %lu B\r\n", __func__,
        //     get_cached_bt_packet_num(), get_cached_usb_packet_num(), bt_data->buf_elem.elem_len);

        fast_copy(recev_buf, bt_data->bt_addr.addr1s, sizeof(bt_data->bt_addr.addr1s));
        // fast_copy(recev_buf + sizeof(bt_data->bt_addr.addr1s), data, data_len);
        fast_copy(recev_buf + sizeof(bt_data->bt_addr.addr1s), bt_data->buf_elem.elem_data + 5, bt_data->buf_elem.elem_len - 5);

        // SEND_USB_DATA(recev_buf, sizeof(bt_data->bt_addr.addr1s) + bt_data->buf_elem.elem_len);
        SEND_USB_DATA(recev_buf, sizeof(bt_data->bt_addr.addr1s) - 5 + bt_data->buf_elem.elem_len);

        BUF_ELEM_FREE(g_bt_buf, bt_data->buf_elem);
    }

    scheduler_handler();
}

void on_ble_data_received(const bt_addr_t* peer_addr, const BYTE* data, WORD data_len)
{
    assert(data != NULL && data_len > 0);

    ret_code_t err_code;

    bt_data_t bt_data;
    bt_data.bt_addr = *peer_addr;
    BUF_ELEM_ALLOC(g_bt_buf, data_len, bt_data.buf_elem);
    if(bt_data.buf_elem.elem_data == NULL) {
        SEND_LOG("(%s): BUF_ELEM_ALLOC failed!\r\n", __func__);
        return;
    }

    fast_copy(bt_data.buf_elem.elem_data, data, data_len);

    err_code = app_sched_event_put(&bt_data, sizeof(bt_data), ble_data_scheduler_handler);
    if(err_code != NRF_SUCCESS) {
        BUF_ELEM_FREE(g_bt_buf, bt_data.buf_elem);
        SEND_LOG("(%s): app_sched_event_put failed!\r\n", __func__);
    }
}

void on_ble_data_sent(const bt_addr_t* peer_addr)
{
    ret_code_t err_code;

    err_code = app_sched_event_put(NULL, 0, ble_data_scheduler_handler);
    if(err_code != NRF_SUCCESS) {
        SEND_LOG("(%s): app_sched_event_put failed!\r\n", __func__);
    }
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press or release).
 */
static void button_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    if(pin_no == USER_BUTTON_NO) {
    };
}

/**@brief Function for initializing the button handler module.
 */
void buttons_init()
{
    ret_code_t err_code;

    static app_button_cfg_t button_conf;
    button_conf.pin_no = USER_BUTTON_NO;
    button_conf.pull_cfg = BUTTON_PULL;
    button_conf.active_state = false;
    button_conf.button_handler = button_handler;

    err_code = app_button_init(&button_conf, 1, BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}


void power_mgmt_init()
{
    ret_code_t err_code;

    nrf_drv_power_config_t power_conf;
    err_code = nrf_drv_power_init(NULL); // Must be called before SoftDevice init
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


APP_TIMER_DEF(m_app_timer_debug);
APP_TIMER_DEF(m_app_timer_maintainer);

void timer_debug_cb(void* p_context)
{
    static int cached_peer_num = -8;

    if(check_usb_connected()) {
//    if(!check_usb_busy()) {
        LEDS_ON(HOST_USB_LED_MASK);
    } else {
        LEDS_OFF(HOST_USB_LED_MASK);
    }

    int peer_num = get_connected_peer_num();

    if(peer_num == cached_peer_num) {
        if(cached_peer_num != 0) {
            LEDS_INVERT(PEER_NUM_LED_MASK);
        }
        return;
    }

    if(peer_num == 0) {
        LEDS_ON(PEER_NUM_LED_MASK);
    } else {
        app_timer_stop(m_app_timer_debug);
        if(peer_num > 0 && peer_num <= 8) {
            app_timer_start(m_app_timer_debug, APP_TIMER_TICKS(1000 * peer_num), NULL);
        } else {
            app_timer_start(m_app_timer_debug, APP_TIMER_TICKS(1000 / 8), NULL);
        }
    }

    cached_peer_num = peer_num;
}
void timer_maintainer_cb(void* p_context)
{
    app_sched_event_put(NULL, 0, usb_data_scheduler_handler);
}


/** @brief Function for initializing the timer.
 */
void timer_init()
{
    ret_code_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create debug timer.
    err_code = app_timer_create(&m_app_timer_debug, APP_TIMER_MODE_REPEATED, timer_debug_cb);
    // err_code = app_timer_create(&m_app_timer_debug, APP_TIMER_MODE_SINGLE_SHOT, timer_debug_cb);
    APP_ERROR_CHECK(err_code);

    // Create maintainer timer.
    err_code = app_timer_create(&m_app_timer_maintainer, APP_TIMER_MODE_REPEATED, timer_maintainer_cb);
    // err_code = app_timer_create(&m_app_timer_maintainer, APP_TIMER_MODE_SINGLE_SHOT, timer_maintainer_cb);
    APP_ERROR_CHECK(err_code);
}

void timer_start()
{
    SEND_LOG("(%s): start...\r\n", __func__);
    ret_code_t err_code;

    //err_code = app_timer_start(m_app_timer_debug, APP_TIMER_TICKS(100), NULL);
    err_code = app_timer_start(m_app_timer_debug, APP_TIMER_TICKS(1000), NULL);
    //err_code = app_timer_start(m_app_timer_debug, APP_TIMER_TICKS(10000), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_app_timer_maintainer, APP_TIMER_TICKS(180), NULL);
    APP_ERROR_CHECK(err_code);
}



int main()
{
    ret_code_t ret;

    // Scheduler settings
    //APP_TIMER_SCHED_EVENT_DATA_SIZE
    #define MAX_SCHED_EVENT_DATA_SIZE MAX(sizeof(bt_data_t), sizeof(usb_data_t))
    #define MAX_SCHED_QUEUE_SIZE 28
    APP_SCHED_INIT(MAX_SCHED_EVENT_DATA_SIZE, MAX_SCHED_QUEUE_SIZE);
    
    BUF_INIT(g_bt_buf);
    BUF_INIT(g_usb_buf);


    // Initialize.
    LEDS_CONFIGURE(PEER_NUM_LED_MASK | HOST_USB_LED_MASK);
    LEDS_OFF(PEER_NUM_LED_MASK | HOST_USB_LED_MASK);
    power_mgmt_init(); // must be called before SoftDevice
    timer_init();
    usb_init(); // since using power, must be called before ble init
    buttons_init();
    bt_collector_init();

    usb_ser_cacher_init();
    bt_collector_cacher_init();


    // Start execution.
    SEND_LOG("test_dtc started.\r\n");
    
    timer_start();
    scan_start();


    PT_INIT(&pt_task_bt);
    PT_INIT(&pt_task_usb);
    PT_INIT(&pt_task_pwr_mgmt);

    while(true) {
        app_sched_execute();
//        if(SOFTDEVICE_PRESENT) {
//            sd_app_evt_wait();
//        } else {
//            __WFI();
//        }
    }
}