/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ble_data_transfer_service_client.h"
#include "sdk_common.h"
#include "ble.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "nrf_log.h"


void ble_dts_client_on_db_disc_evt(ble_dts_t* dts_inst, ble_db_discovery_evt_t* p_evt)
{
    ble_gatt_db_srv_t* p_db_ser = &p_evt->params.discovered_db;
    ble_gatt_db_char_t* p_chars = p_db_ser->charateristics;

    if((p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        && (p_db_ser->srv_uuid.uuid == BLE_UUID_DTS_SERVICE)
        && (p_db_ser->srv_uuid.type == dts_inst->uuid_type)) {
        dts_inst->conn_handle = p_evt->conn_handle;
        for(int i=0; i < (int)p_db_ser->char_count; i++) {
            switch(p_chars[i].characteristic.uuid.uuid) {
                case DTS_SENDER_CHAR_UUID:
                    dts_inst->receiver_char_handle = p_chars[i].characteristic.handle_value;
                    dts_inst->cccd_char_handle = p_chars[i].cccd_handle;
                    break;
                case DTS_RECEIVER_CHAR_UUID:
                    dts_inst->sender_char_handle = p_chars[i].characteristic.handle_value;
                    break;
                default:
                    break;
            }
        }
        if(dts_inst->dts_cbers.service_ready_cber) {
            dts_inst->dts_cbers.service_ready_cber(dts_inst);
        }
    }
}

bool ble_dts_client_init(ble_dts_t* dts_inst)
{
    ret_code_t err_code;

    VERIFY_PARAM_NOT_NULL(dts_inst);

    memset(&dts_inst->dts_cbers, 0, sizeof(dts_inst->dts_cbers));

    // Initialize the service structure.
    dts_inst->service_handle         = BLE_CONN_HANDLE_INVALID;
    dts_inst->sender_char_handle     = BLE_GATT_HANDLE_INVALID;
    dts_inst->receiver_char_handle   = BLE_GATT_HANDLE_INVALID;
    dts_inst->cccd_char_handle       = BLE_GATT_HANDLE_INVALID;
    dts_inst->conn_handle            = BLE_CONN_HANDLE_INVALID;
    dts_inst->is_notification_enabled = false;

    ble_uuid_t    dts_short_uuid;
    ble_uuid128_t dts_vendor_uuid = DTS_VENDOR_UUID;
    dts_short_uuid.uuid = BLE_UUID_DTS_SERVICE;
    err_code = sd_ble_uuid_vs_add(&dts_vendor_uuid, &dts_short_uuid.type);
    APP_ERROR_CHECK(err_code);
    dts_inst->uuid_type = dts_short_uuid.type;

    err_code = ble_db_discovery_evt_register(&dts_short_uuid);
    APP_ERROR_CHECK(err_code);

    return true;
}

void ble_dts_client_evt_handler(const ble_evt_t* p_ble_evt, void* p_context)
{
    if((p_context == NULL) || (p_ble_evt == NULL)) {
        // NRF_LOG_ERROR("(%s): FATAL ERROR: p_ble_evt == NULL or p_context == NULL", __func__);
        return;
    }

    ret_code_t err_code;
    
    ble_dts_t* dts_inst = p_context;

    switch(p_ble_evt->header.evt_id) {
        case BLE_GATTC_EVT_HVX:
            do {
                const ble_gattc_evt_hvx_t* p_ghvn_evt = &p_ble_evt->evt.gattc_evt.params.hvx;
                if(dts_inst->conn_handle == p_ble_evt->evt.gattc_evt.conn_handle
                    && dts_inst->receiver_char_handle == p_ghvn_evt->handle) {
                    if(dts_inst->dts_cbers.data_received_cber) {
                        dts_inst->dts_cbers.data_received_cber(dts_inst, p_ghvn_evt->data, p_ghvn_evt->len);
                    }
                }
            } while(0);
            break;

        case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
            do {
                const ble_gattc_evt_hvx_t* p_ghvn_evt = &p_ble_evt->evt.gattc_evt.params.hvx;
                if(dts_inst->conn_handle == p_ble_evt->evt.gattc_evt.conn_handle
                    && dts_inst->receiver_char_handle == p_ghvn_evt->handle) {
                    if(dts_inst->dts_cbers.data_sent_cber) {
                        dts_inst->dts_cbers.data_sent_cber(dts_inst);
                    }
                }
            } while(0);
            break;

        case BLE_GAP_EVT_CONNECTED:
            do {
                // NRF_LOG_INFO("(%s): Connected", __func__);
                const ble_gap_evt_connected_t* p_connected_evt = &p_ble_evt->evt.gap_evt.params.connected;
                memcpy(dts_inst->peer_addr.addr1s, p_connected_evt->peer_addr.addr, sizeof(dts_inst->peer_addr.addr1s));
            } while(0);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            do {
                if(p_ble_evt->evt.gap_evt.conn_handle == dts_inst->conn_handle) {
                    // NRF_LOG_INFO("(%s): Disconnected", __func__);
                    if(dts_inst->dts_cbers.service_unready_cber) {
                        dts_inst->dts_cbers.service_unready_cber(dts_inst);
                    }
                    dts_inst->conn_handle = BLE_CONN_HANDLE_INVALID;
                }
            } while(0);
            break;

        default:
            // No implementation needed.
            break;
    }
}


bool ble_dts_client_cccd_configure(const ble_dts_t* dts_inst, bool enable)
{
    ret_code_t err_code;
    ble_gattc_write_params_t write_params;

    VERIFY_PARAM_NOT_NULL(dts_inst);

    if((dts_inst->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(dts_inst->cccd_char_handle == BLE_GATT_HANDLE_INVALID)) {
        return NRF_ERROR_INVALID_STATE;
    }

    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    buf[1] = 0;

    memset(&write_params, 0, sizeof(write_params));
    write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    write_params.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    write_params.handle   = dts_inst->cccd_char_handle;
    write_params.offset   = 0;
    write_params.len      = sizeof(buf);
    write_params.p_value  = buf;

    err_code = sd_ble_gattc_write(dts_inst->conn_handle, &write_params);

    return err_code == NRF_SUCCESS;
}


DWORD ble_dts_client_send_data(const ble_dts_t* dts_inst, const BYTE* data, WORD data_len)
{
    ret_code_t err_code;
    ble_gattc_write_params_t write_params;

    VERIFY_PARAM_NOT_NULL(dts_inst);

    if(dts_inst->conn_handle == BLE_CONN_HANDLE_INVALID) {
        NRF_LOG_WARNING("(%s): invald conn_handle, skip send data!", __func__);
        return NRF_ERROR_INVALID_STATE;
    }

    if(data_len > BLE_DTS_MAX_DATA_LEN) {
        NRF_LOG_WARNING("(%s): data length too large: %d B, max lenght supported: %d B, skip send data!", __func__, data_len, BLE_DTS_MAX_DATA_LEN);
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&write_params, 0, sizeof(write_params));
    write_params.write_op = BLE_GATT_OP_WRITE_CMD;
    write_params.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    write_params.handle   = dts_inst->sender_char_handle;
    write_params.offset   = 0;
    write_params.len      = data_len;
    write_params.p_value  = data;

    err_code = sd_ble_gattc_write(dts_inst->conn_handle, &write_params);

    if(err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("(%s): send data failed: err_code: %d", __func__, err_code);
    }

    return err_code;
}
