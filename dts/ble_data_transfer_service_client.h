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

#pragma once
#ifndef _BLE_DATA_TRANSFER_SERVICE_CLIENT_H_
#define _BLE_DATA_TRANSFER_SERVICE_CLIENT_H_

#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"
#include "../types.h"
#include "ble_data_transfer_service_def.h"

#ifdef __cplusplus
extern "C" {
#endif


#define APP_BLE_DTS_CLIENT_OBSERVER_PRIO 2 //Application's BLE observer priority


#define BLE_DTS_CLIENT_DEF(_name)                                                                   \
static ble_dts_t _name;                                                                      \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     APP_BLE_DTS_CLIENT_OBSERVER_PRIO,                                              \
                     ble_dts_client_evt_handler, &_name)

/** @brief Macro for defining multiple ble_dts_client_t instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_DTS_CLIENTS_DEF(_name, _cnt)                    \
static ble_dts_t _name[_cnt];                        \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                        \
                      APP_BLE_OBSERVER_PRIO,                \
                      ble_dts_client_evt_handler, &_name, _cnt)


bool ble_dts_client_init(ble_dts_t* dts_inst);

/**@brief Function for handling events from the database discovery module.
 */
void ble_dts_client_on_db_disc_evt(ble_dts_t* dts_inst, ble_db_discovery_evt_t* p_evt);

/**@brief     Function for handling BLE events from the SoftDevice.
 */
void ble_dts_client_evt_handler(const ble_evt_t* p_ble_evt, void* p_context);

bool ble_dts_client_cccd_configure(const ble_dts_t* dts_inst, bool enable);

DWORD ble_dts_client_send_data(const ble_dts_t* dts_inst, const BYTE* data, WORD data_len);


#ifdef __cplusplus
}
#endif

#endif