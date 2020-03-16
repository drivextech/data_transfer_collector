// Copyright 2020 DriveX.Tech. All rights reserved.
// 
// Licensed under the License.

#ifndef _USB_SER_H_
#define _USB_SER_H_

#include "types.h"



/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


extern void on_usb_data_received(const BYTE* data, WORD data_len);

extern void on_usb_data_sent();

void usb_init();

bool send_usb_data(const BYTE* data, WORD data_len);

bool check_usb_connected();

bool check_usb_busy();


#endif