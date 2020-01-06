
#ifndef _SERIAL_NUM_H_
#define _SERIAL_NUM_H_

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup app_usbd_serial_num USBD serial number generator
 * @ingroup app_usbd
 *
 * @brief @tagAPI52840 Generate a standard USB serial number that is unique for each device.
 * @{
 */

/**@brief Function for generating a default serial number string based on FIRC->DEVICEADDR.
 *
 * After calling this function, the serial number is ready for the USB driver.
 *
 * The generated serial number shows up as a 12-hexidecimal-digit string with no delimiters
 * (e.g 123456ABCDEF). The byte string is also printed on the PCA10059 dongle. It is also used as
 * the default advertising address in the SoftDevice.
 */
void serial_num_generate(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
