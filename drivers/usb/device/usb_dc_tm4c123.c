/* USB device controller driver for TM4C123 devices */

/*
 * Copyright (c) 2019 hackin zhao.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <string.h>
#include <usb/usb_device.h>
#include <misc/util.h>
#include <gpio.h>

#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(usb_dc_tm4c123);

/* Endpoint state */
struct usb_dc_tm4c123_ep_state {
	u16_t ep_mps; /** Endpoint max packet size */
	u8_t ep_type; /** Endpoint type (tm4c123 HAL enum) */
	usb_dc_ep_callback cb; /** Endpoint callback function */
	u8_t ep_stalled; /** Endpoint stall flag */
	u32_t read_count; /** Number of bytes in read buffer  */
	u32_t read_offset; /** Current offset in read buffer */
	struct k_sem write_sem; /** Write boolean semaphore */
};

/* Driver state */
struct usb_dc_tm4c123_state {

	usb_dc_status_callback status_cb; /* Status callback */
	struct usb_dc_tm4c123_ep_state out_ep_state[DT_USB_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_tm4c123_ep_state in_ep_state[DT_USB_NUM_BIDIR_ENDPOINTS];
	u8_t ep_buf[DT_USB_NUM_BIDIR_ENDPOINTS][EP_MPS];
};

/* Zephyr USB device controller API implementation */

int usb_dc_attach(void)
{
	LOG_DBG("");

	return 0;
}

int usb_dc_ep_set_callback(const u8_t ep, const usb_dc_ep_callback cb)
{
	return 0;
}

int usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	LOG_DBG("");

	return 0;
}

int usb_dc_set_address(const u8_t addr)
{
	return 0;
}

int usb_dc_ep_start_read(u8_t ep, u8_t *data, u32_t max_data_len)
{
	return 0;
}

int usb_dc_ep_get_read_count(u8_t ep, u32_t *read_bytes)
{
	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg)
{
	return 0;
}

int usb_dc_ep_set_stall(const u8_t ep)
{
	return 0;
}

int usb_dc_ep_clear_stall(const u8_t ep)
{
	return 0;
}

int usb_dc_ep_is_stalled(const u8_t ep, u8_t *const stalled)
{
	return 0;
}

int usb_dc_ep_enable(const u8_t ep)
{
	return 0;
}

int usb_dc_ep_disable(const u8_t ep)
{
	return 0;
}

int usb_dc_ep_write(const u8_t ep, const u8_t *const data, const u32_t data_len,
		    u32_t *const ret_bytes)
{
	return 0;
}

int usb_dc_ep_read_wait(u8_t ep, u8_t *data, u32_t max_data_len,
			u32_t *read_bytes)
{
	return 0;
}

int usb_dc_ep_read_continue(u8_t ep)
{
	return 0;
}

int usb_dc_ep_read(const u8_t ep, u8_t *const data, const u32_t max_data_len,
		   u32_t *const read_bytes)
{
	return 0;
}

int usb_dc_ep_mps(const u8_t ep)
{
	return 0;
}

int usb_dc_detach(void)
{
	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_reset(void)
{
	LOG_ERR("Not implemented");

	return 0;
}
