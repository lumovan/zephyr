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

static int usb_dc_tm4c123_init(void)
{
    const tConfigHeader *psHdr;
    const tConfigDescriptor *psDesc;

    g_ppsDevInfo[0] = psDevice;
    g_psDCDInst[0].pvCBData = pvDCDCBData;

    USBDCDDeviceInfoInit(ui32Index, psDevice);

    if(g_iUSBMode != eUSBModeOTG)
    {

        MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_USB0);
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
        MAP_SysCtlUSBPLLEnable();

        if(g_ui32PLLDiv == 0)
        {
            USBClockEnable(USB0_BASE, g_ui32PLLDiv, USB_CLOCK_EXTERNAL);
        }
        else
        {
            USBClockEnable(USB0_BASE, g_ui32PLLDiv, USB_CLOCK_INTERNAL);
        }

        if(g_ui32ULPISupport != USBLIB_FEATURE_ULPI_NONE)
        {
            USBULPIEnable(USB0_BASE);

            if(g_ui32ULPISupport & USBLIB_FEATURE_ULPI_HS)
            {
                ULPIConfigSet(USB0_BASE, ULPI_CFG_HS);
            }
            else
            {
                ULPIConfigSet(USB0_BASE, ULPI_CFG_FS);
            }
        }
        else
        {
            USBULPIDisable(USB0_BASE);
        }

        if(g_iUSBMode == eUSBModeForceDevice)
        {
            MAP_USBDevMode(USB0_BASE);
        }
        else if(g_iUSBMode == eUSBModeDevice)
        {
            MAP_USBOTGMode(USB0_BASE);
        }

        g_iUSBMode = eUSBModeDevice;


        if(g_psDCDInst[0].ui32Features & USBLIB_FEATURE_LPM_EN)
        {
            USBDevLPMConfig(USB0_BASE, USB_DEV_LPM_EN);
            USBLPMIntEnable(USB0_BASE, USB_INTLPM_RESUME | USB_INTLPM_ERROR |
                                       USB_INTLPM_ACK | USB_INTLPM_NYET);
            USBDevLPMEnable(USB0_BASE);

            g_psDCDInst[0].ui32LPMState = USBLIB_LPM_STATE_AWAKE;
        }
        else
        {
            USBDevLPMDisable(USB0_BASE);
            USBDevLPMConfig(USB0_BASE, USB_DEV_LPM_NONE);
            g_psDCDInst[0].ui32LPMState = USBLIB_LPM_STATE_DISABLED;
        }
    }

    g_psDCDInst[0].psDMAInstance = USBLibDMAInit(0);

    InternalUSBTickInit();

    psHdr = psDevice->ppsConfigDescriptors[
                                g_psDCDInst[0].ui32DefaultConfiguration - 1];
    psDesc = (const tConfigDescriptor *)(psHdr->psSections[0]->pui8Data);

    if((psDesc->bmAttributes & USB_CONF_ATTR_PWR_M) == USB_CONF_ATTR_SELF_PWR)
    {
        g_psDCDInst[0].ui8Status |= USB_STATUS_SELF_PWR;
    }
    else
    {
        g_psDCDInst[0].ui8Status &= ~USB_STATUS_SELF_PWR;
    }

    if(g_iUSBMode != eUSBModeOTG)
    {
        MAP_USBIntStatusControl(USB0_BASE);
        MAP_USBIntStatusEndpoint(USB0_BASE);

        MAP_USBIntEnableControl(USB0_BASE, USB_INTCTRL_RESET |
                                           USB_INTCTRL_DISCONNECT |
                                           USB_INTCTRL_RESUME |
                                           USB_INTCTRL_SUSPEND |
                                           USB_INTCTRL_SOF);
        MAP_USBIntEnableEndpoint(USB0_BASE, USB_INTEP_ALL);
        MAP_USBDevConnect(USB0_BASE);
        OS_INT_ENABLE(g_psDCDInst[0].ui32IntNum);
    }
}

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
