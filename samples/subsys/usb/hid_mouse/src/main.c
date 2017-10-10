/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample app for WebUSB enabled custom class driver.
 *
 * Sample app for WebUSB enabled custom class driver. The received
 * data is echoed back to the WebUSB based application running in
 * the browser at host.
 */

#include <stdio.h>
#include <string.h>
#include <device.h>
#include <zephyr.h>
#include <usb_device.h>
#include <gpio.h>
#include <board.h>
//#include "nrf_drv_usbd.h"

#define SYS_LOG_LEVEL 4
#include <logging/sys_log.h>

/* Max Bluetooth command data size */
#define BTUSB_CLASS_MAX_DATA_SIZE 100

#define USB_ENDP_BULK_IN 0x81

static struct device *usb_dev;

#define DEV_DATA(dev) \
	((struct usb_dev_data_t * const)(dev)->driver_data)

#define DEVICE_SELF_POWERED 1
#define REMOTE_WU 1

#define USBD_CONFIG_DESCRIPTOR_SIZE 9
#define USBD_CONFIG_DESCRIPTOR_FULL_SIZE (9 + (9 + 9 + 7))

#define USBD_STRING_LANG_IX 0x00
#define USBD_STRING_MANUFACTURER_IX 0x01
#define USBD_STRING_PRODUCT_IX 0x02
#define USBD_STRING_SERIAL_IX 0x00
#define USBD_MOUSE_REPORT_DESCRIPTOR_SIZE 46
#define USBD_MOUSE_REPORT_DESCRIPTOR_IDX  176

#define USBD_HID0_DESCRIPTOR_SIZE 0x09
#define USBD_HID0_DESCRIPTOR_IDX 36

static bool send_mouse_report = false;

/* Device data structure */
struct usb_dev_data_t
{
	/* USB device status code */
	enum usb_dc_status_code usb_status;
	u8_t interface_data[BTUSB_CLASS_MAX_DATA_SIZE];
	u8_t notification_sent;
};

/**
 * USB descriptors configuration
 */
static const u8_t usb_desc[] = {
	/* Device descriptor */
	0x12,						 /* bLength | size of descriptor                                                  */
	0x01,						 /* bDescriptorType | descriptor type                                             */
	0x00, 0x02,					 /* bcdUSB | USB spec release (ver 2.0)                                           */
	0x00,						 /* bDeviceClass ¦ class code (each interface specifies class information)        */
	0x00,						 /* bDeviceSubClass ¦ device sub-class (must be set to 0 because class code is 0) */
	0x00,						 /* bDeviceProtocol | device protocol (no class specific protocol)                */
	64,							 /* bMaxPacketSize0 | maximum packet size (64 bytes)                              */
	0x15, 0x19,					 /* vendor ID  (0x1915 Nordic)                                                    */
	0x0A, 0x52,					 /* product ID (0x520A nRF52 HID mouse on nrf_drv)                                */
	0x01, 0x01,					 /* bcdDevice | final device release number in BCD Format                         */
	USBD_STRING_MANUFACTURER_IX, /* iManufacturer | index of manufacturer string                                  */
	USBD_STRING_PRODUCT_IX,		 /* iProduct | index of product string                                            */
	USBD_STRING_SERIAL_IX,		 /* iSerialNumber | Serial Number string                                          */
	0x01,						 /* bNumConfigurations | number of configurations                                 */

	/** Configuration descriptor */
	0x09,																			  /* bLength | length of descriptor                                             */
	0x02,																			  /* bDescriptorType | descriptor type (CONFIGURATION)                          */
	USBD_CONFIG_DESCRIPTOR_FULL_SIZE, 0x00,											  /* wTotalLength | total length of descriptor(s)  */
	0x01,																			  /* bNumInterfaces                                                             */
	0x01,																			  /* bConfigurationValue                                                        */
	0x00,																			  /* index of string Configuration | configuration string index (not supported) */
	0x80 | (((DEVICE_SELF_POWERED) ? 1U : 0U) << 6) | (((REMOTE_WU) ? 1U : 0U) << 5), /* bmAttributes    */
	49,																				  /* maximum power in steps of 2mA (98mA)                                       */

	/** Interface descriptor */
	0x09, /* bLength                                                                          */
	0x04, /* bDescriptorType | descriptor type (INTERFACE)                                    */
	0x00, /* bInterfaceNumber                                                                 */
	0x00, /* bAlternateSetting                                                                */
	0x01, /* bNumEndpoints | number of endpoints (1)                                          */
	0x03, /* bInterfaceClass | interface class (3..defined by USB spec: HID)                  */
	0x00, /* bInterfaceSubClass |interface sub-class (0.. no boot interface)                  */
	0x02, /* bInterfaceProtocol | interface protocol (1..defined by USB spec: mouse)          */
	0x00, /* interface string index (not supported)                                           */

	/** HID0 Descriptor */
	0x09,												/* bLength | length of descriptor (9 bytes)                    */
	0x21,												/* bHIDDescriptor | descriptor type (HID)                      */
	0x11, 0x01,											/* HID wBcdHID | Spec version 01.11                            */
	0x00,												/* bCountryCode | HW Target country                            */
	0x01,												/* bNumDescriptors | Number of HID class descriptors to follow */
	0x22,												/* bDescriptorType | Report descriptor type is 0x22 (report)   */
	(uint8_t)(USBD_MOUSE_REPORT_DESCRIPTOR_SIZE),		/* Total length of Report descr., low byte                     */
	(uint8_t)(USBD_MOUSE_REPORT_DESCRIPTOR_SIZE / 256), /* Total length of Report descr., high byte                    */

	/** Endpoint1 descriptor */
	0x07,		/* bLength | length of descriptor (7 bytes)                                     */
	0x05,		/* bDescriptorType | descriptor type (ENDPOINT)                                 */
	0x81,		/* bEndpointAddress | endpoint address (IN endpoint, endpoint 1)                */
	0x03,		/* bmAttributes | endpoint attributes (interrupt)                               */
	0x08, 0x00, /* bMaxPacketSizeLowByte,bMaxPacketSizeHighByte | maximum packet size (8 bytes) */
	0x08,		/* bInterval | polling interval (10ms)                                          */

	/** String config descriptor */
	0x04, /* length of descriptor                   */
	0x03, /* descriptor type                        */
	0x09, /*                                        */
	0x04,  /* Supported LangID = 0x0409 (US-English) */

	42,		   /* length of descriptor (? bytes)   */
	0x03,	  /* descriptor type                  */
	'N', 0x00, /* Define Unicode String "Nordic Semiconductor  */
	'o', 0x00,
	'r', 0x00,
	'd', 0x00,
	'i', 0x00,
	'c', 0x00,
	' ', 0x00,
	'S', 0x00,
	'e', 0x00,
	'm', 0x00,
	'i', 0x00,
	'c', 0x00,
	'o', 0x00,
	'n', 0x00,
	'd', 0x00,
	'u', 0x00,
	'c', 0x00,
	't', 0x00,
	'o', 0x00,
	'r', 0x00,

	72,		   /* length of descriptor (? bytes)         */
	0x03,	  /* descriptor type                        */
	'n', 0x00, /* generic unicode string for all devices */
	'R', 0x00,
	'F', 0x00,
	'5', 0x00,
	'2', 0x00,
	' ', 0x00,
	'U', 0x00,
	'S', 0x00,
	'B', 0x00,
	' ', 0x00,
	'H', 0x00,
	'I', 0x00,
	'D', 0x00,
	' ', 0x00,
	'm', 0x00,
	'o', 0x00,
	'u', 0x00,
	's', 0x00,
	'e', 0x00,
	' ', 0x00,
	'o', 0x00,
	'n', 0x00,
	' ', 0x00,
	'n', 0x00,
	'r', 0x00,
	'f', 0x00,
	'_', 0x00,
	'd', 0x00,
	'r', 0x00,
	'v', 0x00,
	' ', 0x00,
	'D', 0x00,
	'e', 0x00,
	'm', 0x00,
	'o', 0x00
};

static const u8_t mouse_desc[] = {
	/** Mouse report descriptor */
	0x05, 0x01, /* usage page (generic desktop). Global item, applies to all subsequent items   */
	0x09, 0x02, /* usage (mouse). Local item                                                    */
	0xA1, 0x01, /* collection (application)                                                     */
	0x09, 0x01, /* usage (pointer)                                                              */
	0xA1, 0x00, /* collection (physical)                                                        */
	0x05, 0x09, /*   usage page (buttons). Global item, applies to all subsequent items         */
	0x19, 0x01, /*   usage minimum (1)                                                          */
	0x29, 0x08, /*   usage maximum (8)                                                          */
	0x15, 0x00, /*   logical minimum (0)                                                        */
	0x25, 0x01, /*   logical maximum (1)                                                        */
	0x95, 0x08, /*   report count (8)                                                           */
	0x75, 0x01, /*   report size (1)                                                            */
	0x81, 0x02, /*   input (data, var, abs)                                                     */
	0x05, 0x01, /*   usage page (generic desktop). Global item, applies to all subsequent items */
	0x15, 0x81, /*   logical minimum (-127)                                                     */
	0x25, 0x7F, /*   logical maximum (127)                                                      */
	0x75, 0x08, /*   report size (8)                                                            */
	0x09, 0x30, /*   usage (X)                                                                  */
	0x09, 0x31, /*   usage (Y)                                                                  */
	0x09, 0x38, /*   usage wheel                                                                */
	0x95, 0x03, /*   report count (3)                                                           */
	0x81, 0x06, /*   input (3 position bytes X, Y & roller)                                     */
	0xC0,		/* end collection                                                               */
	0xC0		/* End Collection                                                               */
};

/* EP Bulk IN handler, used to send data to the Host */
static void usb_ep_in_handler(u8_t ep, enum usb_dc_ep_cb_status_code ep_status)
{
	SYS_LOG_ERR("");
}

/* Describe EndPoints configuration */
static struct usb_ep_cfg_data usb_ep[] = {
	{
		.ep_cb	= usb_ep_in_handler,
		.ep_addr = USB_ENDP_BULK_IN
	},
};

static void usb_status_cb(enum usb_dc_status_code status, u8_t *param)
{
	struct usb_dev_data_t *const dev_data = DEV_DATA(usb_dev);

	ARG_UNUSED(param);

	/* Store the new status */
	dev_data->usb_status = status;

	/* Check the USB status and do needed action if required */
	switch (status)
	{
	case USB_DC_ERROR:
		SYS_LOG_DBG("USB device error");
		break;
	case USB_DC_RESET:
		SYS_LOG_DBG("USB device reset detected");
		break;
	case USB_DC_CONNECTED:
		SYS_LOG_DBG("USB device connected");
		break;
	case USB_DC_CONFIGURED:
		SYS_LOG_DBG("USB device configured");
		send_mouse_report = true;
		break;
	case USB_DC_DISCONNECTED:
		send_mouse_report = false;
		SYS_LOG_DBG("USB device disconnected");
		break;
	case USB_DC_SUSPEND:
		send_mouse_report = false;
		SYS_LOG_DBG("USB device supended");
		break;
	case USB_DC_RESUME:
		SYS_LOG_DBG("USB device resumed");
		break;
	case USB_DC_UNKNOWN:
	default:
		SYS_LOG_DBG("USB unknown state");
		break;
	}
}

static int usb_custom_handler(struct usb_setup_packet *setup,
							   s32_t *len, u8_t **data)
{
	SYS_LOG_DBG("bRequest=%d", setup->bRequest);
	if (setup->bRequest == 0x06) //GetDescriptor
	{
		//determine which descriptor has been asked for
		if (((setup->wValue) >> 8) == 0x21) // HID
		{
			if ((setup->bmRequestType) == 0x81)
			{
				// Which interface
				if (((setup->wValue) & 0xFF) == 0)
				{
					SYS_LOG_DBG("Respond with HID0 descriptor");
					*data = (u8_t *)&usb_desc[USBD_HID0_DESCRIPTOR_IDX];
					*len = USBD_HID0_DESCRIPTOR_SIZE;
					return 0;
				}
			}
		}
		else if (((setup->wValue) >> 8) == 0x22) // HID report
		{
			if ((setup->bmRequestType) == 0x81)
			{
				// Which interface?
				if (((setup->wValue) & 0xFF) == 0)
				{
					SYS_LOG_DBG("Respond with HID report descriptor");
					*data = (u8_t *)mouse_desc;
					*len = USBD_MOUSE_REPORT_DESCRIPTOR_SIZE;
					return 0;
				}
			}
		}
	}

	return -1;
}

static int usb_class_handler(struct usb_setup_packet *setup,
							   s32_t *len, u8_t **data)
{
	SYS_LOG_DBG("bRequest=%d", setup->bRequest);
	switch (setup->bRequest)
	{
		case 0x0A: //SetIdle
		{
			if (setup->bmRequestType == 0x21)
			{
				//accept any value
				nrf_drv_usbd_setup_clear();
				return 0;
			}
			SYS_LOG_DBG("Set Idle wrong type: 0x%2x.", setup->bmRequestType);
			return -1;
		}
		break;
		//
		case 0x0B: // setProtocol or setInterface
			if (setup->bmRequestType == 0x01) // standard request, recipient=interface
			{
				//no alternate setting is supported - STALL always
				SYS_LOG_DBG("No alternate interfaces supported.");
				return -1;
			}
			else if (setup->bmRequestType == 0x21) // class request, recipient=interface
			{
				if (setup->bmRequestType == 0x21)
				{
					//accept any value
					nrf_drv_usbd_setup_clear();
					return 0;
				}
				SYS_LOG_DBG("Set Protocol wrong type: 0x%2x.", setup->bmRequestType);
				return -1;
			}
			else
			{
				SYS_LOG_DBG("Command 0xB. Unknown request: 0x%2x", setup->bmRequestType);
				return -1;
			}
			break;
	}

	return 0;
}

static struct usb_cfg_data usb_cfg = {
	.usb_device_description = usb_desc,
	.cb_usb_status = usb_status_cb,
	.interface = {
		.class_handler = usb_class_handler,
		.custom_handler = usb_custom_handler,
	},
	.num_endpoints = ARRAY_SIZE(usb_ep),
	.endpoint = usb_ep,
};

static int usb_init(struct device *dev)
{
	struct usb_dev_data_t *const dev_data = DEV_DATA(dev);
	int ret;

	usb_cfg.interface.payload_data = dev_data->interface_data;
	usb_dev = dev;

	/* Initialize the USB driver with the right configuration */
	ret = usb_set_config(&usb_cfg);
	if (ret < 0)
	{
		SYS_LOG_ERR("Failed to config USB");
		return ret;
	}

	/* Enable USB driver */
	ret = usb_enable(&usb_cfg);
	if (ret < 0)
	{
		SYS_LOG_ERR("Failed to enable USB");
		return ret;
	}

	return 0;
}

static void move_mouse_pointer(void)
{
	static uint32_t databuffer;
	static volatile uint8_t m_mouse_position = 0;
	int ret = 0;

	switch (m_mouse_position & 0x3)
	{
		case 0:
			/* X = 10, rest all are unchanged */
			databuffer = 0x00000A00;
			break;
		case 1:
			/* Y = 10, rest all are unchanged */
			databuffer = 0x000A0000;
			break;
		case 2:
			/* X = -10, rest all are unchanged */
			databuffer = 0x0000F600;
			break;
		case 3:
			/* Y = -10, rest all are unchanged */
			databuffer = 0x00F60000;
			break;
	}

	/* Send data */
	ret = usb_write(USB_ENDP_BULK_IN, (u8_t *)&databuffer, sizeof(databuffer), NULL);
	if (ret != 0)
	{
		SYS_LOG_DBG("move_mouse_pointer failed! ret=%u", ret);
	}
	else
	{
		m_mouse_position++;
	}
}

static struct usb_dev_data_t usb_dev_data = {
	.usb_status = USB_DC_UNKNOWN,
};

DEVICE_INIT(usb, "usb", &usb_init,
			&usb_dev_data, NULL,
			APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void main(void)
{
	struct device *gpiob;
	SYS_LOG_DBG("Start! %s", CONFIG_ARCH);

	gpiob = device_get_binding(SW0_GPIO_NAME);
	if (!gpiob)
	{
		SYS_LOG_DBG("GPIO binding failed");
		return;
	}

	gpio_pin_configure(gpiob, SW0_GPIO_PIN, GPIO_DIR_IN | GPIO_PUD_PULL_UP);

	while (1)
	{
		u32_t val = 0;

		gpio_pin_read(gpiob, SW0_GPIO_PIN, &val);


		if (send_mouse_report && !val)
			move_mouse_pointer();
		k_sleep(K_MSEC(10));
	}
}