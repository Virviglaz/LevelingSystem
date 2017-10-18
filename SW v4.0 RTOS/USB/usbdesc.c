/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 *      Name:    USBDESC.C
 *      Purpose: USB Descriptors
 *      Version: V1.10
 *----------------------------------------------------------------------------
 *      This file is part of the uVision/ARM development tools.
 *      This software may only be used under the terms of a valid, current,
 *      end user licence from KEIL for a compatible version of KEIL software
 *      development tools. Nothing else gives you the right to use it.
 *
 *      Copyright (c) 2005-2007 Keil Software.
 *---------------------------------------------------------------------------*/

#include "type.h"
#include "usb.h"
#include "hid.h"
#include "usbcfg.h"
#include "usbdesc.h"

const BYTE HID_ReportDescriptor[] = {
	  0x06, 0x01, 0xff,              // USAGE_PAGE (Not Defined)
    0x09, 0x00,                    // USAGE (Undefined)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, USB_PACKET_LEN,          //   REPORT_COUNT (?)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0x19, 0x00,                    //   USAGE_MINIMUM (Undefined)
    0x29, 0x00,                    //   USAGE_MAXIMUM (Undefined)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, USB_PACKET_LEN,          //   REPORT_COUNT (?)
    0x91, 0x00,                    //   OUTPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
const WORD HID_ReportDescSize = sizeof(HID_ReportDescriptor);


/* USB Standard Device Descriptor */
const BYTE USB_DeviceDescriptor[] = {
  USB_DEVICE_DESC_SIZE,              	/* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,        	/* bDescriptorType */
  WBVAL(0x0110), /* 1.10 */          	/* bcdUSB */
  0x00,                              	/* bDeviceClass */
  0x00,                              	/* bDeviceSubClass */
  0x00,                              	/* bDeviceProtocol */
  USB_MAX_PACKET0,                   	/* bMaxPacketSize0 */
  WBVAL(USB_VID),                     /* idVendor */
  WBVAL(USB_PID),                     /* idProduct */
  WBVAL(0x0100), /* 1.00 */          	/* bcdDevice */
  4,                              		/* iManufacturer */
  32,                              		/* iProduct */
  64,                              		/* iSerialNumber */
  1                               		/* bNumConfigurations */
};

/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const BYTE USB_ConfigDescriptor[] = {
/* Configuration 1 */
  USB_CONFIGUARTION_DESC_SIZE,       /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
  WBVAL(                             /* wTotalLength */
		USB_CONFIGUARTION_DESC_SIZE +
    USB_INTERFACE_DESC_SIZE     +
    HID_DESC_SIZE               +
    USB_ENDPOINT_DESC_SIZE	
  ),
  0x01,                              /* bNumInterfaces */
  0x01,                              /* bConfigurationValue */
  0x00,                              /* iConfiguration */
  USB_CONFIG_BUS_POWERED /*|*/       /* bmAttributes */
/*USB_CONFIG_REMOTE_WAKEUP*/,
  USB_CONFIG_POWER_MA(MaxPowerConsuption),          /* bMaxPower */
/* Interface 0, Alternate Setting 0, HID Class */
  USB_INTERFACE_DESC_SIZE,           /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
  0x00,                              /* bInterfaceNumber */
  0x00,                              /* bAlternateSetting */
  0x01,                              /* bNumEndpoints */
  USB_DEVICE_CLASS_HUMAN_INTERFACE,  /* bInterfaceClass */
  HID_SUBCLASS_NONE,                 /* bInterfaceSubClass */
  HID_PROTOCOL_NONE,                 /* bInterfaceProtocol */
  0x5E,                              /* iInterface */
/* HID Class Descriptor */
/* HID_DESC_OFFSET = 0x0012 */
  HID_DESC_SIZE,                     /* bLength */
  HID_HID_DESCRIPTOR_TYPE,           /* bDescriptorType */
  WBVAL(0x0100), /* 1.00 */          /* bcdHID */
  0x00,                              /* bCountryCode */
  0x01,                              /* bNumDescriptors */
  HID_REPORT_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(HID_REPORT_DESC_SIZE),       /* wDescriptorLength */
/* Endpoint, HID Interrupt In */
  USB_ENDPOINT_DESC_SIZE,            /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
  USB_ENDPOINT_IN(1),                /* bEndpointAddress */
  USB_ENDPOINT_TYPE_INTERRUPT,       /* bmAttributes */
  WBVAL(USB_PACKET_LEN),                     /* wMaxPacketSize */
  PoolingInterval,          				 /* bInterval */
/* Terminator */
  0                                  /* bLength */
};

/* USB String Descriptor (optional) */
BYTE USB_StringDescriptor[] = {
/* Index 0x00: LANGID Codes */
  0x04,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(0x0409), /* US English */    /* wLANGID */
/* Index 0x04: Manufacturer */
  28,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'P',0,
  'a',0,
  'v',0,
  'e',0,
  'l',0,
  ' ',0,
  'N',0,
  'a',0,
  'd',0,
  'e',0,
  'i',0,
  'n',0,
	 0, 0,
/* Index 0x20: Product */
  34,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'L',0,
  'E',0,
  'V',0,
  'E',0,
  'L',0,
  'I',0,
  'N',0,
  'G',0,
  ' ',0,
  'S',0,
  'Y',0,
  'S',0,
  'T',0,
  'E',0,
  'M',0,
	 0, 0,
/* Index 0x44: Serial Number */
  20,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
  ' ',0,
	 0, 0,
/* Index 0x56: Interface 0, Alternate Setting 0 */
  8,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'H',0,
  'I',0,
  'D',0,
	 0 ,0
};
