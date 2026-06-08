// taken from lk3rd for maestro9610 (only for 9610)
#ifndef __EXYNOS_USB_H__
#define __EXYNOS_USB_H__

#define SZ_1M                           0x00100000
#define SZ_4K							0x00001000

//==========================
// Define
//==========================
#define CONTROL_EP			0
#define BULK_IN_EP			1
#define BULK_OUT_EP			2
#define TOTL_EP_COUNT			16

#define USBDEV3_MDWIDTH			64	// master data bus width
#define USBDEV3_DATA_BUF_SIZ		16384	// 16KB

#define CMDCOMPLETEWAIT_UNIT		1000

#define RX_FIFO_SIZE			1024
#define NPTX_FIFO_START_ADDR		RX_FIFO_SIZE
#define NPTX_FIFO_SIZE			256
#define PTX_FIFO_SIZE			256

#define CTRL_BUF_SIZE			128		//512

// string descriptor
#define LANGID_US_L                 	(0x09)
#define LANGID_US_H                 	(0x04)

// Feature Selectors
#define EP_STALL          		0
#define DEVICE_REMOTE_WAKEUP		1
#define TEST_MODE			2

/* Test Mode Selector*/
#define TEST_J				1
#define TEST_K				2
#define TEST_SE0_NAK			3
#define TEST_PACKET			4
#define TEST_FORCE_ENABLE		5

#define USB_DEVICE			0
#define USB_HOST			1
#define USB_OTG				2

#define FULL_SPEED_CONTROL_PKT_SIZE	64
#define FULL_SPEED_BULK_PKT_SIZE	64

#define HIGH_SPEED_CONTROL_PKT_SIZE	64
#define HIGH_SPEED_BULK_PKT_SIZE	512

#define SUPER_SPEED_CONTROL_PKT_EXP_SZ	9	// 2^9 = 512
#define SUPER_SPEED_CONTROL_PKT_SIZE	512
#define SUPER_SPEED_BULK_PKT_SIZE	1024

/* usb transfer packet size in host fastboot tool */
#define HOST_TRANSFER_SIZE		SZ_1M

/////////////////////////////////////////////////
// Event Buffer Structures
//

#define USBDEV3_EVENT_BUFFER_COUNT	128	//256

#endif