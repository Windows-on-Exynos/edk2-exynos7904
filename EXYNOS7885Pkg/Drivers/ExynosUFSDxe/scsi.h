/*
 * (C) Copyright 2017 SAMSUNG Electronics
 * Kiwoong Kim <kwmad.kim@samsung.com>
 *
 * This software is proprietary of Samsung Electronics.
 * No part of this software, either material or conceptual may be copied or distributed, transmitted,
 * transcribed, stored in a retrieval system or translated into any human or computer language in any form by any means,
 * electronic, mechanical, manual or otherwise, or disclosed
 * to third parties without the express written permission of Samsung Electronics.
 *
 */

#ifndef __SCSI_DEVICE_H__
#define __SCSI_DEVICE_H__

// Define list_node if not already defined
typedef struct list_node {
    struct list_node *next;
    struct list_node *prev;
} list_node;

// Define bnum_t if not already defined
typedef UINTN bnum_t;

typedef struct scsi_command_meta scm;
typedef struct scsi_device_s scsi_device_t;

typedef EFI_STATUS (exec_t)(scm *);
typedef scsi_device_t *(get_sdev_t)(void);

struct scsi_device_s {
    EFI_BLOCK_IO_PROTOCOL *block_io; // Reemplaza bdev_t con EFI_BLOCK_IO_PROTOCOL
    list_node lu_node;
    UINT32 lun;
    char vendor[40+1+3];
    char product[20+1+3];
    char revision[8+1+3];
    exec_t* exec;
    get_sdev_t* get_ssu_sdev;
};

/*
 * @cdb: command descriptor block
 * @lun: logical unit number
 * @datalen: data length in byte for CDB and COMMAND UPIU(UFS)
 */

struct scsi_command_meta {
    UINT8 cdb[16];
    UINT32 datalen;
    UINT8 *buf;
    UINT8 sense_buf[64];
    UINT8 status;

    scsi_device_t *sdev;
};

enum scsi_opcode {
    SCSI_OP_TEST_UNIT_READY       = 0x00,
    SCSI_OP_READ_CAPACITY_10      = 0x25,
    SCSI_OP_READ_10               = 0x28,
    SCSI_OP_WRITE_10              = 0x2A,
    SCSI_OP_SECU_PROT_IN          = 0xA2,
    SCSI_OP_SECU_PROT_OUT         = 0xB5,
    SCSI_OP_INQUIRY               = 0x12,
    SCSI_OP_WRITE_BUFFER          = 0x3B,
    SCSI_OP_REQUEST_SENSE         = 0x03,
    SCSI_OP_FORMAT_UNIT           = 0x04,
    SCSI_OP_START_STOP_UNIT       = 0x1B,
    SCSI_OP_UNMAP                 = 0x42,
    SCSI_MODE_SEL10               = 0x55,
    SCSI_MODE_SEN10               = 0x5A,
};

/* External Functions */
EFI_STATUS scsi_scan(scsi_device_t *sdev, UINT32 wlun, UINT32 dev_num, exec_t *func,
            const char *name_s, bnum_t max_seg, list_node *list);
EFI_STATUS scsi_scan_ssu(scsi_device_t *sdev, UINT32 wlun,
            exec_t *func, get_sdev_t *func1, list_node *lu_list);
EFI_STATUS scsi_do_ssu(void);
EFI_STATUS scsi_ufs_ffu(const void *buf, UINT32 len);
void scsi_exit(list_node *lu_node, const char *prefix);

struct uic_pwr_mode {
	UINT8 lane;
	UINT8 gear;
	UINT8 mode;
	UINT8 hs_series;
};

#endif /* __SCSI_DEVICE_H__ */
