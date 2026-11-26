/*
 * satl.c
 *
 * ATA over SCSI Translation Layer-specific code 
 *
 * Copyright (c) 2026 Peter Eriksson <pen@lysator.liu.se>
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/types.h>

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/scsi/scsi_all.h>
#include <camlib.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utils.h"
#include "satl.h"
#include "ata.h"


extern int f_debug;

#if 0
/*
 * SAT ATA PASS-THROUGH(16) opcode.
 */
#define SAT_ATA_PASS_16        0xA1
#define SAT_ATA_PASS_16        0x85
#endif

/*
 * ATA command.
 */
#define ATA_IDENTIFY_DEVICE    0xec

#define ATA_CMD_SMART                 0xb0

#define ATA_SMART_READ_DATA           0xd0
#define ATA_SMART_READ_LOG            0xd5
#define ATA_SMART_RETURN_STATUS       0xda

#define ATA_SMART_LBA                 0x00c24f00U

/*
 * ATA PASS-THROUGH protocol field, CDB byte 1 bits 4:1.
 *
 * Protocol 4 means PIO Data-In.
 */
#define SAT_PROTO_PIO_DATA_IN  4

/*
 * Values for the T_LENGTH field in CDB byte 2.
 */
#define SAT_TLEN_NONE          0
#define SAT_TLEN_FEATURES      1
#define SAT_TLEN_SECTOR_COUNT  2
#define SAT_TLEN_STPSIU        3

#define IDENTIFY_LENGTH        512
#define SAT_TIMEOUT_MS         30000

#define SMART_DATA_SIZE 512

/*
 * Decode an ATA IDENTIFY string.
 *
 * ATA stores text as byte-swapped 16-bit words:
 *
 *     word bytes: 'A', 'B'
 *     memory order after transfer often appears as 'A', 'B',
 *     but the defined string representation requires swapping
 *     each word's two bytes.
 */
static void
ata_string(const uint8_t *identify, unsigned first_word,
    unsigned word_count, char *output, size_t output_size)
{
    size_t source_offset;
    size_t length;
    size_t i;

    if (output_size == 0)
        return;

    source_offset = (size_t)first_word * 2;
    length = (size_t)word_count * 2;

    if (length >= output_size)
        length = output_size - 1;

    for (i = 0; i + 1 < length; i += 2) {
        output[i] = (char)identify[source_offset + i + 1];
        output[i + 1] = (char)identify[source_offset + i];
    }

    /*
     * Handle an odd-sized output buffer, although ATA strings
     * normally contain an even number of bytes.
     */
    if ((length & 1) != 0)
        output[length - 1] =
            (char)identify[source_offset + length - 2];

    output[length] = '\0';

    /*
     * Remove trailing spaces and NUL bytes.
     */
    while (length > 0 &&
        (output[length - 1] == ' ' || output[length - 1] == '\0')) {
        output[length - 1] = '\0';
        length--;
    }
}

static uint16_t
ata_word(const uint8_t *identify, unsigned word_number)
{
    size_t offset;

    offset = (size_t)word_number * 2;

    /*
     * ATA IDENTIFY numeric words are little-endian.
     */
    return ((uint16_t)identify[offset] |
        ((uint16_t)identify[offset + 1] << 8));
}

static void
print_cam_error(struct cam_device *dev,
		union ccb *ccb)
{
    cam_status status;
    char buf[1024];
    
    status = ccb->ccb_h.status & CAM_STATUS_MASK;

    fprintf(stderr, "%s%u: CAM status 0x%x: %s\n",
	    dev->given_dev_name,
	    dev->given_unit_number,
	    status,
	    cam_error_string(dev, ccb, buf, sizeof(buf),
			     CAM_ESF_ALL, CAM_EPF_ALL));
}


static void
print_identify(const uint8_t *identify) {
    char serial[21];
    char firmware[9];
    char model[41];
    uint16_t word0;

    /*
     * Standard ATA IDENTIFY string locations:
     *
     * Words 10-19: serial number, 20 bytes
     * Words 23-26: firmware revision, 8 bytes
     * Words 27-46: model number, 40 bytes
     */
    ata_string(identify, 10, 10, serial, sizeof(serial));
    ata_string(identify, 23, 4, firmware, sizeof(firmware));
    ata_string(identify, 27, 20, model, sizeof(model));

    word0 = ata_word(identify, 0);

    printf("\tATA IDENTIFY succeeded\n");
    printf("\tModel:          %s\n", model);
    printf("\tSerial:         %s\n", serial);
    printf("\tFirmware:       %s\n", firmware);
    printf("\tWord 0:         0x%04x\n", word0);

    /*
     * ATA IDENTIFY word 0 bit 15:
     *
     * 0 = ATA device
     * 1 = ATAPI/device using IDENTIFY PACKET DEVICE format
     *
     * Since this example sends IDENTIFY DEVICE, a normal SATA disk
     * should normally have bit 15 clear.
     */
    printf("\tDevice class:   %s\n",
        (word0 & 0x8000) != 0 ? "ATAPI/packet" : "ATA");
}


int
drive_satl_command(DRIVE *dp,
		   uint8_t command,
		   uint8_t code,
		   uint32_t lba,
		   uint8_t *buffer,
		   size_t bufsize) {
    union ccb *ccb;
    struct ccb_scsiio *csio;
    uint8_t *cdb;
    cam_status status;
    uint8_t cdb_size = 16;
    ssize_t result;

 Retry:
    if (f_debug)
	fprintf(stderr, "  [drive_satl_command(0x%02x, 0x%02x, 0x%04x, ..., %zu), CDB Size=%d]\n", command, code, lba, bufsize, cdb_size);
    
    result = -1;
    ccb = cam_getccb(dp->cam);
    if (ccb == NULL) {
        fprintf(stderr, "%s: drive_satl_command: cam_getccb failed: %s\n",
            dp->cam->device_path, strerror(errno));
        return -1;
    }

    csio = &ccb->csio;

    /*
     * Fill the SCSI I/O CCB.
     *
     * Use CAM_TAG_ACTION_NONE because this is a management command
     * being sent through a USB bridge. Tagged queuing is unnecessary,
     * and many USB bridges do not benefit from it.
     *
     * CAM_DIR_IN means data is transferred from the device.
     */
    cam_fill_csio(csio,
		  1,                      /* retry count */
		  NULL,                   /* completion callback */
		  CAM_DIR_IN,             /* data direction */
		  CAM_TAG_ACTION_NONE,    /* tag action */
		  buffer,                 /* data buffer */
		  bufsize,                /* transfer length */
		  SSD_FULL_SIZE,          /* sense buffer size */
		  cdb_size,               /* CDB length */
		  SAT_TIMEOUT_MS);        /* timeout in milliseconds */
    
    cdb = csio->cdb_io.cdb_bytes;
    memset(cdb, 0, cdb_size);

    
    switch (cdb_size) {
    case 12:
	/*
	 * ATA PASS-THROUGH(12) register layout:
	 *
	 * Byte 0   OPERATION CODE (0xA1)
	 * Byte 1   Bit 7-5 MULTIPLE_COUNT, Bit 4-1 PROTOCOL, Bit 0 Reserved
	 * Byte 2   Bit 7-6 OFF_LINE, Bit 5 CK_COND, Bit 4 Reserved, Bit 3 T_DIR, Bit 2 BYTE_BLOCK, Bit 1-0 T_LENGTH
	 * Byte 3   Features
	 * Byte 4   SECTOR_COUNT
	 * Byte 5   LBA low
	 * Byte 6   LBA mid
	 * Byte 7   LBA high
	 * Byte 8   Device
	 * Byte 9   Command
	 * Byte 10  Reserved
	 * Byte 11  Reserved1
	 */
	cdb[0] = ATA_PASS_12;
	cdb[1] = (uint8_t)(SAT_PROTO_PIO_DATA_IN << 1);
	cdb[2] =
	    (1U << 3) |             /* T_DIR: device to host */
	    (1U << 2) |             /* BYT_BLOK: blocks */
	    SAT_TLEN_SECTOR_COUNT;  /* length from sector count */

	cdb[3] = code;
	cdb[4] = 1;                 /* one 512-byte sector */

	cdb[5] = lba&0xff;
	cdb[6] = (lba>>8)&0xff;
	cdb[7] = (lba>>16)&0xff;

	cdb[8] = 0;                 /* device register */
	cdb[9] = command;
	break;

    case 16:
	/*
	 * ATA PASS-THROUGH(16) register layout:
	 *
	 * Byte 0   Operation Code
	 * Byte 1   Bit 7-5 Multiple Count, Bit 4-2 Off.Line, Bit 1 SRST, Bit 0 Extend=0
	 * Byte 2   Bit 7-6 Reserved, Bit 5 Dir, Bit 4-0 Protocol 
	 * Byte 3   Bit 7 HWRST, Bit 6 DMA, Bit 5 D.En, Bit 4 LH.En, Bit 3 LM.En, Bit 2 LL.En, Bit 1 SC.En, Bit 0 F.En
	 * Byte 4   Features
	 * Byte 5   Sector count, high
	 * Byte 6   Sector count, low
	 * Byte 7   LBA low, high
	 * Byte 8   LBA low, low
	 * Byte 9   LBA mid, high
	 * Byte 10  LBA mid, low
	 * Byte 11  LBA high, high
	 * Byte 12  LBA high, low
	 * Byte 13  Device
	 * Byte 14  Command
	 * Byte 15  Control
	 */
	cdb[0] = ATA_PASS_16;
	cdb[1] = (uint8_t)(SAT_PROTO_PIO_DATA_IN << 1);
	cdb[2] =
	    (1U << 3) |             /* T_DIR: device to host */
	    (1U << 2) |             /* BYT_BLOK: blocks */
	    SAT_TLEN_SECTOR_COUNT;  /* length from sector count */

	cdb[4] = code;
	cdb[5] = 0;
	cdb[6] = 1;                 /* one 512-byte sector */
	cdb[7] = 0x00;
	cdb[8] = (lba&0xff);
	cdb[9] = 0x00;
	cdb[10] = (lba>>8)&0xff;
	cdb[11] = 0x00;
	cdb[12] = (lba>>16)&0xff;
	cdb[13] = 0;                /* device register */
	cdb[14] = command;
	cdb[15] = 0;                /* ATA control register */
    }
    
	
    if (cam_send_ccb(dp->cam, ccb) < 0) {
	fprintf(stderr, "%s%u: cam_send_ccb failed: %s\n",
		dp->cam->given_dev_name,
		dp->cam->given_unit_number,
		strerror(errno));
	result = -1;
	goto out;
    }

    status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (status != CAM_REQ_CMP) {
	if (f_debug)
	    print_cam_error(dp->cam, ccb);
	result = -1;
        goto out;
    }

    result = bufsize - csio->resid;
    if (csio->resid != 0) {
        fprintf(stderr,
            "%s%u: NOTICE: short SATL transfer: "
            "%zu of %zu bytes transferred\n",
		dp->cam->given_dev_name,
		dp->cam->given_unit_number,
	        result,
		bufsize);
        goto out;
    }

out:
    cam_freeccb(ccb);
    
    if (result < 0 && cdb_size == 16) {
	cdb_size = 12;
	goto Retry;
    }
    
    return result;
}

/*
 * Send ATA IDENTIFY DEVICE through SAT ATA PASS-THROUGH(16).
 *
 * Return:
 *      0  success
 *     -1  failure
 */
int
satl_identify(DRIVE *dp) {
    uint8_t identify[IDENTIFY_LENGTH];
#if 0
    union ccb *ccb;
    struct ccb_scsiio *csio;
    uint8_t *cdb;
    cam_status status;
    uint8_t cdb_size = 16;
#endif
    int result;

    memset(identify, 0, IDENTIFY_LENGTH);

#if 0
 Retry:
    if (f_debug)
	fprintf(stderr, "  [satl_identify]\n");
    result = -1;
    ccb = cam_getccb(dp->cam);
    if (ccb == NULL) {
        fprintf(stderr, "%s: sat_ata_identify: cam_getccb failed: %s\n",
            dp->cam->device_path, strerror(errno));
        return -1;
    }

    csio = &ccb->csio;

    /*
     * Fill the SCSI I/O CCB.
     *
     * Use CAM_TAG_ACTION_NONE because this is a management command
     * being sent through a USB bridge. Tagged queuing is unnecessary,
     * and many USB bridges do not benefit from it.
     *
     * CAM_DIR_IN means data is transferred from the device.
     */
    cam_fill_csio(csio,
		  1,                      /* retry count */
		  NULL,                   /* completion callback */
		  CAM_DIR_IN,             /* data direction */
		  CAM_TAG_ACTION_NONE,    /* tag action */
		  identify,               /* data buffer */
		  IDENTIFY_LENGTH,        /* transfer length */
		  SSD_FULL_SIZE,          /* sense buffer size */
		  cdb_size,               /* CDB length */
		  SAT_TIMEOUT_MS);        /* timeout in milliseconds */
    
    cdb = csio->cdb_io.cdb_bytes;
    memset(cdb, 0, cdb_size);

    /*
     * ATA PASS-THROUGH(16), SAT:
     *
     * Byte 0:
     *     0x85 = ATA PASS-THROUGH(16)
     *
     * Byte 1:
     *     bit 0       EXTEND = 0
     *     bits 4:1    PROTOCOL = 4, PIO Data-In
     *     bits 7:5    MULTIPLE_COUNT = 0
     *
     * Byte 2:
     *     bits 1:0    T_LENGTH = 2
     *                  transfer length comes from sector count
     *     bit 2       BYT_BLOK = 1
     *                  sector count is in 512-byte blocks
     *     bit 3       T_DIR = 1
     *                  data-in from ATA device
     *     bit 5       CK_COND = 0
     *
     * Byte 6:
     *     ATA sector count low = 1 sector = 512 bytes
     *
     * Byte 14:
     *     ATA command = IDENTIFY DEVICE, 0xec
     */

    
    switch (cdb_size) {
    case 12:
	/*
	 * ATA PASS-THROUGH(12) register layout:
	 *
	 * Byte 0   OPERATION CODE (0xA1)
	 * Byte 1   Bit 7-5 MULTIPLE_COUNT, Bit 4-1 PROTOCOL, Bit 0 Reserved
	 * Byte 2   Bit 7-6 OFF_LINE, Bit 5 CK_COND, Bit 4 Reserved, Bit 3 T_DIR, Bit 2 BYTE_BLOCK, Bit 1-0 T_LENGTH
	 * Byte 3   Features
	 * Byte 4   SECTOR_COUNT
	 * Byte 5   LBA low
	 * Byte 6   LBA mid
	 * Byte 7   LBA high
	 * Byte 8   Device
	 * Byte 9   Command
	 * Byte 10  Reserved
	 * Byte 11  Reserved1
	 */
	cdb[0] = ATA_PASS_12;
	cdb[1] = (uint8_t)(SAT_PROTO_PIO_DATA_IN << 1);
	cdb[2] =
	    (1U << 3) |             /* T_DIR: device to host */
	    (1U << 2) |             /* BYT_BLOK: blocks */
	    SAT_TLEN_SECTOR_COUNT;  /* length from sector count */
    
	cdb[4] = 1;                 /* one 512-byte sector */
	cdb[8] = 0;                /* device register */
	cdb[9] = ATA_IDENTIFY_DEVICE;
	break;

    case 16:
	/*
	 * ATA PASS-THROUGH(16) register layout:
	 *
	 * Byte 0   Operation Code
	 * Byte 1   Bit 7-5 Multiple Count, Bit 4-2 Off.Line, Bit 1 SRST, Bit 0 Extend=0
	 * Byte 2   Bit 7-6 Reserved, Bit 5 Dir, Bit 4-0 Protocol 
	 * Byte 3   Bit 7 HWRST, Bit 6 DMA, Bit 5 D.En, Bit 4 LH.En, Bit 3 LM.En, Bit 2 LL.En, Bit 1 SC.En, Bit 0 F.En
	 * Byte 4   Features
	 * Byte 5   Sector count, high
	 * Byte 6   Sector count, low
	 * Byte 7   LBA low, high
	 * Byte 8   LBA low, low
	 * Byte 9   LBA mid, high
	 * Byte 10  LBA mid, low
	 * Byte 11  LBA high, high
	 * Byte 12  LBA high, low
	 * Byte 13  Device
	 * Byte 14  Command
	 * Byte 15  Control
	 */
	cdb[0] = ATA_PASS_16;
	cdb[1] = (uint8_t)(SAT_PROTO_PIO_DATA_IN << 1);
	cdb[2] =
	    (1U << 3) |             /* T_DIR: device to host */
	    (1U << 2) |             /* BYT_BLOK: blocks */
	    SAT_TLEN_SECTOR_COUNT;  /* length from sector count */
	
	cdb[6] = 1;                 /* one 512-byte sector */
	cdb[13] = 0;                /* device register */
	cdb[14] = ATA_IDENTIFY_DEVICE;
	cdb[15] = 0;                /* ATA control register */
    }
    
	
    if (cam_send_ccb(dp->cam, ccb) < 0) {
	fprintf(stderr, "%s%u: cam_send_ccb failed: %s\n",
		dp->cam->given_dev_name,
		dp->cam->given_unit_number,
		strerror(errno));
	goto out;
    }

    status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (status != CAM_REQ_CMP) {
	if (f_debug)
	    print_cam_error(dp->cam, ccb);
        goto out;
    }

    if (csio->resid != 0) {
        fprintf(stderr,
            "%s%u: short ATA IDENTIFY transfer: "
            "%u of %u bytes transferred\n",
		dp->cam->given_dev_name,
		dp->cam->given_unit_number,
		IDENTIFY_LENGTH - csio->resid,
		IDENTIFY_LENGTH);
        goto out;
    }

    result = 0;

out:
    cam_freeccb(ccb);
#endif

    result = drive_satl_command(dp, ATA_IDENTIFY_DEVICE, 0, 0,
				(uint8_t *) &identify, sizeof(identify));
    if (result < 0)
	return -1;

    if (result != sizeof(identify)) {
	fprintf(stderr, "** SATL SATA IDENTIFY too small returned data\n");
	return -1;
    }
    
    if (1) {
	char serial[21];
	char firmware[9];
	char model[41];
#if 0
	uint16_t word0;
	
	word0 = ata_word(identify, 0);
#endif	
	/*
	 * Standard ATA IDENTIFY string locations:
	 *
	 * Words 10-19: serial number, 20 bytes
	 * Words 23-26: firmware revision, 8 bytes
	 * Words 27-46: model number, 40 bytes
	 */
#if 0
	if (word0 & 0x8000)
	    dp->raw.proto = DRIVE_PROTO_ATAPI;
#endif
	    
	ata_string(identify, 10, 10, serial, sizeof(serial));
	ata_string(identify, 23, 4, firmware, sizeof(firmware));
	ata_string(identify, 27, 20, model, sizeof(model));

#if 0
	fprintf(stderr, "VENDOR = %s\n", dp->vendor ? dp->vendor : "<null>");
	fprintf(stderr, "PHYS   = %s\n", dp->phys ? dp->phys : "<null>");
	fprintf(stderr, "INQ_VENDOR = %.*s\n",
		(int) sizeof(dp->cam->inq_data.vendor),
		dp->cam->inq_data.vendor);
#endif
#if 0	
	if (!dp->phys && dp->cam->inq_data.vendor[0])
	    dp->phys = strdupget(dp->cam->inq_data.vendor, sizeof(dp->cam->inq_data.vendor));
#endif
	if (dp->vendor)
	    free(dp->vendor);
	dp->vendor = strdup("ATA");
	if (dp->product)
	    free(dp->product);
	dp->product = strdup(model);
	if (dp->version)
	    free(dp->version);
	dp->version = strdup(firmware);

	/*
	 * We override and previously detected ident/serial since it might have
	 * been truncated by the normal SATL SCSI emulation.
	 */
	if (dp->ident)
	    free(dp->ident);
	dp->ident = strdup(serial);
	
	if (f_debug)
	    print_identify(identify);
    }

#if 0
    else {	
	if (cdb_size == 16) {
	    cdb_size = 12;
	    goto Retry;
	}
    }
#endif
    
    return result;
}



int
drive_satl_smart_data(DRIVE *dp) {
    uint8_t data[SMART_DATA_SIZE];
    ssize_t len;


    len = drive_satl_command(dp, ATA_CMD_SMART, ATA_SMART_READ_DATA, ATA_SMART_LBA, data, sizeof(data));
    if (len < 0)
	return len;
    if (len != SMART_DATA_SIZE) {
	errno = EIO;
	return -1;
    }

    return drive_ata_parse_smart_data(dp, data, sizeof(data));
}


 

int
satl_health_status(DRIVE *dp) {
    int rc1 = 0, rc2 = 0, rc3 = 0;


    if (dp->flags.have_smart) {
	if (f_debug)
	    fprintf(stderr, "  [temperature & power_on_hours]\n");
	rc1 = drive_satl_smart_data(dp);
	if (rc1 < 0)
	    return rc1;
	
	if (f_debug)
	    fprintf(stderr, "  [smart status]\n");
#if 0	
	rc2 = drive_satl_smart_status(dp);
#else
	rc2 = 0;
#endif
    }
    if (dp->flags.have_selftest) {
	if (f_debug)
	    fprintf(stderr, "  [selftest]\n");
#if 0
	drive_satl_smart_log_directory(dp);
	rc3 = drive_satl_smart_log(dp);
#else
	rc3 = 0;
#endif
    }

    if (rc1 > 0 || rc2 > 0 || rc3 > 0)
	return 1;
    if (rc1 == 0 && rc2 == 0 && rc3 == 0)
	return 0;
    
    return -1;
}
