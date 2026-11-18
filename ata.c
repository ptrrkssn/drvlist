/*
 * ata.c
 *
 * Code to access SATA (ada) drives
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
#include <fcntl.h>
#include <errno.h>

#include <camlib.h>
#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/ata/ata_all.h>
#include <cam/scsi/scsi_message.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "utils.h"
#include "ata.h"

extern int f_debug;

/*
 * ATA SMART commands.
 *
 * Local definitions avoid depending on the exact names exported by
 * <sys/ata.h> in a particular FreeBSD source version.
 */
#define ATA_CMD_SMART                 0xB0
#define ATA_SMART_READ_DATA           0xD0
#define ATA_SMART_RETURN_STATUS       0xDA

#define ATA_SMART_LBA_GOOD            0x00C24F00U
#define ATA_SMART_LBA_FAILED          0x002CF400U

#define ATA_SMART_READ_LOG            0xd5
#define ATA_SELF_TEST_LOG_ADDRESS     0x06
#define ATA_SMART_READ_LOG_LBA        0x00c24f06U

#define ATA_SMART_LOG_SIZE            512
#define ATA_SELF_TEST_ENTRY_SIZE      24
#define ATA_SELF_TEST_ENTRY_COUNT     21
#define ATA_SELF_TEST_ENTRY_OFFSET    2
#define ATA_SELF_TEST_INDEX_OFFSET    508


/*
 * SMART commands require these values in the LBA mid and LBA high
 * registers:
 *
 *     LBA low  = 0x00
 *     LBA mid  = 0x4f
 *     LBA high = 0xc2
 *
 * ata_28bit_cmd() accepts them as a 28-bit LBA value.
 */
#define ATA_SMART_LBA                   0x00c24f00U

#define SMART_DATA_SIZE                 512
#define SMART_ATTRIBUTE_OFFSET          2
#define SMART_ATTRIBUTE_SIZE            12
#define SMART_ATTRIBUTE_COUNT           30

#define SMART_ATTR_AIRFLOW_TEMPERATURE  190
#define SMART_ATTR_TEMPERATURE_CELSIUS  194

struct ata_smart_attribute {
    uint8_t id;
    uint8_t flags[2];
    uint8_t current;
    uint8_t worst;
    uint8_t raw[6];
    uint8_t reserved;
};

static int
ata_smart_checksum_valid(const uint8_t *buffer, size_t length)
{
    unsigned int sum;
    size_t i;

    sum = 0;

    for (i = 0; i < length; i++)
        sum += buffer[i];

    return ((sum & 0xffU) == 0);
}



/*
 * Extract a plausible temperature from a SMART attribute.
 *
 * For attributes 190 and 194, most drives store the current Celsius
 * value in raw byte zero. Some vendors put minimum and maximum values
 * in later bytes, so the whole raw field must not be interpreted as a
 * single 48-bit temperature.
 */
static int
ata_smart_attribute_temperature(const struct ata_smart_attribute *attr,
				uint16_t *temp) {
    unsigned int value;


    value = attr->raw[0];

    /*
     * Reject obvious unavailable or nonsensical values.
     *
     * The upper limit is intentionally generous. Drives normally
     * operate below roughly 70 C, but rejecting everything above that
     * could hide a genuinely overheating device.
     */
    if (value == 0 || value == 0xff || value > 125)
        return -1;

    *temp = value;
    return 0;
}


int
ata_cmd_smart(struct cam_device *dev,
	      uint8_t code,
	      uint8_t *buf,
	      size_t bufsize) {
    union ccb *ccb;
    uint32_t cam_status;
    int saved_errno;
    size_t got;
    

    if (!dev || !buf) {
        errno = EINVAL;
        return -1;
    }

    memset(buf, 0, bufsize);

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        return -1;
    }

    /*
     * Build an ATA data-in request.
     *
     * ATA_SMART_READ_DATA transfers one 512-byte sector from the
     * device to the host.
     */
    cam_fill_ataio(&ccb->ataio,
        1,                      /* retries */
        NULL,                   /* completion callback */
        CAM_DIR_IN,             /* device to host */
        CAM_TAG_ACTION_NONE,    /* non-NCQ management command */
        buf,
        bufsize,
        5000);                  /* timeout in milliseconds */

    /*
     * Construct:
     *
     *     command       = SMART (0xb0)
     *     features      = SMART READ DATA (0xd0)
     *     sector count  = 1
     *     LBA low       = 0x00
     *     LBA mid       = 0x4f
     *     LBA high      = 0xc2
     */
    ata_28bit_cmd(&ccb->ataio,
		  ATA_CMD_SMART,
		  code,
		  ATA_SMART_LBA,
		  1);

    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;
        cam_freeccb(ccb);
        errno = saved_errno;
        return -1;
    }

    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (cam_status != CAM_REQ_CMP) {
        cam_freeccb(ccb);
        errno = EIO;
        return -1;
    }

    if (ccb->ataio.resid != 0) {
        cam_freeccb(ccb);
        errno = EIO;
        return -1;
    }

    got = bufsize - ccb->ataio.resid;
    cam_freeccb(ccb);
    
    return got;
}


int
ata_drive_temperature(struct cam_device *dev,
		      DRIVE *dp) {
    uint8_t data[SMART_DATA_SIZE];
    ssize_t len;
    uint16_t fallback_temperature;
    size_t offset, i;
    int have_fallback;


    len = ata_cmd_smart(dev, ATA_SMART_READ_DATA, data, sizeof(data));
    if (len <= 0)
	return len;
    if (len != SMART_DATA_SIZE) {
	errno = EIO;
	return -1;
    }
    
    /*
     * Prefer attribute 194, Temperature Celsius.
     *
     * Keep attribute 190, Airflow Temperature, as a fallback because
     * some drives expose only 190.
     */
    have_fallback = 0;
    fallback_temperature = 0.0;

    for (i = 0; i < SMART_ATTRIBUTE_COUNT; i++) {
	struct ata_smart_attribute *attr;
	
        offset = SMART_ATTRIBUTE_OFFSET + i * SMART_ATTRIBUTE_SIZE;
	
        attr = (struct ata_smart_attribute *) &data[offset];
        if (attr->id == 0)
            continue;
	
        if (attr->id == SMART_ATTR_TEMPERATURE_CELSIUS) {
            if (ata_smart_attribute_temperature(attr, &dp->raw.temperature) == 0) {
		if (f_debug)
		    fprintf(stderr, "\tATA SMART DATA: drive temperature=%u\n", dp->raw.temperature);
                return 1;
	    }
        }
	
        if (attr->id == SMART_ATTR_AIRFLOW_TEMPERATURE &&
            !have_fallback) {
            if (ata_smart_attribute_temperature(attr, &fallback_temperature) == 0) {
		if (f_debug)
		    fprintf(stderr, "\tATA SMART DATA: airflow temperature=%u\n", fallback_temperature);
                have_fallback = 1;
	    }
        }
    }
    
    if (have_fallback) {
        dp->raw.temperature = fallback_temperature;
        return 1;
    }
    
    return 0;
}


 



static int
ata_cam_send(struct cam_device *device, union ccb *ccb)
{
	/* Disable freezing the device queue */
	ccb->ccb_h.flags |= CAM_DEV_QFRZDIS;


	if (cam_send_ccb(device, ccb) < 0) {
		return (1);
	}

	/*
	 * Consider any non-CAM_REQ_CMP status as error and report it here,
	 * unless caller set AP_FLAG_CHK_COND, in which case it is responsible.
	 */
	if (!(ccb->ataio.cmd.flags & CAM_ATAIO_NEEDRESULT) &&
	    (ccb->ccb_h.status & CAM_STATUS_MASK) != CAM_REQ_CMP) {
		return (1);
	}

	return (0);
}


static int
ata_do_cmd(struct cam_device *device, union ccb *ccb, int retries,
	   uint32_t flags, uint8_t protocol, uint8_t ata_flags,
	   uint8_t tag_action, uint8_t command, uint16_t features,
	   u_int64_t lba, uint16_t sector_count, uint8_t *data_ptr,
	   uint16_t dxfer_len, int timeout, int force48bit)
{
	CCB_CLEAR_ALL_EXCEPT_HDR(&ccb->ataio);
	cam_fill_ataio(&ccb->ataio,
		       retries,
		       NULL,
		       flags,
		       tag_action,
		       data_ptr,
		       dxfer_len,
		       timeout);

	if (force48bit || lba > ATA_MAX_28BIT_LBA)
		ata_48bit_cmd(&ccb->ataio, command, features, lba, sector_count);
	else
		ata_28bit_cmd(&ccb->ataio, command, features, lba, sector_count);

	if (ata_flags & AP_FLAG_CHK_COND)
		ccb->ataio.cmd.flags |= CAM_ATAIO_NEEDRESULT;

	return ata_cam_send(device, ccb);
}

int
ata_identify(struct cam_device *cdb,
	     char **vendor,
	     char **model,
	     char **revision,
	     int *have_selftest) {
    union ccb *ccb;
    struct ata_params apb;
    uint8_t *bp;
    u_int i, error;
    uint8_t command, retry_command;
    
    
    if ((ccb = cam_getccb(cdb)) == NULL) {
	return -1;
    }

    command = ATA_ATA_IDENTIFY;
    retry_command = ATA_ATAPI_IDENTIFY;
    
 retry:
    error = ata_do_cmd(cdb,
		       ccb,
		       1, /*retries*/
		       CAM_DIR_IN, /*flags*/
		       AP_PROTO_PIO_IN, /*protocol*/
		       AP_FLAG_BYT_BLOK_BLOCKS | AP_FLAG_TLEN_SECT_CNT, /*ata_flags*/
#if 0
		       MSG_SIMPLE_Q_TAG, /*tag_action*/
#else
		       CAM_TAG_ACTION_NONE,
#endif
		       command, /*command*/
		       0, /*features*/
		       0, /*lba*/
		       sizeof(struct ata_params) / 512, /*sector_count*/
		       (uint8_t *)&apb, /*data_ptr*/
		       sizeof(apb), /*dxfer_len*/
		       30000, /* timeout */
		       0 /*force48bit*/);
    
    if (error != 0) {
	if (retry_command != 0) {
	    command = retry_command;
	    retry_command = 0;
	    goto retry;
	}
	return (1);
    }
    
    ata_param_fixup(&apb);
    
    error = 1;
    bp = (uint8_t *) &apb;
    for (i = 0; i < sizeof(apb); i++) {
	if (bp[i] != 0)
	    error = 0;
    }
    
    /* check for invalid (all zero) response */
    if (error != 0) {
	return (error);
    }
    
    cam_freeccb(ccb);
    
    *vendor = strdup("ATA");
    *model = strndup((char *) apb.model, sizeof(apb.model));
    *revision = strndup((char *) apb.revision, sizeof(apb.revision));
    
    return 0;
}



/*
 * ATA/SATA: SMART RETURN STATUS
 *
 * The command returns one of these cylinder/LBA register pairs:
 *
 *     LBA mid = 0x4f, LBA high = 0xc2: thresholds not exceeded
 *     LBA mid = 0xf4, LBA high = 0x2c: threshold exceeded
 */
int
ata_smart_status(struct cam_device *dev,
		 DRIVE *dp) {
    union ccb *ccb;
    uint32_t cam_status;
    uint8_t lba_mid;
    uint8_t lba_high;
    int saved_errno;
    int rc = 0;
    

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        errno = ENOMEM;
        return (-1);
    }

    /*
     * CAM_ATAIO_NEEDRESULT must be part of the flags supplied to
     * cam_fill_ataio(). It requests the final ATA task-file registers.
     */
    cam_fill_ataio(&ccb->ataio,
        1,                                      /* retries */
        NULL,                                   /* callback */
        CAM_DIR_NONE | CAM_ATAIO_NEEDRESULT,
        CAM_TAG_ACTION_NONE,
        NULL,                                   /* no data transfer */
        0,
        5000);                                  /* timeout, ms */

    /*
     * ATA SMART RETURN STATUS:
     *
     * command       = 0xb0
     * features      = 0xda
     * sector count  = 0
     * LBA low       = 0x00
     * LBA mid       = 0x4f
     * LBA high      = 0xc2
     */
    ata_28bit_cmd(&ccb->ataio,
        ATA_CMD_SMART,
        ATA_SMART_RETURN_STATUS,
        ATA_SMART_LBA_GOOD,
        0);

    ccb->ataio.cmd.flags |= CAM_ATAIO_NEEDRESULT;
    
    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;

        fprintf(stderr, "%s%u: cam_send_ccb: %s\n",
            dev->given_dev_name,
            dev->given_unit_number,
            strerror(saved_errno));

        cam_freeccb(ccb);
        errno = saved_errno;
        return (-1);
    }

    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (cam_status != CAM_REQ_CMP) {
        fprintf(stderr,
            "%s%u: SMART RETURN STATUS failed: "
            "CAM status 0x%x, ATA status 0x%02x, error 0x%02x\n",
            dev->given_dev_name,
            dev->given_unit_number,
            cam_status,
            ccb->ataio.res.status,
            ccb->ataio.res.error);

        cam_freeccb(ccb);
        errno = EIO;
        return (-1);
    }

    /*
     * SMART RETURN STATUS changes LBA mid/high to one of:
     *
     *     4f c2: threshold not exceeded
     *     f4 2c: threshold exceeded
     */
    lba_mid = ccb->ataio.res.lba_mid;
    lba_high = ccb->ataio.res.lba_high;

    cam_freeccb(ccb);

    if (lba_mid == 0x4f && lba_high == 0xc2) {
        dp->raw.smart_status = DRIVE_SMART_PASSED;
	rc = 1;
    } else if (lba_mid == 0xf4 && lba_high == 0x2c) {
        dp->raw.smart_status = DRIVE_SMART_FAILED;
	rc = 1;
    } else {
	fprintf(stderr,
		"%s%u: unexpected SMART status registers: "
		"LBA mid=0x%02x, LBA high=0x%02x\n",
		dev->given_dev_name,
		dev->given_unit_number,
		lba_mid,
		lba_high);
	
	errno = EIO;
	return (-1);
    }

    if (f_debug)
	fprintf(stderr, "\tATA SMART STATUS: status=%s\n", drive_smart_status2str(dp->raw.smart_status));
    
    return rc;
}


static enum drive_selftest_status
classify_ata_selftest(uint8_t status)
{
    switch (status) {
    case 0x0:
        return (DRIVE_SELFTEST_PASSED);

    case 0x1:
    case 0x2:
        return (DRIVE_SELFTEST_ABORTED);

    case 0x3:
    case 0x4:
    case 0x5:
    case 0x6:
    case 0x7:
    case 0x8:
        return (DRIVE_SELFTEST_FAILED);

    case 0xf:
        return (DRIVE_SELFTEST_RUNNING);

    default:
        return (DRIVE_SELFTEST_UNKNOWN);
    }
}

/*
 * Read the most recent ATA SMART self-test result from an ada device.
 */
int
ata_selftest_status(struct cam_device *dev,
		    DRIVE *dp) {
    union ccb *ccb;
    uint8_t buffer[ATA_SMART_LOG_SIZE];
    uint8_t index;
    uint8_t status_byte;
    uint8_t status;
    uint32_t cam_status;
    size_t entry_offset;
    int saved_errno;
    uint64_t poh;
    

    if (!dev || !dp) {
        errno = EINVAL;
        return (-1);
    }

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        errno = ENOMEM;
        return (-1);
    }

    cam_fill_ataio(&ccb->ataio,
        1,
        NULL,
        CAM_DIR_IN,
        CAM_TAG_ACTION_NONE,
        buffer,
        sizeof(buffer),
        10000);

    /*
     * SMART READ LOG:
     *
     * command       = 0xb0
     * feature       = 0xd5
     * sector count  = 1
     * LBA low       = log address 0x06
     * LBA mid       = 0x4f
     * LBA high      = 0xc2
     */
    ata_28bit_cmd(&ccb->ataio,
        ATA_CMD_SMART,
        ATA_SMART_READ_LOG,
        ATA_SMART_READ_LOG_LBA,
        1);

    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;
        cam_freeccb(ccb);
        errno = saved_errno;
        return (-1);
    }

    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (cam_status != CAM_REQ_CMP) {
        fprintf(stderr,
            "%s%u: ATA SMART READ LOG failed: CAM status 0x%x\n",
            dev->given_dev_name,
            dev->given_unit_number,
            cam_status);

        cam_freeccb(ccb);
        errno = EIO;
        return (-1);
    }

    if (ccb->ataio.resid != 0) {
        fprintf(stderr,
            "%s%u: incomplete ATA self-test log transfer, "
            "residual=%u\n",
            dev->given_dev_name,
            dev->given_unit_number,
            ccb->ataio.resid);

        cam_freeccb(ccb);
        errno = EIO;
        return (-1);
    }

    cam_freeccb(ccb);

    if (!ata_smart_checksum_valid(buffer, sizeof(buffer))) {
        fprintf(stderr,
            "%s%u: ATA SMART self-test log checksum failed\n",
            dev->given_dev_name,
            dev->given_unit_number);

        errno = EIO;
        return (-1);
    }

    /*
     * Byte 508 contains the index of the most recently written
     * descriptor. Valid values are 1 through 21.
     */
    index = buffer[ATA_SELF_TEST_INDEX_OFFSET];

    if (index == 0) {
        dp->raw.selftest_status = DRIVE_SELFTEST_NONE;
        return (0);
    }

    if (index > ATA_SELF_TEST_ENTRY_COUNT) {
        fprintf(stderr,
            "%s%u: invalid ATA self-test descriptor index %u\n",
            dev->given_dev_name,
            dev->given_unit_number,
            index);

        errno = EIO;
        return (-1);
    }

    entry_offset = ATA_SELF_TEST_ENTRY_OFFSET +
        ((size_t)index - 1) * ATA_SELF_TEST_ENTRY_SIZE;

    /*
     * Entry layout:
     *
     * byte 0     test number/type
     * byte 1     status in upper nibble, remaining percentage below
     * byte 2-3   lifetime hours, little-endian
     * byte 4-7   failing LBA, little-endian
     */
     status_byte = buffer[entry_offset + 1];
     status = (status_byte >> 4) & 0x0f;
     
     dp->raw.selftest_status = classify_ata_selftest(status);
     
     poh = get_le16(&buffer[entry_offset + 2]);
     if (poh > dp->raw.power_on_hours)
	 dp->raw.power_on_hours = poh;
     
     return 1;
}


int
ata_health_status(struct cam_device *dev,
		  DRIVE *dp) {
    int rc1 = 0, rc2 = 0, rc3 = 0;


    if (f_debug)
	fprintf(stderr, "  [temp]\n");
    rc1 = ata_drive_temperature(dev, dp);
    if (rc1 < 0)
	return rc1;
    
    if (f_debug)
	fprintf(stderr, "  [smart]\n");
    rc2 = ata_smart_status(dev, dp);

    if (dp->flags.have_selftest) {
	if (f_debug)
	    fprintf(stderr, "  [selftest]\n");
	rc3 = ata_selftest_status(dev, dp);
    }

    if (rc1 > 0 || rc2 > 0 || rc3 > 0)
	return 1;
    if (rc1 == 0 && rc2 == 0 && rc3 == 0)
	return 0;
    
    return -1;
}
