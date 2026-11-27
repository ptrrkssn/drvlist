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
#define ATA_CMD_SMART                 0xb0

#define ATA_SMART_READ_DATA           0xd0
#define ATA_SMART_READ_LOG            0xd5
#define ATA_SMART_RETURN_STATUS       0xda

#define ATA_SMART_LBA_GOOD                      0x00c24f00U
#define ATA_SMART_LBA_FAILED          	        0x002cf400U

#define ATA_SMART_LOG_DIRECTORY_LBA             0x00c24f00U
#define ATA_SMART_SUMMARY_ERROR_LOG_LBA         0x00c24f01U
#define ATA_SMART_COMPREHENSIVE_ERROR_LOG_LBA   0x00c24f02U
#define ATA_SMART_SELF_TEST_LOG_LBA             0x00c24f06U
#define ATA_SMART_SELECTIVE_SELF_TEST_LOG_LBA   0x00c24f09U

#define ATA_SMART_LOG_SIZE            	    512

#define ATA_SMART_SELF_TEST_ENTRY_SIZE      24
#define ATA_SMART_SELF_TEST_ENTRY_COUNT     21
#define ATA_SMART_SELF_TEST_ENTRY_OFFSET    2
#define ATA_SMART_SELF_TEST_INDEX_OFFSET    508

#define ATA_SMART_LBA                   0x00c24f00U

#define SMART_DATA_SIZE                 512
#define SMART_ATTRIBUTE_OFFSET          2
#define SMART_ATTRIBUTE_SIZE            12
#define SMART_ATTRIBUTE_COUNT           30

#define SMART_ATTR_POWER_ON_HOURS         9
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

struct ata_smart_log_directory {
    uint16_t version;
    uint16_t sectors[255];
};



static enum drive_selftest_status
classify_ata_selftest(uint8_t status)
{
    switch (status) {
    case 0x0: /* Completed without error */
        return (DRIVE_SELFTEST_PASSED);

    case 0x1: /* Aborted by host */
    case 0x2: /* Interrupted by host reset */
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

static const char *
ata_cap_gen2str(uint16_t cap) {
    if (cap & ATA_SATA_GEN3)
	return "SATA-3";
    else if (cap & ATA_SATA_GEN2)
	return "SATA-2";
    else if (cap & ATA_SATA_GEN1)
	return "SATA-1";
    else
	return "?";
}

const char *
ata_selftest_status2str(uint8_t status) {
    switch (status) {
    case 0x0:
	return "Completed without error";
    case 0x01:
	return "Aborted by host";
    case 0x2:
	return "Interrupted by host";
    case 0x3:
	return "Fatal or unknown error prevented completion";
    case 0x4:
	return "Completed with unknown test element failure";
    case 0x5:
	return "Completed with electrical element failure";
    case 0x6:
	return "Completed with servo/seek element failure";
    case 0x7:
	return "Completed with read element failure";
    case 0x8:
	return "Completed with handling-damage failure";
    case 0x9:
	return "Unknown error 0x09";
    case 0xA:
	return "Unknown error 0x0A";
    case 0xB:
	return "Unknown error 0x0B";
    case 0xC:
	return "Unknown error 0x0C";
    case 0xD:
	return "Unknown error 0x0D";
    case 0xE:
	return "Unknown error 0x0E";
    case 0xF:
	return "Self-test in progress";
    default:
	return "?";
    }
}

const char *
ata_selftest_type2str(uint8_t type) {
    switch (type) {
    case 0x00:
	return "Offline data collection";
    case 0x01:
	return "Short self-test, offline/background";
    case 0x02:
	return "Long self-test, offline/background";
    case 0x03:
	return "Conveyance self-test, offline/background";
    case 0x04:
	return "Selective self-test, offline/background";
    case 0x80:
	return "Offline data collection, captive/foreground";
    case 0x81:
	return "Short self-test, captive/foreground";
    case 0x82:
	return "Long self-test, captive/foreground";
    case 0x83:
	return "Conveyance self-test, captive/foreground";
    case 0x84:
	return "Selective self-test, captive/foreground";
    default:
	if ((type >= 0x05 && type <= 0x3F) || (type >= 0x85 && type <= 0xBF))
	    return "Reserved";
	if ((type >= 0x40 && type <= 0x7F) || (type >= 0xC0 && type <= 0xFF))
	    return "Vendor specific";
    }
    return "?";
}


static int
ata_smart_checksum_valid(const uint8_t *buffer,
			 size_t length)
{
    unsigned int sum;
    size_t i;

    sum = 0;

    for (i = 0; i < length; i++)
        sum += buffer[i];

    return ((sum & 0xffU) == 0);
}


ssize_t
drive_ata_command(DRIVE *dp,
		  uint8_t command,
		  uint8_t code,
		  uint32_t lba,
		  void *buf,
		  size_t bufsize) {
    struct cam_device *dev = dp->cam;
    union ccb *ccb;
    uint32_t cam_status;
    uint32_t cam_resid;
    int saved_errno;
    

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
		   buf,
		   bufsize,
		   5000);

    ata_28bit_cmd(&ccb->ataio,
		  command,
		  code,
		  lba,
		  1);
    
    ccb->ccb_h.flags |= CAM_DEV_QFRZDIS;
    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;
        cam_freeccb(ccb);
        errno = saved_errno;
        return (-1);
    }

    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;
    cam_resid = ccb->ataio.resid;
    cam_freeccb(ccb);
    
    if (cam_status != CAM_REQ_CMP) {
        fprintf(stderr,
		"%s%u: ATA COMMAND %02x/%02x failed: CAM status 0x%x\n",
		dev->given_dev_name,
		dev->given_unit_number,
		command, code,
		cam_status);
        errno = EIO;
        return (-1);
    }

    if (cam_resid != 0) {
        fprintf(stderr,
		"%s%u: ATA COMMAND %02x/%02x: incomplete ATA self-test log transfer, "
		"residual=%u\n",
		dev->given_dev_name,
		dev->given_unit_number,
		command, code,
		cam_resid);
        errno = EIO;
        return (-1);
    }

    return bufsize-cam_resid;
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


static ssize_t
ata_cmd_smart(DRIVE *dp,
	      uint8_t code,
	      uint8_t *buf,
	      size_t bufsize) {
    return drive_ata_command(dp,
			     ATA_CMD_SMART,
			     code,
			     ATA_SMART_LBA,
			     buf, bufsize);
}
			     

int
drive_ata_get_smart_data(DRIVE *dp,
			 uint8_t *buf,
			 size_t bufsize) {
    return ata_cmd_smart(dp, ATA_SMART_READ_DATA, buf, bufsize);
}

int
drive_ata_parse_smart_data(DRIVE *dp,
			   uint8_t *buf,
			   size_t bufsize) { 
    uint16_t fallback_temperature;
    size_t offset, i;
    int have_fallback;
    uint64_t poh;
    

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

	/* Sanity check */
	if (offset+sizeof(*attr) >= bufsize)
	    break;
	
        attr = (struct ata_smart_attribute *) &buf[offset];
        if (attr->id == 0)
            continue;

	switch (attr->id) {
	case SMART_ATTR_TEMPERATURE_CELSIUS:
            if (ata_smart_attribute_temperature(attr, &dp->raw.temperature) == 0) {
		if (f_debug)
		    fprintf(stderr, "\tATA SMART DATA: drive temperature=%u\n", dp->raw.temperature);
                return 1;
	    }
	    break;
	
	case SMART_ATTR_AIRFLOW_TEMPERATURE:
            if (!have_fallback) {
		if (ata_smart_attribute_temperature(attr, &fallback_temperature) == 0) {
		    if (f_debug)
			fprintf(stderr, "\tATA SMART DATA: airflow temperature=%u\n", fallback_temperature);
		    have_fallback = 1;
		}
	    }
	    break;

	case SMART_ATTR_POWER_ON_HOURS:
	    poh = get_le16(attr->raw);
	    if (f_debug)
		fprintf(stderr, "\tATA SMART DATA: power_on_hours = %lu\n", poh);
	    if (dp->raw.power_on_hours == DRIVE_NO_HOURS || poh > dp->raw.power_on_hours)
		dp->raw.power_on_hours = poh;
	    break;
	}
    }
    
    if (have_fallback) {
        dp->raw.temperature = fallback_temperature;
        return 1;
    }
    
    return 0;
}

int
drive_ata_smart_data(DRIVE *dp) {
    uint8_t data[SMART_DATA_SIZE];
    ssize_t res;

    res = drive_ata_get_smart_data(dp, data, sizeof(data));
    if (res < 0)
	return res;

    return drive_ata_parse_smart_data(dp, data, res);
}
    



int
drive_ata_parse_identify(DRIVE *dp,
			struct ata_params *app) {
    uint32_t logical_size = 512;
    uint32_t physical_size = 512;
    char buf[128], tbuf[64];;
    
    ata_param_fixup(app);

    
    if ((app->pss & 0xc000) == 0x4000) {
	if (app->pss & (1U << 12)) {
	    uint32_t words_per_logical;

	    words_per_logical = (uint32_t) app->lss_1 | ((uint32_t) app->lss_2 << 16);
	    logical_size = words_per_logical * 2;
	}
	if (app->pss & (1U << 13)) {
	    unsigned exponent = app->pss & 0x0f;

	    physical_size = logical_size << exponent;
	} else
	    physical_size = logical_size;
    }


    if (logical_size == physical_size)
	sprintf(buf, "%sn", bsize2str(logical_size, NULL, 0));
    else if (physical_size == 4096)
	sprintf(buf, "%se", bsize2str(logical_size, NULL, 0));
    else
	sprintf(buf, "%s/%s",
		bsize2str(logical_size, NULL, 0),
		bsize2str(physical_size, tbuf, sizeof(tbuf)));
		
    dp->sector = strdup(buf);

    dp->rpm = strdup(rpm2str(app->media_rotation_rate));
    dp->form = strdup(form2str(app->form_factor & ATA_FORM_FACTOR_MASK));
    
    if (f_debug) {
	fprintf(stderr, "\tATA IDENTIFY: model           = %.*s\n",
		(int) sizeof(app->model), app->model);
	fprintf(stderr, "\tATA IDENTIFY: revision        = %.*s\n",
		(int) sizeof(app->revision), app->revision);
	fprintf(stderr, "\tATA IDENTIFY: serial          = %.*s\n",
		(int) sizeof(app->serial), app->serial);
	
	fprintf(stderr, "\tATA IDENTIFY: rpm             = %s\n",
		dp->rpm);
	fprintf(stderr, "\tATA IDENTIFY: form_factor     = %s\n",
		dp->form);
	
	fprintf(stderr, "\tATA IDENTIFY: physical_sector = %u\n", physical_size);
	fprintf(stderr, "\tATA IDENTIFY: logical_sector  = %u\n", logical_size);
		
	fprintf(stderr, "\tATA IDENTIFY: tcg             = 0x%04x\n",
		app->tcg);
	fprintf(stderr, "\tATA IDENTIFY: capabilities1   = 0x%04x\n",
		app->capabilities1);
	fprintf(stderr, "\tATA IDENTIFY: capabilities2   = 0x%04x\n",
		app->capabilities2);
	fprintf(stderr, "\tATA IDENTIFY: multi           = 0x%04x\n",
		app->multi);
	fprintf(stderr, "\tATA IDENTIFY: mwdmamodes      = 0x%04x\n",
		app->mwdmamodes);
	fprintf(stderr, "\tATA IDENTIFY: version_major   = 0x%04x\n",
		app->version_major);
	fprintf(stderr, "\tATA IDENTIFY: version_minor   = 0x%04x\n",
		app->version_minor);
	fprintf(stderr, "\tATA IDENTIFY: mwdmamodes      = 0x%04x\n",
		app->mwdmamodes);
	fprintf(stderr, "\tATA IDENTIFY: extension       = 0x%04x :%s%s\n",
		app->enabled.extension,
		((app->enabled.extension & ATA_SUPPORT_SMARTLOG) ? " SMARTLOG" : ""),
		((app->enabled.extension & ATA_SUPPORT_SMARTTEST) ? " SMARTTEST" : ""));

	if (app->atavalid & ATA_FLAG_64_70) {
	    fprintf(stderr, "\tATA IDENTIFY: apiomodes       = 0x%04x\n",
		    app->apiomodes);
	}
	fprintf(stderr, "\tATA IDENTIFY: satacapabilities  = 0x%04x (%s)\n",
		app->satacapabilities, ata_cap_gen2str(app->satacapabilities));
	fprintf(stderr, "\tATA IDENTIFY: satacapabilities2 = 0x%04x\n",
		app->satacapabilities2);
    }
    
    dp->vendor  = strdup("ATA");
    dp->product = strndup((char *) app->model,    sizeof(app->model));
    dp->version = strndup((char *) app->revision, sizeof(app->revision));

    dp->proto = strdup(ata_cap_gen2str(app->satacapabilities));
    
    dp->flags.have_smart = (app->enabled.extension & ATA_SUPPORT_SMARTLOG) != 0;
    dp->flags.have_selftest = (app->enabled.extension & ATA_SUPPORT_SMARTTEST) != 0;

    return 0;
}

int
drive_ata_identify(DRIVE *dp) {
    struct cam_device *cdb = dp->cam;
    union ccb *ccb;
    struct ata_params apb;
    uint8_t *bp;
    u_int i, error;
    uint8_t command, retry_command;
    

    if (f_debug)
	fprintf(stderr, "  [ata_identify]\n");
    
    if ((ccb = cam_getccb(cdb)) == NULL) {
	return -1;
    }

    command = ATA_ATA_IDENTIFY;
    retry_command = ATA_ATAPI_IDENTIFY;
    
 retry:
    { ssize_t res;
	
	res = drive_ata_command(dp, command, 0, 0, (uint8_t *) &apb, sizeof(apb));
	if (res <= 0)
	    error = 1;
	else
	    error = 0;
    }
    
    if (error != 0) {
	if (retry_command != 0) {
	    command = retry_command;
	    retry_command = 0;
	    goto retry;
	}
	return (1);
    }
    
    
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

    drive_ata_parse_identify(dp, &apb);
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
drive_ata_smart_status(DRIVE *dp) {
    struct cam_device *dev = dp->cam;
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



int
drive_ata_smart_log_directory(DRIVE *dp) {
    struct ata_smart_log_directory dir;
    ssize_t res;
    

    res = drive_ata_command(dp, ATA_CMD_SMART,
			    ATA_SMART_READ_LOG,
			    ATA_SMART_LOG_DIRECTORY_LBA,
			    &dir,
			    sizeof(dir));
    if (res < 0) {
        return (-1);
    }

    if (f_debug) {
	int i;
	fprintf(stderr, "\tSMART LOG DIRECTORY: version = %u\n", get_le16((uint8_t *) &dir.version));
	for (i = 0; i < 255; i++) {
	    uint16_t ns = get_le16((uint8_t *) &dir.sectors[i]);
	    if (ns != 0)
		fprintf(stderr, "\tSMART LOG DIRECTORY: log 0x%02x = %u sectors\n", i+1, ns);
	}
    }	    

    return 0;
}



/*
 * Read the most recent ATA SMART self-test result from an ada device.
 */
ssize_t
drive_ata_get_smart_log(DRIVE *dp,
			uint8_t *buf,
			size_t bufsize) {
    memset(buf, 0, bufsize);
    return drive_ata_command(dp, ATA_CMD_SMART,
			    ATA_SMART_READ_LOG,
			    ATA_SMART_SELF_TEST_LOG_LBA,
			    buf,
			    bufsize);
}


int
drive_ata_parse_smart_log(DRIVE *dp,
			  uint8_t *buf,
			  size_t bufsize) {
    struct cam_device *dev = dp->cam;
    uint8_t index;
    uint8_t status_byte;
    uint8_t status;
    size_t entry_offset;
    uint64_t poh;
    int i = 0;
    

    if (!ata_smart_checksum_valid(buf, bufsize)) {
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
    index = buf[ATA_SMART_SELF_TEST_INDEX_OFFSET];

    if (f_debug)
	fprintf(stderr, "\tATA SELFTEST: most recent index = %u\n", index);
    
    if (index == 0) {
        dp->raw.selftest.status = DRIVE_SELFTEST_NONE;
        return (0);
    }

    if (index > ATA_SMART_SELF_TEST_ENTRY_COUNT) {
        fprintf(stderr,
            "%s%u: invalid ATA self-test descriptor index %u\n",
            dev->given_dev_name,
            dev->given_unit_number,
            index);

        errno = EIO;
        return (-1);
    }

    entry_offset = ATA_SMART_SELF_TEST_ENTRY_OFFSET +
        ((size_t)index - 1) * ATA_SMART_SELF_TEST_ENTRY_SIZE;

    /*
     * Entry layout:
     *
     * byte 0     test number/type
     * byte 1     status in upper nibble, remaining percentage below
     * byte 2-3   lifetime hours, little-endian
     * byte 4-7   failing LBA, little-endian
     */
     status_byte = buf[entry_offset + 1];
     status = (status_byte >> 4) & 0x0f;
 
     poh = get_le16(&buf[entry_offset + 2]);
     if (dp->raw.last_self_test == DRIVE_NO_HOURS || poh > dp->raw.last_self_test)
	 dp->raw.last_self_test = poh;
     
     if (f_debug) {
	 uint16_t rev = get_le16(buf);
	 fprintf(stderr, "\tATA SELFTEST revision       = %d\n", rev);
	 fprintf(stderr, "\tATA SELFTEST last_self_test = %lu\n", poh);
     }

#if 0     
     /* Get the hours from the latest selftest if we don't have anything else */
     if (dp->raw.power_on_hours == DRIVE_NO_HOURS || poh > dp->raw.power_on_hours)
	 dp->raw.power_on_hours = poh;
#endif
     
     /* Get the status from the latest test */
     dp->raw.selftest.status = classify_ata_selftest(status);

     for (i = 0; i < ATA_SMART_SELF_TEST_ENTRY_COUNT; i++) {
	 off_t off = ATA_SMART_SELF_TEST_ENTRY_OFFSET + i*ATA_SMART_SELF_TEST_ENTRY_SIZE;
	 
	 uint8_t status = (buf[off+1] >> 4) & 0x0f;
	 uint8_t remain = (buf[off+1] & 0xF)*10;
	 uint16_t hours = get_le16(&buf[off+2]);

	 
	 dp->raw.selftest.tests++;
	 switch (classify_ata_selftest(status)) {
	 case DRIVE_SELFTEST_PASSED:
	     dp->raw.selftest.passed++;
	     break;
	 case DRIVE_SELFTEST_FAILED:
	     dp->raw.selftest.failed++;
	     break;
	 case DRIVE_SELFTEST_ABORTED:
	     dp->raw.selftest.aborted++;
	     break;
	 case DRIVE_SELFTEST_RUNNING:
	     dp->raw.selftest.running++;
	     break;
	 case DRIVE_SELFTEST_UNKNOWN:
	 case DRIVE_SELFTEST_NONE:
	     break;
	 }
     
	 if (f_debug) {
	     fprintf(stderr, "\tATA SELFTEST %2d : %5u : %u : %-35s : %-35s : %u %%\n",
		     i+1,
		     hours,
		     buf[off+0],
		     ata_selftest_type2str(buf[off+0]),
		     ata_selftest_status2str(status),
		     remain);
	 }
     }
     
     return 1;
}

ssize_t
drive_ata_smart_log(DRIVE *dp) {
    uint8_t buffer[ATA_SMART_LOG_SIZE];
    ssize_t res;

    res = drive_ata_get_smart_log(dp, buffer, sizeof(buffer));
    if (res < 0)
	return res;

    return drive_ata_parse_smart_log(dp, buffer, res);
}
    


int
drive_ata_health_status(DRIVE *dp) {
    int rc1 = 0, rc2 = 0, rc3 = 0;


    if (dp->flags.have_smart) {
	if (f_debug)
	    fprintf(stderr, "  [temp]\n");
	rc1 = drive_ata_smart_data(dp);
	if (rc1 < 0)
	    return rc1;
	
	if (f_debug)
	    fprintf(stderr, "  [smart]\n");
	rc2 = drive_ata_smart_status(dp);
    }

    if (dp->flags.have_selftest) {
	if (f_debug)
	    fprintf(stderr, "  [selftest]\n");
	drive_ata_smart_log_directory(dp);
	rc3 = drive_ata_smart_log(dp);
    }

    if (rc1 > 0 || rc2 > 0 || rc3 > 0)
	return 1;
    if (rc1 == 0 && rc2 == 0 && rc3 == 0)
	return 0;
    
    return -1;
}

