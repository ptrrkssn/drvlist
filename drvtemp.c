/*
 * drvtemp.c
 *
 * Code to read drive temperature from SAS/SCSI (da), SATA (ada) and NVME (nda) drives
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

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/scsi/scsi_all.h>
#include <cam/scsi/scsi_message.h>
#include <cam/ata/ata_all.h>
#include <camlib.h>

#include <dev/nvme/nvme.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "drvtemp.h"


/*
 * ATA SMART commands.
 *
 * Local definitions avoid depending on the exact names exported by
 * <sys/ata.h> in a particular FreeBSD source version.
 */
#define ATA_CMD_SMART                   0xb0
#define ATA_SMART_READ_DATA             0xd0

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

/*
 * Validate the checksum of a 512-byte ATA SMART READ DATA sector.
 *
 * The unsigned sum of all 512 bytes should have its low eight bits
 * equal to zero.
 */
static int
ata_smart_checksum_valid(const uint8_t data[SMART_DATA_SIZE]) {
    unsigned int sum;
    size_t i;

    sum = 0;

    for (i = 0; i < SMART_DATA_SIZE; i++)
        sum += data[i];

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
				double *temperature_c) {
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
        return (-1);

    *temperature_c = (double)value;
    return (0);
}

/*
 * Read the temperature from a SATA drive exposed as /dev/adaN.
 *
 * Returns:
 *      0  success
 *     -1  failure, with errno set
 *
 * On success, *temperature_c receives the temperature in Celsius.
 */
static int
cam_drvtemp_ada(struct cam_device *dev,
	       double *temperature_c)
{
    union ccb *ccb;
    struct ata_smart_attribute attr;
    uint8_t data[SMART_DATA_SIZE];
    uint32_t cam_status;
    double fallback_temperature;
    int have_fallback;
    int saved_errno;
    size_t i;
    size_t offset;

    
    if (!temperature_c) {
        errno = EINVAL;
        return -1;
    }

    memset(data, 0, sizeof(data));

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
        data,
        sizeof(data),
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
        ATA_SMART_READ_DATA,
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

    cam_freeccb(ccb);

    if (!ata_smart_checksum_valid(data)) {
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
        offset = SMART_ATTRIBUTE_OFFSET +
            i * SMART_ATTRIBUTE_SIZE;

        memcpy(&attr, &data[offset], sizeof(attr));

        if (attr.id == 0)
            continue;

        if (attr.id == SMART_ATTR_TEMPERATURE_CELSIUS) {
            if (ata_smart_attribute_temperature(&attr,
                temperature_c) == 0)
                return 1;
        }

        if (attr.id == SMART_ATTR_AIRFLOW_TEMPERATURE &&
            !have_fallback) {
            if (ata_smart_attribute_temperature(&attr,
                &fallback_temperature) == 0)
                have_fallback = 1;
        }
    }

    if (have_fallback) {
        *temperature_c = fallback_temperature;
        return 1;
    }

    return 0;
}


#define TEMP_LOG_PAGE   0x0d

struct log_sense_cdb {
    uint8_t opcode;
    uint8_t byte2;
    uint8_t page;
    uint8_t subpage;
    uint8_t reserved[3];
    uint8_t alloc_len[2];
    uint8_t control;
};

static uint16_t
be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static int
cam_drvtemp_da(struct cam_device *dev,
	       double *temp_c) {
    union ccb *ccb;
    struct log_sense_cdb *cdb;

    uint8_t buf[252];
    size_t len, off;


    ccb = cam_getccb(dev);
    if (!ccb)
	return -1;

    memset(buf, 0, sizeof(buf));

    cam_fill_csio(&ccb->csio,
        1,                      /* retries */
        NULL,                   /* callback */
        CAM_DIR_IN,
        MSG_SIMPLE_Q_TAG,
        buf,
        sizeof(buf),
        SSD_FULL_SIZE,
        10,                     /* CDB length */
        5000);                  /* timeout ms */

    cdb = (struct log_sense_cdb *)&ccb->csio.cdb_io.cdb_bytes;

    memset(cdb, 0, sizeof(*cdb));

    cdb->opcode = LOG_SENSE;

    /*
     * PC = 01b: current cumulative values.
     * Page code = 0x0d: temperature.
     */
    cdb->page = (1 << 6) | TEMP_LOG_PAGE;

    cdb->alloc_len[0] = sizeof(buf) >> 8;
    cdb->alloc_len[1] = sizeof(buf) & 0xff;

    if (cam_send_ccb(dev, ccb) < 0) {
        cam_freeccb(ccb);
        return -1;
    }

    if ((ccb->ccb_h.status & CAM_STATUS_MASK) != CAM_REQ_CMP) {
        fprintf(stderr, "LOG SENSE failed: CAM status 0x%x\n",
            ccb->ccb_h.status & CAM_STATUS_MASK);

        cam_freeccb(ccb);
        return -1;
    }

    /*
     * LOG SENSE page header:
     *
     * byte 0   page code
     * byte 1   subpage/reserved
     * byte 2-3 page length
     */
    len = be16(&buf[2]) + 4;

    if (len > sizeof(buf))
        len = sizeof(buf);

    /*
     * Parameters begin at byte 4.
     *
     * Parameter header:
     *
     *  0-1  parameter code
     *  2    parameter control byte
     *  3    parameter length
     */
    off = 4;

    while (off + 4 <= len) {
        uint16_t parameter = be16(&buf[off]);
        uint8_t plen = buf[off + 3];

        if (off + 4 + plen > len)
            break;

        if (parameter == 0x0000 && plen >= 2) {
	    int rc;
	    
            /*
             * Temperature parameter.
             *
             * First data byte is reserved.
             * Second is temperature in degrees Celsius.
             *
             * 0xff means temperature unavailable.
             */
            uint8_t temp = buf[off + 5];

            if (temp == 0xff)
                rc = 0;
            else {
		*temp_c = temp;
		rc = 1;
	    }

            cam_freeccb(ccb);
            return rc;
        }

        off += 4 + plen;
    }

    cam_freeccb(ccb);
    return 0;
}


/*
 * Read the NVMe composite temperature from an nda device.
 *
 * Returns:
 *      1 on success
 *      0 on no temp information in drive
 *     -1 on failure, with errno set where applicable.
 */
static int
cam_drvtemp_nda(struct cam_device *dev,
		double *temperature_c)
{
    union ccb *ccb;
    uint8_t health_log[512];
    uint16_t kelvin;
    uint32_t numd;
    uint32_t cam_status;
    int saved_errno;

    
    if (!dev || !temperature_c) {
        errno = EINVAL;
        return -1;
    }

    memset(health_log, 0, sizeof(health_log));

    ccb = cam_getccb(dev);
    if (ccb == NULL)
        return -1;

    /*
     * Build an NVMe Admin command CCB.
     *
     * CAM_DIR_IN means device-to-host data transfer.
     */
    cam_fill_nvmeadmin(&ccb->nvmeio,
        1,                      /* retries */
        NULL,                   /* completion callback */
        CAM_DIR_IN,
        health_log,
        sizeof(health_log),
        5000);                  /* timeout in milliseconds */

    /*
     * NVMe Admin Get Log Page command, opcode 0x02.
     */
    ccb->nvmeio.cmd.opc = NVME_OPC_GET_LOG_PAGE;

    /*
     * The SMART / Health Information log can be requested globally
     * using namespace ID 0xffffffff.
     */
    ccb->nvmeio.cmd.nsid = htole32(0xffffffffU);

    /*
     * CDW10:
     *
     * bits  7:0  LID  = 0x02, SMART / Health Information
     * bits 31:16 NUMD = number of DWORDs minus one
     *
     * 512 bytes / 4 = 128 DWORDs
     * NUMD = 127
     */
    numd = (sizeof(health_log) / sizeof(uint32_t)) - 1;

    ccb->nvmeio.cmd.cdw10 =
        htole32((numd << 16) | 0x02U);

    /*
     * CDW11 is zero because the log-page offset is zero.
     */
    ccb->nvmeio.cmd.cdw11 = 0;

    if (cam_send_ccb(dev, ccb) < 0) {
	saved_errno = errno;
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

    /*
     * NVMe SMART / Health Information log:
     *
     * byte 0    Critical Warning
     * byte 1-2  Composite Temperature, little-endian Kelvin
     */
    kelvin = le16dec(&health_log[1]);

    /*
     * Zero means that no valid temperature was reported.
     */
    if (kelvin == 0) {
        cam_freeccb(ccb);
        errno = EIO;
        return 0;
    }

    *temperature_c = (double)kelvin - 273.15;

    cam_freeccb(ccb);
    return 1;
}


int
cam_drvtemp(struct cam_device *dev,
	    double *temp) {
    int rc;
    
    
    if (!dev || !temp) {
	errno = EINVAL;
	return -1;
    }

    if (strcmp(dev->given_dev_name, "da") == 0)
	rc = cam_drvtemp_da(dev, temp);
    else if (strcmp(dev->given_dev_name, "ada") == 0)
	rc = cam_drvtemp_ada(dev, temp);
    else if (strcmp(dev->given_dev_name, "nda") == 0)
	rc = cam_drvtemp_nda(dev, temp);
    else {
	errno = ENOENT;
	rc = -1;
    }

    return rc;
}


int
drvtemp(const char *path,
	double *temp_c) {
    const char *cp;
    int saved_errno, rc;
    struct cam_device *dev;
    

    if (!path) {
	errno = EINVAL;
	return -1;
    }
    
    cp = strrchr(path, '/');
    if (cp)
	++cp;
    else
	cp = path;

    dev = cam_open_device(cp, O_RDWR);
    if (!dev)
	return -1;

    rc = cam_drvtemp(dev, temp_c);
    
    saved_errno = errno;
    cam_close_device(dev);
    errno = saved_errno;
    
    return rc;
}

#ifdef WITH_MAIN
int
main(int argc,
     char *argv[]) {
    int i, rc;
    double t;


    for (i = 1; i < argc; i++) {
	rc = drvtemp(argv[i], &t);
	
	if (rc > 0)
	    printf("%10s : %.1f C\n", argv[i], t);
	else if (rc == 0)
	    printf("%10s : -\n", argv[i]);
	else {
	    fprintf(stderr, "%s: Error: %s: %s\n", argv[0], argv[i], strerror(errno));
	    exit(1);
	}
    }

    return 0;
}
#endif

