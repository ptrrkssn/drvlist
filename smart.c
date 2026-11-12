/*
 * smart.c
 *
 * Code to read SMART status from SAS/SCSI (da), SATA (ada) and NVME (nda) drives
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
#include <sys/endian.h>

#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/ata/ata_all.h>
#include <cam/scsi/scsi_all.h>
#include <camlib.h>

#include <dev/nvme/nvme.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "smart.h"


#define ATA_CMD_SMART                  0xb0
#define ATA_SMART_RETURN_STATUS        0xda

#define ATA_SMART_LBA_GOOD             0x00c24f00U
#define ATA_SMART_LBA_FAILED           0x002cf400U

#define SCSI_LOG_SENSE                 0x4d
#define SCSI_IE_LOG_PAGE               0x2f

#define NVME_ADMIN_GET_LOG_PAGE        0x02
#define NVME_LOG_HEALTH_INFORMATION    0x02
#define NVME_HEALTH_LOG_SIZE           512

/*
 * Some FreeBSD header revisions expose the opcode under another name.
 * Use the standard NVMe value locally.
 */
#ifndef NVME_OPC_GET_LOG_PAGE
#define NVME_OPC_GET_LOG_PAGE NVME_ADMIN_GET_LOG_PAGE
#endif

static uint16_t
get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8 | p[1]);
}

static void
smart_result_init(struct smart_result *result)
{
    result->health = SMART_HEALTH_UNKNOWN;
    result->detail1 = 0;
    result->detail2 = 0;
}

/*
 * ATA/SATA: SMART RETURN STATUS
 *
 * The command returns one of these cylinder/LBA register pairs:
 *
 *     LBA mid = 0x4f, LBA high = 0xc2: thresholds not exceeded
 *     LBA mid = 0xf4, LBA high = 0x2c: threshold exceeded
 */
static int
cam_smart_status_ada(struct cam_device *dev,
		     struct smart_result *result)
{
    union ccb *ccb;
    uint32_t cam_status;
    uint8_t lba_mid;
    uint8_t lba_high;
    int saved_errno;

    smart_result_init(result);

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

    result->detail1 = lba_mid;
    result->detail2 = lba_high;

    cam_freeccb(ccb);

    if (lba_mid == 0x4f && lba_high == 0xc2) {
        result->health = SMART_HEALTH_PASSED;
	result->protocol = SMART_PROTOCOL_ATA;
        return (1);
    }

    if (lba_mid == 0xf4 && lba_high == 0x2c) {
        result->health = SMART_HEALTH_FAILED;
	result->protocol = SMART_PROTOCOL_ATA;
        return (1);
    }

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

/*
 * SCSI/SAS: Informational Exceptions log page 0x2f
 *
 * Parameter code 0 contains the device's informational-exception
 * additional sense code and qualifier.
 */
static int
cam_smart_status_da(struct cam_device *dev, struct smart_result *result)
{
    union ccb *ccb;
    uint8_t response[252];
    uint8_t *cdb;
    uint32_t cam_status;
    size_t response_length;
    size_t offset;
    int saved_errno;

    
    smart_result_init(result);
    memset(response, 0, sizeof(response));

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        errno = ENOMEM;
        return (-1);
    }

    cam_fill_csio(&ccb->csio,
        1,                      /* retries */
        NULL,                   /* callback */
        CAM_DIR_IN,
        CAM_TAG_ACTION_NONE,
        response,
        sizeof(response),
        SSD_FULL_SIZE,
        10,                     /* CDB length */
        5000);                  /* timeout, ms */

    cdb = ccb->csio.cdb_io.cdb_bytes;
    memset(cdb, 0, 10);

    /*
     * LOG SENSE(10)
     *
     * byte 0: operation code 0x4d
     * byte 2:
     *     PC = 01b, current cumulative values
     *     page code = 0x2f, informational exceptions
     * byte 7-8: allocation length
     */
    cdb[0] = SCSI_LOG_SENSE;
    cdb[2] = 0x40 | SCSI_IE_LOG_PAGE;
    cdb[7] = (uint8_t)(sizeof(response) >> 8);
    cdb[8] = (uint8_t)(sizeof(response) & 0xff);

    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;
        cam_freeccb(ccb);
        errno = saved_errno;
        return (-1);
    }

    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (cam_status != CAM_REQ_CMP) {
        fprintf(stderr,
            "%s%u: SCSI LOG SENSE page 0x2f failed: "
            "CAM status 0x%x\n",
            dev->given_dev_name,
            dev->given_unit_number,
            cam_status);

        cam_freeccb(ccb);
        errno = EIO;
        return (-1);
    }

    /*
     * Do not inspect bytes that CAM says were not transferred.
     */
    response_length = sizeof(response);

    if (ccb->csio.resid > 0 &&
        (uint32_t)ccb->csio.resid <= sizeof(response)) {
        response_length -= (size_t)ccb->csio.resid;
    }

    cam_freeccb(ccb);

    if (response_length < 4) {
        errno = EIO;
        return (-1);
    }

    /*
     * LOG SENSE header:
     *
     * byte 0: page code
     * byte 1: subpage/reserved
     * byte 2-3: page length
     */
    {
        size_t advertised_length;

        advertised_length = (size_t)get_be16(&response[2]) + 4;

        if (advertised_length < response_length)
            response_length = advertised_length;
    }

    /*
     * Parameter descriptor:
     *
     * byte 0-1: parameter code
     * byte 2:   parameter control
     * byte 3:   parameter length
     * byte 4..: parameter data
     */
    offset = 4;

    while (offset + 4 <= response_length) {
        uint16_t parameter_code;
        uint8_t parameter_length;
        uint8_t asc;
        uint8_t ascq;

        parameter_code = get_be16(&response[offset]);
        parameter_length = response[offset + 3];

        if (offset + 4 + parameter_length > response_length)
            break;

        if (parameter_code == 0 && parameter_length >= 2) {
            asc = response[offset + 4];
            ascq = response[offset + 5];

            result->detail1 = asc;
            result->detail2 = ascq;

            /*
             * No informational exception is currently reported.
             */
            if (asc == 0 && ascq == 0) {
                result->health = SMART_HEALTH_PASSED;
		result->protocol = SMART_PROTOCOL_SCSI;
                return (1);
            }

            /*
             * Common predictive-failure ASC:
             *
             *     ASC 0x5d: Failure prediction threshold exceeded
             *
             * ASCQ provides the component-specific detail.
             */
            if (asc == 0x5d) {
                result->health = SMART_HEALTH_FAILED;
		result->protocol = SMART_PROTOCOL_SCSI;
                return (1);
            }

            /*
             * The device reports another informational exception.
             * Preserve ASC/ASCQ and report it as a warning.
             */
            result->health = SMART_HEALTH_WARNING;
	    result->protocol = SMART_PROTOCOL_SCSI;
            return (1);
        }

        offset += 4 + parameter_length;
    }

    fprintf(stderr,
        "%s%u: informational-exceptions parameter not found\n",
        dev->given_dev_name,
        dev->given_unit_number);

    errno = EIO;
    return (-1);
}

/*
 * NVMe: SMART / Health Information log page 0x02
 *
 * The first byte is Critical Warning:
 *
 * bit 0: available spare below threshold
 * bit 1: temperature outside threshold
 * bit 2: NVM subsystem reliability degraded
 * bit 3: media placed in read-only mode
 * bit 4: volatile memory backup failed
 * bit 5: persistent-memory read-only condition
 */
static int
cam_smart_status_nda(struct cam_device *dev,
		     struct smart_result *result)
{
    union ccb *ccb;
    uint8_t health_log[NVME_HEALTH_LOG_SIZE];
    uint32_t cam_status;
    uint32_t numd;
    uint8_t critical_warning;
    int saved_errno;


    smart_result_init(result);
    memset(health_log, 0, sizeof(health_log));

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        errno = ENOMEM;
        return (-1);
    }

    cam_fill_nvmeadmin(&ccb->nvmeio,
        1,                      /* retries */
        NULL,                   /* callback */
        CAM_DIR_IN,
        health_log,
        sizeof(health_log),
        5000);                  /* timeout, ms */

    ccb->nvmeio.cmd.opc = NVME_OPC_GET_LOG_PAGE;

    /*
     * Controller-wide SMART / Health Information.
     *
     * Some controllers also support namespace-specific health logs,
     * but the controller-wide log is the portable health query.
     */
    ccb->nvmeio.cmd.nsid = htole32(0xffffffffU);

    /*
     * NUMD is the number of DWORDs minus one.
     *
     * 512 bytes = 128 DWORDs, so NUMD = 127.
     */
    numd = sizeof(health_log) / sizeof(uint32_t) - 1;

    ccb->nvmeio.cmd.cdw10 =
        htole32((numd << 16) | NVME_LOG_HEALTH_INFORMATION);

    ccb->nvmeio.cmd.cdw11 = 0;

    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;
        cam_freeccb(ccb);
        errno = saved_errno;
        return (-1);
    }

    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (cam_status != CAM_REQ_CMP) {
        fprintf(stderr,
            "%s%u: NVMe health-log request failed: "
            "CAM status 0x%x\n",
            dev->given_dev_name,
            dev->given_unit_number,
            cam_status);

        cam_freeccb(ccb);
        errno = EIO;
        return (-1);
    }

    cam_freeccb(ccb);

    critical_warning = health_log[0];

    result->detail1 = critical_warning;

    if (critical_warning == 0) {
        result->health = SMART_HEALTH_PASSED;
	result->protocol = SMART_PROTOCOL_NVME;
        return (1);
    }

    /*
     * Treat read-only mode or degraded media reliability as failed.
     * Spare, temperature, or backup warnings remain warnings.
     */
    if ((critical_warning & ((1U << 2) | (1U << 3))) != 0)
        result->health = SMART_HEALTH_FAILED;
    else
        result->health = SMART_HEALTH_WARNING;

    result->protocol = SMART_PROTOCOL_NVME;
    return (1);
}


const char *
smart_health_name(enum smart_health health)
{
    switch (health) {
    case SMART_HEALTH_PASSED:
        return ("PASSED");
    case SMART_HEALTH_WARNING:
        return ("WARNING");
    case SMART_HEALTH_FAILED:
        return ("FAILED");
    case SMART_HEALTH_UNKNOWN:
    default:
        return ("UNKNOWN");
    }
}


int
cam_smart_status(struct cam_device *dev,
		 struct smart_result *result) {
    int rc;

    
    if (!dev || !result) {
	errno = EINVAL;
	return -1;
    }

    if (strcmp(dev->given_dev_name, "da") == 0)
	rc = cam_smart_status_da(dev, result);
    else if (strcmp(dev->given_dev_name, "ada") == 0)
	rc = cam_smart_status_ada(dev, result);
    else if (strcmp(dev->given_dev_name, "nda") == 0)
	rc = cam_smart_status_nda(dev, result);
    else {
	errno = ENOENT;
	rc = -1;
    }

    return rc;
}

int
smart_status(const char *path,
	     struct smart_result *result) {
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

    rc = cam_smart_status(dev, result);
    
    saved_errno = errno;
    cam_close_device(dev);
    errno = saved_errno;
    
    return rc;
}

#if 0
static void
print_smart_result(const struct smart_result *result) {
    printf("%s", smart_health_name(result->health));

    if (result->protocol == SMART_PROTOCOL_ATA) {
        printf(" (LBA mid=0x%02x, LBA high=0x%02x)",
            result->detail1,
            result->detail2);
    } else if (result->protocol == SMART_PROTOCOL_SCSI) {
        printf(" (ASC=0x%02x, ASCQ=0x%02x)",
            result->detail1,
            result->detail2);
    } else if (result->protocol == SMART_PROTOCOL_NVME) {
        printf(" (critical warning=0x%02x)",
            result->detail1);
    }

    putchar('\n');
}


int
main(int argc,
     char *argv[])
{
    int i, rc;
    struct smart_result result;
    
    for (i = 1; i < argc; i++) {
	memset(&result, 0, sizeof(result));
	rc = smart_status(argv[i], &result);
	if (rc > 0) {
	    printf("%10s : ", argv[i]);
	    print_smart_result(&result);
	} else if (rc == 0)
	    printf("%10s : -\n", argv[i]);
	else {
	    fprintf(stderr, "%s: Error: %s: %s\n", argv[0], argv[i], strerror(errno));
	    exit(1);
	}
    }
}
#endif
