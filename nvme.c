/*
 * nvme.c
 *
 * Code to access NVME (nda) drives
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
#include <camlib.h>

#include <dev/nvme/nvme.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "utils.h"
#include "nvme.h"


extern int f_debug;
extern char *argv0;

#define NVME_ADMIN_GET_LOG_PAGE        0x02
#define NVME_LOG_HEALTH_INFORMATION    0x02
#define NVME_HEALTH_LOG_SIZE           512


#define NVME_DEVICE_SELF_TEST_LOG      0x06

#define NVME_SELF_TEST_LOG_SIZE        564
#define NVME_SELF_TEST_HEADER_SIZE     4
#define NVME_SELF_TEST_ENTRY_SIZE      28
#define NVME_SELF_TEST_ENTRY_COUNT     20


/*
 * Read the NVMe composite temperature from an nda device.
 *
 * Returns:
 *      1 on success
 *      0 on no temp information in drive
 *     -1 on failure, with errno set where applicable.
 */
int
nvme_drive_temperature(struct cam_device *dev,
		       DRIVE *dp) {
    union ccb *ccb;
    uint8_t health_log[512];
    uint16_t kelvin;
    uint32_t numd;
    uint32_t cam_status;
    int saved_errno;

    
    if (!dev || !dp) {
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

    dp->raw.temperature = kelvin - 273;

    if (f_debug)
	fprintf(stderr, "\tdrive_temperature: NVME HEALTH: temp = %u\n", dp->raw.temperature);
    
    cam_freeccb(ccb);
    return 1;
}


int
nvme_identify_fd(int fd,
		 char **vendor,
		 char **model,
		 char **revision,
		 char **serial,
		 char **phys,
		 int *have_selftest) {
    struct nvme_pt_command pt;
    struct nvme_controller_data cdata;
    char *cp, *tv;
    
    
    memset(&pt, 0, sizeof(pt));
    memset(&cdata, 0, sizeof(cdata));
    
    pt.cmd.opc = NVME_OPC_IDENTIFY;
    pt.cmd.cdw10 = htole32(1);
    pt.buf = &cdata;
    pt.len = sizeof(cdata);
    pt.is_read = 1;
    
    if (ioctl(fd, NVME_PASSTHROUGH_CMD, &pt) < 0) {
	fprintf(stderr, "NVME ioctl: %s\n", strerror(errno));
	return -1;
    }
	
    if (nvme_completion_is_error(&pt.cpl)) {
	fprintf(stderr, "NVME nvme_completion\n");
	return -1;
    }

    tv = strdupget(((const char*)cdata.mn), NVME_MODEL_NUMBER_LENGTH);
    if (!tv)
	return -1;
	
    /* Locate space separating vendor and model */
    for (cp = tv; *cp && !isspace(*cp); ++cp)
	;
    if (*cp) {
	*cp++ = '\0';
	
	while (isspace(*cp))
	    ++cp;
	if (vendor)
	    *vendor = strdup(tv);
	if (model)
	    *model = strdup(cp);
    } else {
	if (vendor)
	    *vendor = strdup("?");
	if (model)
	    *model = strdup(tv);
    }

    if (revision)
	*revision = strdupget(((const char*)cdata.fr), NVME_FIRMWARE_REVISION_LENGTH);

    if (serial)
	*serial = strdupget(((const char*)cdata.sn), NVME_SERIAL_NUMBER_LENGTH);

    if (phys) {
	char pbuf[MAXPATHLEN];
	
	sprintf(pbuf, "pci vendor 0x%04x:0x%04x oui %02x:%02x:%02x controller 0x%04x",
		cdata.vid, cdata.ssvid,
		cdata.ieee[0], cdata.ieee[1], cdata.ieee[2],
		cdata.ctrlr_id);
	*phys = strdup(pbuf);
    }

#define NVME_OACS_DEVICE_SELF_TEST (1U << 4)
    
    uint16_t oacs = get_le16((const uint8_t *) &cdata.oacs);
    if ((oacs & NVME_OACS_DEVICE_SELF_TEST) == 0) {
	*have_selftest = 0;
    } else
	*have_selftest = 1;
    
    return 0;
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
int
nvme_smart_status(struct cam_device *dev,
		  DRIVE *dp) {
    union ccb *ccb;
    uint8_t health_log[NVME_HEALTH_LOG_SIZE];
    struct nvme_health_information_page health;
    uint32_t cam_status;
    uint32_t numd;
    uint8_t critical_warning;
    int saved_errno;
    uint64_t poh;
    uint16_t temp_c;
    

    memset(health_log, 0, sizeof(health_log));
    memset(&health, 0, sizeof(health));

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        errno = ENOMEM;
        return (-1);
    }

    cam_fill_nvmeadmin(&ccb->nvmeio,
		       1,                      /* retries */
		       NULL,                   /* callback */
		       CAM_DIR_IN,
		       (uint8_t *) &health,
		       sizeof(health),
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
    numd = sizeof(health) / sizeof(uint32_t) - 1;

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

    temp_c = get_le16((const uint8_t *) &health.temperature)-273;
    dp->raw.temperature = temp_c;
    if (f_debug)
	fprintf(stderr, "\tNVME Health: temperature     = %u\n", temp_c);
    

    critical_warning = health.critical_warning;

    if (f_debug) {
	fprintf(stderr, "\tNVME Health: percentage used = %u\n", health.percentage_used);
	fprintf(stderr, "\tNVME Health: power_cycles    = %zu\n", get_le64((const uint8_t *) &health.power_cycles));
	fprintf(stderr, "\tNVME Health: power_on_hours  = %zu\n", get_le64((const uint8_t *) &health.power_on_hours));
	fprintf(stderr, "\tNVME Health: media errors    = %zu\n", get_le64((const uint8_t *) &health.media_errors));
    }

    dp->raw.endurance_used = health.percentage_used;
    
    poh = get_le64((const uint8_t *) &health.power_on_hours);
    dp->raw.power_on_hours = poh;

    if (critical_warning == 0) {
        dp->raw.smart_status = DRIVE_SMART_PASSED;
        return (1);
    }

    /*
     * Treat read-only mode or degraded media reliability as failed.
     * Spare, temperature, or backup warnings remain warnings.
     */
    if ((critical_warning & ((1U << 2) | (1U << 3))) != 0)
        dp->raw.smart_status = DRIVE_SMART_FAILED;
    else
        dp->raw.smart_status = DRIVE_SMART_WARNING;

    return (1);
}



static enum drive_selftest_status
classify_nvme_selftest(uint8_t code)
{
    switch (code) {
    case 0x0:
        return (DRIVE_SELFTEST_PASSED);

    case 0x1:
    case 0x2:
    case 0x3:
    case 0x4:
    case 0x5:
    case 0x8:
    case 0x9:
        return (DRIVE_SELFTEST_ABORTED);

    case 0x6:
    case 0x7:
        return (DRIVE_SELFTEST_FAILED);

    case 0xf:
        return (DRIVE_SELFTEST_NONE);

    default:
        return (DRIVE_SELFTEST_UNKNOWN);
    }
}

/*
 * Read the most recent NVMe Device Self-test result through an
 * already-opened CAM nda device.
 */
int
nvme_selftest_status(struct cam_device *dev,
		     DRIVE *dp) {
    union ccb *ccb;
    uint8_t buffer[NVME_SELF_TEST_LOG_SIZE];
    uint8_t current_operation;
    uint8_t status;
    uint8_t result_code;
    uint32_t numd;
    uint32_t cam_status;
    int saved_errno;

    
    if (!dev || !dp) {
        errno = EINVAL;
        return (-1);
    }

    memset(buffer, 0, sizeof(buffer));

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        errno = ENOMEM;
        return (-1);
    }

    cam_fill_nvmeadmin(&ccb->nvmeio,
		       1,
		       NULL,
		       CAM_DIR_IN,
		       buffer,
		       sizeof(buffer),
		       10000);

    /*
     * NVMe Get Log Page, Device Self-test log 0x06.
     */
    ccb->nvmeio.cmd.opc = NVME_ADMIN_GET_LOG_PAGE;
    ccb->nvmeio.cmd.nsid = htole32(0xffffffffU);

    /*
     * NUMD is number of DWORDs minus one.
     *
     * 564 bytes / 4 = 141 DWORDs, therefore NUMD = 140.
     */
    numd = (sizeof(buffer) / sizeof(uint32_t)) - 1;

    ccb->nvmeio.cmd.cdw10 =
        htole32((numd << 16) | NVME_DEVICE_SELF_TEST_LOG);
    ccb->nvmeio.cmd.cdw11 = 0;

#if 0
    fprintf(stderr, "opc   = 0x%02x\n", ccb->nvmeio.cmd.opc);
    fprintf(stderr, "nsid  = 0x%08x\n", ccb->nvmeio.cmd.nsid);
    fprintf(stderr, "cdw10 = 0x%08x\n", ccb->nvmeio.cmd.cdw10);
    fprintf(stderr, "cdw11 = 0x%08x\n", ccb->nvmeio.cmd.cdw11);
    fprintf(stderr, "len   = %zu\n", sizeof(buffer));
#endif
    
    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;
        cam_freeccb(ccb);
        errno = saved_errno;
        return (-1);
    }

#if 0
    fprintf(stderr, "CAM status : %#x\n", ccb->ccb_h.status);
    fprintf(stderr, "cpl.cdw0   : %#08x\n", ccb->nvmeio.cpl.cdw0);
    fprintf(stderr, "cpl.sqhd   : %#04x\n", ccb->nvmeio.cpl.sqhd);
    fprintf(stderr, "cpl.sqid   : %#04x\n", ccb->nvmeio.cpl.sqid);
    fprintf(stderr, "cpl.cid    : %#04x\n", ccb->nvmeio.cpl.cid);
    fprintf(stderr, "cpl.status : %#04x\n", ccb->nvmeio.cpl.status);
#endif
    
    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;
    if (cam_status != CAM_REQ_CMP) {
        fprintf(stderr,
		"%s%u: NVMe Device Self-test log failed: "
		"CAM status 0x%x\n",
		dev->given_dev_name,
		dev->given_unit_number,
		cam_status);

#if 0
	fprintf(stderr, "ccb_h.status = 0x%08x\n", ccb->ccb_h.status);
	fprintf(stderr, "base status = 0x%08x\n", ccb->ccb_h.status & CAM_STATUS_MASK);
	fprintf(stderr, "CAMP_REQ_CMP = 0x%08x\n", CAM_REQ_CMP);

	struct nvme_completion *cpl = &ccb->nvmeio.cpl;
	fprintf(stderr, "CAM status = 0x%08x\n", ccb->ccb_h.status);
	fprintf(stderr, "NVMe cdw0 = 0x%08x\n", cpl->cdw0);
	fprintf(stderr, "NVMe status = 0x%04x\n", cpl->status);
#endif
	
        cam_freeccb(ccb);
        errno = EIO;
        return (-1);
    }

#if 0
    if (ccb->nvmeio.resid != 0) {
        fprintf(stderr,
		"%s%u: incomplete NVMe self-test log transfer, "
		"residual=%u\n",
		dev->given_dev_name,
		dev->given_unit_number,
		ccb->nvmeio.resid);

        cam_freeccb(ccb);
        errno = EIO;
        return (-1);
    }
#endif
    
    cam_freeccb(ccb);

    /*
     * Byte zero:
     *     bits 3:0 contain the current Device Self-test operation.
     */
    current_operation = buffer[0] & 0x0f;

    if (current_operation != 0) {
        dp->raw.selftest_status = DRIVE_SELFTEST_RUNNING;

        /*
         * Byte one is current completion percentage.
         * It is not stored in the common result structure.
         */
        return (0);
    }

    /*
     * The first result structure is the newest.
     *
     * Result byte:
     *     bits 3:0 = Device Self-test result
     *     bits 7:4 = Device Self-test code
     */
    status = buffer[NVME_SELF_TEST_HEADER_SIZE];

    result_code = status & 0x0f;
    dp->raw.selftest_status = classify_nvme_selftest(result_code);

#if 0
    fprintf(stderr, "** result->state = %d\n", result->state);
#endif
    
    if (dp->raw.selftest_status == DRIVE_SELFTEST_NONE)
        return (0);

    /*
     * Result entry layout:
     *
     * byte 0       status and test code
     * byte 1       segment number
     * byte 2       valid diagnostic information
     * byte 3       reserved
     * byte 4-11    power-on hours, little-endian
     * byte 12-15   namespace identifier
     * byte 16-23   failing LBA, little-endian
     * byte 24      status-code type
     * byte 25      status code
     * byte 26-27   vendor specific
     */
#if 0
    valid_diagnostic =
        buffer[NVME_SELF_TEST_HEADER_SIZE + 2];
#endif

#if 0
    /*
     * The NVMe field is 64-bit. The common structure retains only
     * 16 bits for compatibility with the SCSI/ATA display field.
     * Saturate instead of silently wrapping.
     */
    poh = get_le64(&buffer[NVME_SELF_TEST_HEADER_SIZE + 4]);
    dp->raw.power_on_hours = poh;
#endif
    
    return (1);
}

int
nvme_health_status(struct cam_device *dev,
		   DRIVE *dp) {
    int rc1 = 0, rc2 = 0;


    
    if (f_debug)
	fprintf(stderr, "  [smart & temp]\n");
    rc1 = nvme_smart_status(dev, dp);

    if (dp->flags.have_selftest) {
	if (f_debug)
	    fprintf(stderr, "  [selftest]\n");
	rc2 = nvme_selftest_status(dev, dp);
    }

    if (rc1 > 0 || rc2 > 0)
	return 1;
    if (rc1 == 0 && rc2 == 0)
	return 0;
    
    return -1;
}
