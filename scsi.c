/* XXX: Use FORMAT STATUS LOG PAGE to get power-on-hours since last format */

/*
 * scsi.c
 *
 * SCSI/SAS-specific code to handle "da" drives
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
#include <camlib.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "utils.h"

#include "health.h"

#include "smart.h"
#include "tests.h"

#include "scsi.h"

extern int f_debug;
extern char *argv0;


#define SCSI_LOG_SENSE               0x4D
#define SCSI_TEMP_LOG_PAGE           0x0D
#define SCSI_SELFTEST_RESULTS_PAGE   0x10
#define SCSI_IE_LOG_PAGE             0x2F


struct log_sense_cdb {
    uint8_t opcode;
    uint8_t byte2;
    uint8_t page;
    uint8_t subpage;
    uint8_t reserved[3];
    uint8_t alloc_len[2];
    uint8_t control;
};




ssize_t
scsi_cmd_log_sense(struct cam_device *dev,
		   uint8_t page_code,
		   uint8_t *buf,
		   size_t bufsize) {
    union ccb *ccb;
    struct log_sense_cdb *cdb;
    ssize_t len;
    size_t got;
    size_t cam_resid;
    uint32_t cam_status;
    int saved_errno;
    

    ccb = cam_getccb(dev);
    if (!ccb)
	return -1;

    memset(buf, 0, bufsize);

    cam_fill_csio(&ccb->csio,
        1,                      /* retries */
        NULL,                   /* callback */
        CAM_DIR_IN,
	CAM_TAG_ACTION_NONE,
        buf,
        bufsize,
        SSD_FULL_SIZE,
	sizeof(*cdb),           /* CDB length */
        5000);                  /* timeout ms */
    
    cdb = (struct log_sense_cdb *)&ccb->csio.cdb_io.cdb_bytes;
    memset(cdb, 0, sizeof(*cdb));

    cdb->page = (1 << 6) | page_code;
    cdb->opcode = SCSI_LOG_SENSE;
    put_be16(cdb->alloc_len, bufsize);
    
    if (cam_send_ccb(dev, ccb) < 0) {
	saved_errno = errno;
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE failed: cam_send_ccb: %s\n",
		    argv0, dev->given_dev_name, dev->given_unit_number,
		    strerror(errno));
	
        cam_freeccb(ccb);
	errno = saved_errno;
        return -1;
    }

    cam_status = (ccb->ccb_h.status & CAM_STATUS_MASK);
    cam_resid = ccb->csio.resid;
    
    cam_freeccb(ccb);
    
    if (cam_status != CAM_REQ_CMP) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE failed: CAM status 0x%x\n",
		    argv0, dev->given_dev_name, dev->given_unit_number,
		    cam_status);
	errno = EBUSY;
        return -1;
    }

    if (cam_resid < 0 ||
	(size_t)ccb->csio.resid > bufsize) {
	errno = EIO;
	return -1;
    }
	
    /*
     * Determine how many bytes CAM actually transferred.
     */
    if (ccb->csio.resid < 0 ||
        (size_t)ccb->csio.resid > bufsize) {
        cam_freeccb(ccb);
        errno = EIO;
        return -1;
    }
    got = bufsize - (size_t)ccb->csio.resid;

    if (got < 4) {
	fprintf(stderr, "%s: Error: %s%u: LOG SENSE failed: Truncated LOG SENSE page\n",
		argv0, dev->given_dev_name, dev->given_unit_number);
	errno = EIO;
	return -1;
    }

    /*
     * LOG SENSE page header:
     *
     * byte 0   page code
     * byte 1   subpage/reserved
     * byte 2-3 page length
     */

    if ((buf[0] & 0x3f) != page_code) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE failed: 0x%02x: Invalid Page Code\n",
		    argv0, dev->given_dev_name, dev->given_unit_number,
		    (buf[0] & 0x3f));

	errno = EINVAL;
	return -1;
    }
    
    len = get_be16(&buf[2]) + 4;

    if (len > bufsize) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE buffer too small: len=%zd > bufsize = %zd\n",
		    argv0, dev->given_dev_name, dev->given_unit_number,
		    len, bufsize);
	errno = EINVAL;
	return -1;
    }
    
    if (got != len) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE buffer length inconsistent: len=%zd vs got=%zd\n",
		    argv0, dev->given_dev_name, dev->given_unit_number,
		    len, got);
	errno = EINVAL;
	return -1;
    }
    
    return len;
}
	       


int
scsi_drive_temperature(struct cam_device *dev,
		       double *temp_c) {
    uint8_t buf[252];
    ssize_t len;
    off_t off;
    int rc = -1;
    
    
    len = scsi_cmd_log_sense(dev, SCSI_TEMP_LOG_PAGE, buf, sizeof(buf));
    if (len < 0)
	return -1;
    
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
        uint16_t parameter = get_be16(&buf[off]);
        uint8_t plen = buf[off + 3];

        if (off + 4 + plen > len)
            break;

	if (parameter == 0x0000 && plen >= 2) {
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

	    /* We ignore other temp & humidity parameters */
	    return rc;
	}

        off += 4 + plen;
    }

    return rc;
    
}



/*
 * SCSI/SAS: Informational Exceptions log page 0x2f
 *
 * Parameter code 0 contains the device's informational-exception
 * additional sense code and qualifier.
 */
int
scsi_smart_status(struct cam_device *dev,
		  struct smart_result *result) {
    uint8_t buf[252];
    ssize_t len;
    off_t off;
    
    smart_result_init(result);
    len = scsi_cmd_log_sense(dev, SCSI_IE_LOG_PAGE, buf, sizeof(buf));
    if (len < 0)
	return -1;
    
    /*
     * Parameter descriptor:
     *
     * byte 0-1: parameter code
     * byte 2:   parameter control
     * byte 3:   parameter length
     * byte 4..: parameter data
     */
    off = 4;

    while (off + 4 <= len) {
        uint16_t parameter_code;
        uint8_t parameter_length;
        uint8_t asc;
        uint8_t ascq;

        parameter_code = get_be16(&buf[off]);
        parameter_length = buf[off + 3];

        if (off + 4 + parameter_length > len)
            break;

        if (parameter_code == 0 && parameter_length >= 2) {
            asc = buf[off + 4];
            ascq = buf[off + 5];

            result->detail1 = asc;
            result->detail2 = ascq;
	    
	    result->temp_c = buf[off+6];
	    

            /*
             * No informational exception is currently reported.
             */
            if (asc == 0 && ascq == 0) {
                result->health = HEALTH_SMART_PASSED;
		result->protocol = HEALTH_PROTOCOL_SCSI;
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
                result->health = HEALTH_SMART_FAILED;
		result->protocol = HEALTH_PROTOCOL_SCSI;
                return (1);
            }

            /*
             * The device reports another informational exception.
             * Preserve ASC/ASCQ and report it as a warning.
             */
            result->health = HEALTH_SMART_WARNING;
	    result->protocol = HEALTH_PROTOCOL_SCSI;
            return (1);
        }

        off += 4 + parameter_length;
    }

    fprintf(stderr,
        "%s%u: informational-exceptions parameter not found\n",
        dev->given_dev_name,
        dev->given_unit_number);

    errno = EIO;
    return (-1);
}




#define SELFTEST_LOG_BUFFER_SIZE              4096
#define SELFTEST_PARAMETER_DATA_LENGTH        16

/*
 * Four-byte log-parameter header plus 16 bytes of self-test data.
 */
#define SELFTEST_PARAMETER_TOTAL_LENGTH       20

static enum health_tests_status
scsi_classify_tests_result(uint8_t code) {
    switch (code) {
    case 0x0:
        return (HEALTH_TESTS_PASSED);

    case 0x1:
    case 0x2:
        return (HEALTH_TESTS_ABORTED);

    case 0x3:
    case 0x4:
    case 0x5:
    case 0x6:
    case 0x7:
    case 0x8:
        return (HEALTH_TESTS_FAILED);

    case 0x9:
        return (HEALTH_TESTS_RUNNING);

    case 0xf:
        return (HEALTH_TESTS_NONE);

    default:
        return (HEALTH_TESTS_UNKNOWN);
    }
}

/*
 * Read the newest entry from SCSI Self-test Results log page 0x10.
 */
int
scsi_tests_status(struct cam_device *dev,
		  struct tests_result *result) {
    uint8_t buf[SELFTEST_LOG_BUFFER_SIZE];
    ssize_t len;
    off_t off;
    int rc = -1;
    
    
    memset(result, 0, sizeof(*result));
    result->state = HEALTH_TESTS_UNKNOWN;
	

    len = scsi_cmd_log_sense(dev, SCSI_SELFTEST_RESULTS_PAGE, buf, sizeof(buf));
    if (len < 0)
	return -1;

    off = 4;
    while (off + 4 <= len) {
        uint16_t parameter_code;
        uint8_t parameter_length;
        const uint8_t *data;
        uint8_t status_byte;

        parameter_code = get_be16(&buf[off]);
        parameter_length = buf[off + 3];

        if (off + 4 + parameter_length > len) {
            fprintf(stderr,
                "%s%u: truncated self-test parameter 0x%04x\n",
                dev->given_dev_name,
                dev->given_unit_number,
                parameter_code);

            errno = EIO;
            return -1;
        }

        /*
         * Self-test result descriptors contain 16 data bytes.
         * Parameter code 1 is normally the newest entry. If the
         * drive starts numbering differently, the first valid
         * descriptor is still treated as the newest.
         */
        if (parameter_length >= SELFTEST_PARAMETER_DATA_LENGTH) {
            data = &buf[off + 4];
	    uint16_t poh;
	    
            /*
             * Descriptor byte zero:
             *
             * bits 7-5  self-test code
             * bit  4    reserved
             * bits 3-0  self-test result
             */
            status_byte = data[0];

            result->parameter_code = parameter_code;
            result->test_code = (status_byte >> 5) & 0x07;
            result->result_code = status_byte & 0x0f;

            /*
             * Bytes 2-3: accumulated power-on hours.
             */

	    poh = get_be16(&data[2]);
	    if (poh > result->power_on_hours)
		result->power_on_hours = poh;

            /*
             * Bytes 4-11: address of first failing logical block.
             *
             * A drive commonly returns all ones when no failing LBA
             * applies.
             */
            result->failing_lba = get_be64(&data[4]);

            /*
             * Byte 12: sense key in low nibble.
             * Bytes 13-14: ASC and ASCQ.
             */
            result->details.scsi.sense_key = data[12] & 0x0f;
            result->details.scsi.asc = data[13];
            result->details.scsi.ascq = data[14];

            result->state =
                scsi_classify_tests_result(result->result_code);

	    result->details.scsi.tests.done++;
	    
	    fprintf(stderr, "%s%u: Test %lu PowerOnHours %u [%u] State %d\n",
		    dev->given_dev_name,
		    dev->given_unit_number,
		    result->details.scsi.tests.done,
		    poh, result->power_on_hours,
		    result->state);

	    switch (result->state) {
	    case HEALTH_TESTS_PASSED:
		result->details.scsi.tests.passed++;
		break;
	    case HEALTH_TESTS_FAILED:
		result->details.scsi.tests.failed++;
		break;
	    default:
		break;
	    }
	    rc = 1;
        }

        off += 4 + parameter_length;
    }

    /*
     * A valid but empty page means no recorded self-test result.
     */
    if (result->state == HEALTH_TESTS_UNKNOWN && result->details.scsi.tests.done == 0)
	result->state = HEALTH_TESTS_NONE;
    
    return rc;
}





int
scsi_health_status(struct cam_device *dev,
		   struct health_result *result,
		   int have_selftest) {
    struct smart_result smart;
    struct tests_result tests;
    int rc1, rc2, rc3;


    memset(result, 0, sizeof(*result));
    
    rc1 = scsi_drive_temperature(dev, &result->drive.temperature);
    if (rc1 < 0)
	return rc1;
    
    rc2 = scsi_smart_status(dev, &smart);
    if (rc2 >= 0) {
	result->protocol = HEALTH_PROTOCOL_SCSI;
	result->smart.status = smart.health;
	result->drive.power_on_hours = smart.power_on_hours;
    }
    
    rc3 = scsi_tests_status(dev, &tests);
    if (rc3 >= 0) {
	result->tests.status = tests.state;
	result->drive.power_on_hours = tests.power_on_hours;
    }

    if (rc1 > 0 || rc2 > 0)
	return 1;
    if (rc1 == 0 && rc2 == 0)
	return 0;
    
    return -1;
}
