/*
 * tests.c
 *
 * Code to read selftest log status from SAS/SCSI (da) drives
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
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tests.h"

#define SCSI_LOG_SENSE                        0x4d
#define SCSI_TESTS_RESULTS_PAGE            0x10

#define SELFTEST_LOG_BUFFER_SIZE              4096
#define SELFTEST_PARAMETER_DATA_LENGTH        16

/*
 * Four-byte log-parameter header plus 16 bytes of self-test data.
 */
#define SELFTEST_PARAMETER_TOTAL_LENGTH       20


static uint16_t
decode_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) |
        (uint16_t)p[1];
}

static uint64_t
decode_be64(const uint8_t *p)
{
    return ((uint64_t)p[0] << 56) |
        ((uint64_t)p[1] << 48) |
        ((uint64_t)p[2] << 40) |
        ((uint64_t)p[3] << 32) |
        ((uint64_t)p[4] << 24) |
        ((uint64_t)p[5] << 16) |
        ((uint64_t)p[6] << 8) |
        (uint64_t)p[7];
}

const char *
scsi_tests_state_name(enum scsi_tests_state state)
{
    switch (state) {
    case SCSI_TESTS_NONE:
        return ("NONE");
    case SCSI_TESTS_PASSED:
        return ("PASSED");
    case SCSI_TESTS_FAILED:
        return ("FAILED");
    case SCSI_TESTS_ABORTED:
        return ("ABORTED");
    case SCSI_TESTS_IN_PROGRESS:
        return ("RUNNING");
    case SCSI_TESTS_UNKNOWN:
    default:
        return ("UNKNOWN");
    }
}

const char *
scsi_tests_result_name(uint8_t code)
{
    switch (code) {
    case 0x0:
        return ("completed without error");
    case 0x1:
        return ("aborted by SEND DIAGNOSTIC");
    case 0x2:
        return ("aborted by another command");
    case 0x3:
        return ("unknown error prevented completion");
    case 0x4:
        return ("failed in an unknown test segment");
    case 0x5:
        return ("failed in electrical segment");
    case 0x6:
        return ("failed in servo segment");
    case 0x7:
        return ("failed in read segment");
    case 0x8:
        return ("failed due to handling damage");
    case 0x9:
        return ("self-test in progress");
    case 0xf:
        return ("no self-test has been run");
    default:
        return ("reserved result code");
    }
}

enum scsi_tests_state
classify_tests_result(uint8_t code)
{
    switch (code) {
    case 0x0:
        return (SCSI_TESTS_PASSED);

    case 0x1:
    case 0x2:
        return (SCSI_TESTS_ABORTED);

    case 0x3:
    case 0x4:
    case 0x5:
    case 0x6:
    case 0x7:
    case 0x8:
        return (SCSI_TESTS_FAILED);

    case 0x9:
        return (SCSI_TESTS_IN_PROGRESS);

    case 0xf:
        return (SCSI_TESTS_NONE);

    default:
        return (SCSI_TESTS_UNKNOWN);
    }
}

/*
 * Read the newest entry from SCSI Self-test Results log page 0x10.
 *
 * dev:
 *     An already-opened CAM device corresponding to a native SCSI/SAS
 *     direct-access device, normally /dev/daN.
 *
 * result:
 *     Receives the decoded newest self-test entry.
 *
 * Returns:
 *      0  The log page was read and decoded. Inspect result->state.
 *     -1  Command, transport, or decoding failure. errno is set.
 */
int
scsi_tests_read_last(struct cam_device *dev,
			struct scsi_tests_result *result) {
    union ccb *ccb;
    uint8_t buffer[SELFTEST_LOG_BUFFER_SIZE];
    uint8_t *cdb;
    uint32_t cam_status;
    size_t transferred;
    size_t page_length;
    size_t available_length;
    size_t offset;
    int saved_errno;

    if (dev == NULL || result == NULL) {
        errno = EINVAL;
        return (-1);
    }

    memset(result, 0, sizeof(*result));
    result->state = SCSI_TESTS_UNKNOWN;

    memset(buffer, 0, sizeof(buffer));

    ccb = cam_getccb(dev);
    if (ccb == NULL) {
        errno = ENOMEM;
        return -1;
    }

    cam_fill_csio(&ccb->csio,
        1,                          /* retries */
        NULL,                       /* completion callback */
        CAM_DIR_IN,
        CAM_TAG_ACTION_NONE,
        buffer,
        sizeof(buffer),
        SSD_FULL_SIZE,
        10,                         /* LOG SENSE(10) CDB */
        10000);                     /* timeout in milliseconds */

    cdb = ccb->csio.cdb_io.cdb_bytes;
    memset(cdb, 0, 10);

    /*
     * LOG SENSE(10):
     *
     * byte 0    operation code 0x4d
     * byte 2    PC=01b, current cumulative values
     *           page code=0x10, self-test results
     * byte 3    subpage code=0
     * byte 7-8  allocation length
     */
    cdb[0] = SCSI_LOG_SENSE;
    cdb[2] = 0x40 | SCSI_TESTS_RESULTS_PAGE;
    cdb[3] = 0;
    cdb[7] = (uint8_t)(sizeof(buffer) >> 8);
    cdb[8] = (uint8_t)(sizeof(buffer) & 0xff);

    if (cam_send_ccb(dev, ccb) < 0) {
        saved_errno = errno != 0 ? errno : EIO;

        fprintf(stderr, "%s%u: LOG SENSE failed: %s\n",
            dev->given_dev_name,
            dev->given_unit_number,
            strerror(saved_errno));

        cam_freeccb(ccb);
        errno = saved_errno;
        return -1;
    }

    cam_status = ccb->ccb_h.status & CAM_STATUS_MASK;

    if (cam_status != CAM_REQ_CMP) {
        fprintf(stderr,
            "%s%u: Self-test Results LOG SENSE failed: "
            "CAM status 0x%x, SCSI status 0x%x\n",
            dev->given_dev_name,
            dev->given_unit_number,
            cam_status,
            ccb->csio.scsi_status);

        cam_freeccb(ccb);
        errno = EIO;
        return -1;
    }

    /*
     * Determine how many bytes CAM actually transferred.
     */
    if (ccb->csio.resid < 0 ||
        (size_t)ccb->csio.resid > sizeof(buffer)) {
        cam_freeccb(ccb);
        errno = EIO;
        return -1;
    }

    transferred = sizeof(buffer) - (size_t)ccb->csio.resid;

    cam_freeccb(ccb);

    if (transferred < 4) {
        fprintf(stderr,
            "%s%u: truncated self-test log-page header\n",
            dev->given_dev_name,
            dev->given_unit_number);

        errno = EIO;
        return -1;
    }

    /*
     * Check the returned page code. Bit 6 may contain the SPF flag,
     * so only compare the low six page-code bits.
     */
    if ((buffer[0] & 0x3f) != SCSI_TESTS_RESULTS_PAGE) {
        fprintf(stderr,
            "%s%u: device returned log page 0x%02x, expected 0x10\n",
            dev->given_dev_name,
            dev->given_unit_number,
            buffer[0] & 0x3f);

        errno = EIO;
        return -1;
    }

    /*
     * The page length excludes the four-byte page header.
     */
    page_length = (size_t)decode_be16(&buffer[2]);
    available_length = page_length + 4;

    if (available_length > transferred)
        available_length = transferred;

    /*
     * Parameter descriptors start at offset four:
     *
     * bytes 0-1  parameter code
     * byte  2    parameter control byte
     * byte  3    parameter data length
     * bytes 4..  parameter data
     */
    offset = 4;

    while (offset + 4 <= available_length) {
        uint16_t parameter_code;
        uint8_t parameter_length;
        const uint8_t *data;
        uint8_t status_byte;

        parameter_code = decode_be16(&buffer[offset]);
        parameter_length = buffer[offset + 3];

        if (offset + 4 + parameter_length > available_length) {
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
            data = &buffer[offset + 4];

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
            result->power_on_hours = decode_be16(&data[2]);

            /*
             * Bytes 4-11: address of first failing logical block.
             *
             * A drive commonly returns all ones when no failing LBA
             * applies.
             */
            result->failing_lba = decode_be64(&data[4]);

            /*
             * Byte 12: sense key in low nibble.
             * Bytes 13-14: ASC and ASCQ.
             */
            result->sense_key = data[12] & 0x0f;
            result->asc = data[13];
            result->ascq = data[14];

            result->state =
                classify_tests_result(result->result_code);

            return 1;
        }

        offset += 4 + parameter_length;
    }

    /*
     * A valid but empty page means no recorded self-test result.
     */
    result->state = SCSI_TESTS_NONE;
    return 0;
}


#if 0
int
main(int argc, char **argv)
{
    struct cam_device *dev;
    struct scsi_tests_result result;

    if (argc != 2) {
        fprintf(stderr, "usage: %s /dev/daN\n", argv[0]);
        return (EXIT_FAILURE);
    }

    dev = cam_open_device(argv[1], O_RDWR);
    if (dev == NULL) {
        fprintf(stderr, "cam_open_device(%s): %s\n",
            argv[1], cam_errbuf);
        return (EXIT_FAILURE);
    }

    if (strcmp(dev->given_dev_name, "da") != 0) {
        fprintf(stderr,
            "%s is not a da device; CAM identifies it as %s%u\n",
            argv[1],
            dev->given_dev_name,
            dev->given_unit_number);

        cam_close_device(dev);
        return (EXIT_FAILURE);
    }

    if (scsi_tests_read_last(dev, &result) < 0) {
        fprintf(stderr, "%s: cannot read self-test log: %s\n",
            argv[1], strerror(errno));

        cam_close_device(dev);
        return (EXIT_FAILURE);
    }

    cam_close_device(dev);

    printf("%s: last self-test %s\n",
        argv[1],
        scsi_tests_state_name(result.state));

    if (result.state != SCSI_TESTS_NONE) {
        printf("  parameter:      0x%04x\n",
            result.parameter_code);
        printf("  test code:      0x%x\n",
            result.test_code);
        printf("  result code:    0x%x (%s)\n",
            result.result_code,
            scsi_tests_result_name(result.result_code));
        printf("  power-on hours: %u\n",
            result.power_on_hours);

        if (result.failing_lba != UINT64_MAX) {
            printf("  failing LBA:    %" PRIu64 "\n",
                result.failing_lba);
        }

        if (result.sense_key != 0 ||
            result.asc != 0 ||
            result.ascq != 0) {
            printf("  sense:          key=0x%x ASC=0x%02x "
                "ASCQ=0x%02x\n",
                result.sense_key,
                result.asc,
                result.ascq);
        }
    }

    switch (result.state) {
    case SCSI_TESTS_PASSED:
    case SCSI_TESTS_NONE:
        return (EXIT_SUCCESS);

    case SCSI_TESTS_FAILED:
    case SCSI_TESTS_ABORTED:
    case SCSI_TESTS_IN_PROGRESS:
    case SCSI_TESTS_UNKNOWN:
    default:
        return (EXIT_FAILURE);
    }
}
#endif
