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
#include <cam/ata/ata_all.h>
#include <cam/scsi/scsi_all.h>
#include <camlib.h>
#include <dev/nvme/nvme.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utils.h"
#include "tests.h"


#define SCSI_LOG_SENSE                        0x4d
#define TESTS_RESULTS_PAGE            	      0x10

#define SELFTEST_LOG_BUFFER_SIZE              4096
#define SELFTEST_PARAMETER_DATA_LENGTH        16

/*
 * Four-byte log-parameter header plus 16 bytes of self-test data.
 */
#define SELFTEST_PARAMETER_TOTAL_LENGTH       20



const char *
tests_result_code2str(uint8_t code) {
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
        return ("self-test running");
    case 0xf:
        return ("no self-test has been run");
    default:
        return ("reserved result code");
    }
}



void
tests_result_init(struct tests_result *result) {
    memset(result, 0, sizeof(*result));
    result->state = HEALTH_TESTS_UNKNOWN;
}
