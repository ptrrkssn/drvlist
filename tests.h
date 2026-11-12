/*
 * tests.h
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

#ifndef DRVLIST_TESTS_H
#define DRVLIST_TESTS_H 1

enum scsi_tests_state {
    SCSI_TESTS_UNKNOWN = -1,
    SCSI_TESTS_NONE = 0,
    SCSI_TESTS_PASSED,
    SCSI_TESTS_FAILED,
    SCSI_TESTS_ABORTED,
    SCSI_TESTS_IN_PROGRESS
};

struct scsi_tests_result {
    enum scsi_tests_state state;

    uint16_t parameter_code;
    uint8_t test_code;
    uint8_t result_code;

    uint16_t power_on_hours;
    uint64_t failing_lba;

    uint8_t sense_key;
    uint8_t asc;
    uint8_t ascq;
};

extern const char *
scsi_tests_state_name(enum scsi_tests_state state);

extern const char *
scsi_tests_result_name(uint8_t code);

extern enum scsi_tests_state
classify_tests_result(uint8_t code);

extern int
scsi_tests_read_last(struct cam_device *dev,
		     struct scsi_tests_result *result);

#endif
