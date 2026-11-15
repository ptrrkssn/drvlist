/*
 * health.h
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

#ifndef DRVLIST_HEALTH_H
#define DRVLIST_HEALTH_H 1

#include <stdint.h>

enum health_protocol {
    HEALTH_PROTOCOL_UNKNOWN = -1,
    HEALTH_PROTOCOL_ATA = 0,
    HEALTH_PROTOCOL_SCSI = 1,
    HEALTH_PROTOCOL_NVME = 2
};

enum health_smart_status {
    HEALTH_SMART_UNKNOWN = -1,
    HEALTH_SMART_NONE = 0,
    HEALTH_SMART_PASSED  = 1,
    HEALTH_SMART_WARNING = 2,
    HEALTH_SMART_FAILED  = 3
};

enum health_tests_status {
    HEALTH_TESTS_UNKNOWN = -1,
    HEALTH_TESTS_NONE = 0,
    HEALTH_TESTS_RUNNING = 1,
    HEALTH_TESTS_PASSED = 2,
    HEALTH_TESTS_FAILED = 3,
    HEALTH_TESTS_ABORTED = 4
};

struct health_result {
    enum health_protocol protocol;
    struct {
	double   temperature;
	uint64_t power_on_hours;
    } drive;
    struct {
	enum health_smart_status status;
    } smart;
    struct {
	enum health_tests_status status;
    } tests;
    union {
	struct {
	    uint8_t sense_key;
	    uint8_t asc;
	    uint8_t ascq;
	} scsi;
#if 0	
	struct {
	} ata;
	struct {
	} nvme;
#endif
    } details;
};    

extern const char *
health_smart_status2str(enum health_smart_status status);

extern const char *
health_tests_status2str(enum health_tests_status status);

#endif
    
    
	
