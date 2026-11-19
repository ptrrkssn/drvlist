/* 
 * drive.c
 *
 * DRIVE functions
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

#include <string.h>

#include "drive.h"

#define MAXDRIVE 2048

extern char *f_sort;

DRIVE *dv;
int dc = 0;
int ds = 0;


void
drive_init(DRIVE *dp) {
    memset(dp, 0, sizeof(*dp));
    dp->type = DRIVE_TYPE_UNKNOWN;
    dp->raw.power_on_hours = DRIVE_NO_POWER_ON_HOURS;
    dp->raw.endurance_used = DRIVE_NO_ENDURANCE_USED;
    dp->raw.temperature    = DRIVE_NO_TEMPERATURE;
}

static int
drive_sort_ident(const DRIVE *da,
		 const DRIVE *db) {
    return strcmp(da->ident, db->ident);
}

static int
drive_sort_devpath(const DRIVE *da,
		   const DRIVE *db) {
    int d;

    d = strcmp(da->driver, db->driver);
    if (d)
	return d;
    
    return strcmp(da->path, db->path);
}

int
drive_sort(const void *a,
	   const void *b) {
    if (f_sort) {
	if (strcmp(f_sort, "ident") == 0)
	    return drive_sort_ident((const DRIVE *) a, (const DRIVE *) b);
    }
    
    return drive_sort_devpath((const DRIVE *) a, (const DRIVE *) b);
}



const char *
drive_smart_status2str(enum drive_smart_status status) {
    switch (status) {
    case DRIVE_SMART_UNKNOWN:
	return "UNKNOWN";
    case DRIVE_SMART_PASSED:
        return "PASSED";
    case DRIVE_SMART_NONE:
        return "-";
    case DRIVE_SMART_WARNING:
        return "WARNING";
    case DRIVE_SMART_FAILED:
        return "FAILED";
    }

    return "?";
}

const char *
drive_selftest_status2str(enum drive_selftest_status status) {
    switch (status) {
    case DRIVE_SELFTEST_UNKNOWN:
	return "UNKNOWN";
    case DRIVE_SELFTEST_NONE:
	return "-";
    case DRIVE_SELFTEST_RUNNING:
	return "RUNNING";
    case DRIVE_SELFTEST_PASSED:
	return "PASSED";
    case DRIVE_SELFTEST_FAILED:
	return "FAILED";
    case DRIVE_SELFTEST_ABORTED:
	return "ABORTED";
    }
    return "?";
}

const char *
drive_type2str(enum drive_type type) {
    switch (type) {
    case DRIVE_TYPE_UNKNOWN:
	return "?";
    case DRIVE_TYPE_SCSI:
	return "SCSI";
    case DRIVE_TYPE_ATA:
	return "ATA";
    case DRIVE_TYPE_SATL:
	return "SATL";
    case DRIVE_TYPE_NVME:
	return "NVME";
    case DRIVE_TYPE_ATAPI:
	return "ATAPI";
    case DRIVE_TYPE_MMCSD:
	return "MMCSD";
    }
}
