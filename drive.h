/* 
 * drive.h
 *
 * DRIVE types & defines
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


#ifndef DRVLIST_DRIVE_H
#define DRVLIST_DRIVE_H 1

#include <stdint.h>
#include <camlib.h>

enum drive_class {
    DRIVE_CLASS_UNKNOWN = 0,
    DRIVE_CLASS_SCSI    = 1,
    DRIVE_CLASS_ATA     = 2,
    DRIVE_CLASS_SATL    = 3, /* ATA drive on SAS/USB controller */
    DRIVE_CLASS_NVME    = 4,
    DRIVE_CLASS_ATAPI   = 5, /* CD, DVD */
    DRIVE_CLASS_MMCSD   = 6  /* MMC, SD, SDIO cards */
};

enum drive_smart_status {
    DRIVE_SMART_UNKNOWN = -1,
    DRIVE_SMART_NONE    =  0,
    DRIVE_SMART_PASSED  =  1,
    DRIVE_SMART_WARNING =  2,
    DRIVE_SMART_FAILED  =  3
};

enum drive_selftest_status {
    DRIVE_SELFTEST_UNKNOWN = -1,
    DRIVE_SELFTEST_NONE    =  0,
    DRIVE_SELFTEST_RUNNING =  1,
    DRIVE_SELFTEST_PASSED  =  2,
    DRIVE_SELFTEST_FAILED  =  3,
    DRIVE_SELFTEST_ABORTED =  4
};


typedef struct {
    struct cam_device *cam;
    
    struct {
	unsigned int have_smart : 1;
	unsigned int have_selftest : 1;
    } flags;
    
    /* Printable information */
    char *ident;     /* Unique ID (serial number) */
    char *name;      /* da0, ada0, nda0 */
    char *vendor;    /* INTEL, SEAGATE, WDC, HGST */
    char *product;   
    char *version;   /* Firmware version */
    char *ctrl;      /* Controller */
    char *path;
    char *phys;
    char *proto;
    char *class;
    char *xport;
    char *size;
    char *temp;
    char *smart;
    char *tests;
    char *hours;
    char *wear;
    char *made;

    /* Binary raw data */
    struct {
	enum drive_class class;
	cam_xport xport;
	enum drive_smart_status smart_status;
	struct {
	    enum drive_selftest_status status;
	    uint16_t tests;
	    uint16_t running;
	    uint16_t passed;
	    uint16_t failed;
	    uint16_t aborted;
	} selftest;
	uint64_t power_on_hours;
	uint64_t last_self_test;
	uint16_t endurance_used;
	uint16_t temperature;
        struct {
	    struct {
		uint64_t specified;
		uint64_t accumulated;
	    } start_stop, load_unload;
	} cycles;
    } raw;
} DRIVE;

#define DRIVE_NO_TEMPERATURE    0xffff
#define DRIVE_NO_ENDURANCE_USED 0xffff
#define DRIVE_NO_HOURS          0xffffffffffffffff

extern DRIVE *dv;
extern int dc;
extern int ds;

extern void
drive_init(DRIVE *dp);

extern const char *
drive_cam_devtype2str(uint8_t devtype);

extern const char *
drive_class2str(enum drive_class proto);

extern const char *
drive_cam_proto2str(cam_proto proto);

extern const char *
drive_cam_xport2str(cam_xport xport);

extern const char *
drive_smart_status2str(enum drive_smart_status status);

extern const char *
drive_selftest_status2str(enum drive_selftest_status status);

extern int
drive_classify(DRIVE *dp);

#endif
