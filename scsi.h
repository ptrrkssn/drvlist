/*
 * scsi.h
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

#ifndef DRVLIST_SCSI_H
#define DRVLIST_SCSI_H 1

#include <cam/cam.h>

#include "drive.h"


#define INQUIRY_EVPD 0x0100

extern int
drive_scsi_parse_inquiry(DRIVE *dp,
			 uint8_t *buffer,
			 size_t bufsize);

extern const char *
scsi_inquiry_version2str(uint8_t version);

extern ssize_t
drive_scsi_inquiry(DRIVE *dp,
		   uint16_t page,
		   void *buf,
		   size_t bufsize);

extern int
drive_scsi_has_vpd_page(DRIVE *dp,
			uint8_t page_id);

extern ssize_t
drive_scsi_log_sense(DRIVE *dp,
		     uint8_t page,
		     uint8_t subpage,
		     uint8_t *buf,
		     size_t bufsize);

#if 0
extern int
drive_scsi_supported_log_pages(DRIVE *dp,
			       struct supported_log_pages *pages);
#endif


extern int
drive_scsi_temperature(DRIVE *dp);

extern int
drive_scsi_ie_status(DRIVE *dp);

extern int
drive_scsi_selftest_results_status(DRIVE *dp);

extern int
drive_scsi_start_stop_cycle(DRIVE *dp);

extern int
drive_scsi_format_status_poweron(DRIVE *dp);

extern int
drive_scsi_endurance_used(DRIVE *dp);

extern int
drive_scsi_is_satl(DRIVE *dp);


extern int
drive_scsi_health_status(DRIVE *dp);

#endif
