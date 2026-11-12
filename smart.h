/*
 * smart.h
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

#ifndef DRVLIST_SMART_H
#define DRVLIST_SMART_H 1

#include <stdint.h>
#include <cam/cam.h>

/*
 * Health result returned by the functions below.
 *
 * UNKNOWN means that the command failed, the requested health facility
 * was unavailable, or the response could not be interpreted.
 */
enum smart_health {
    SMART_HEALTH_UNKNOWN = -1,
    SMART_HEALTH_PASSED  = 0,
    SMART_HEALTH_WARNING = 1,
    SMART_HEALTH_FAILED  = 2
};

enum smart_protocol {
    SMART_PROTOCOL_ATA = 0,
    SMART_PROTOCOL_SCSI = 1,
    SMART_PROTOCOL_NVME = 2
};

struct smart_result {
    enum smart_health health;
    enum smart_protocol protocol;
    
    /*
     * Protocol-specific detail:
     *
     * ATA:
     *     detail1 = returned LBA mid register
     *     detail2 = returned LBA high register
     *
     * SCSI:
     *     detail1 = ASC
     *     detail2 = ASCQ
     *
     * NVMe:
     *     detail1 = Critical Warning bit mask
     *     detail2 = 0
     */
    uint8_t detail1;
    uint8_t detail2;
};


extern int
cam_smart_status(struct cam_device *dev,
		 struct smart_result *result);

extern const char *
smart_health_name(enum smart_health health);

#endif
