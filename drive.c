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

#include <errno.h>
#include <string.h>

#include "drive.h"
#include "scsi.h"
#include "satl.h"
#include "utils.h"

#define MAXDRIVE 2048

extern char *argv0;
extern char *f_sort;
extern int f_debug;

DRIVE *dv;
int dc = 0;
int ds = 0;


extern const char *
drive_cam_devtype2str(uint8_t devtype) {
    switch (devtype) {
    case T_DIRECT:
	return "DIRECT";
    case T_SEQUENTIAL:
	return "SEQUENTIAL";
    case T_PRINTER:
	return "PRINTER";
    case T_PROCESSOR:
	return "PROCESSOR";
    case T_WORM:
	return "WORM";
    case T_CDROM:
	return "CDROM";
    case T_SCANNER:
	return "SCANNER";
    case T_OPTICAL:
	return "OPTICAL";
    case T_CHANGER:
	return "CHANGER";
    case T_COMM:
	return "COMM";
    case T_ASC0:
	return "ASC0";
    case T_ASC1:
	return "ASC1";
    case T_STORARRAY:
	return "STORARRAY";
    case T_ENCLOSURE:
	return "ENCLOSURE";
    case T_RBC:
	return "RBC";
    case T_OCRW:
	return "OCRW";
    case T_OSD:
	return "OSD";
    case T_ADC:
	return "ADC";
    case T_ZBC_HM:
	return "ZBC_HM";
    case T_NODEVICE:
	return "-";
    case T_ANY:
	return "ANY";
    default:
	return "?";
    }
}

const char *
drive_cam_proto2str(cam_proto proto) {
    switch (proto) {
    case PROTO_UNKNOWN:
	return "?";
	
    case PROTO_UNSPECIFIED:
	return "-";
	
    case PROTO_SCSI:
	return "SCSI";
	
    case PROTO_ATA:
	return "ATA";

    case PROTO_ATAPI:
	return "ATAPI";
	
    case PROTO_SATAPM:
	return "SATAPM";
	
    case PROTO_SEMB:
	return "SEMB";
	
    case PROTO_NVME:
	return "NVME";
	
    case PROTO_MMCSD:
	return "MMCSD";
    }
}
	    

const char *
drive_cam_xport2str(cam_xport xport) {
    switch (xport) {
    case XPORT_UNKNOWN:
	return "?";
	
    case XPORT_UNSPECIFIED:
	return "-";
	
    case XPORT_SPI:
	return "SCSI";
	
    case XPORT_FC:
	return "FC";
	
    case XPORT_SSA:
	return "SSA";
	
    case XPORT_USB:
	return "USB";
	
    case XPORT_PPB:
	return "PPB";
	
    case XPORT_ATA:
	return "PATA";
	
    case XPORT_SAS:
	return "SAS";
	
    case XPORT_SATA:
	return "SATA";
	
    case XPORT_ISCSI:
	return "ISCSI";
	
    case XPORT_SRP:
	return "SRP";
	
    case XPORT_NVME:
	return "NVME";
	
    case XPORT_MMCSD:
	return "MMCSD";
	
    case XPORT_NVMF:
	return "NVMF";
	
    case XPORT_UFSHCI:
	return "UFSHCI";
    }
}

void
drive_init(DRIVE *dp) {
    memset(dp, 0, sizeof(*dp));
    dp->raw.proto = DRIVE_PROTO_UNKNOWN;
    dp->raw.xport = XPORT_UNKNOWN;
    dp->raw.power_on_hours = DRIVE_NO_HOURS;
    dp->raw.endurance_used = DRIVE_NO_ENDURANCE_USED;
    dp->raw.temperature    = DRIVE_NO_TEMPERATURE;
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
drive_proto2str(enum drive_protocol proto) {
    switch (proto) {
    case DRIVE_PROTO_UNKNOWN:
	return "?";
    case DRIVE_PROTO_SCSI:
	return "SCSI";
    case DRIVE_PROTO_ATA:
	return "ATA";
    case DRIVE_PROTO_SATL:
	return "SATL";
    case DRIVE_PROTO_NVME:
	return "NVME";
    case DRIVE_PROTO_ATAPI:
	return "ATAPI";
    case DRIVE_PROTO_MMCSD:
	return "MMCSD";
    }
}

#define DRIVE_IS_ATAPI(d_protocol, d_type, c_transport)         \
    (d_protocol == PROTO_SCSI && d_type == T_CDROM &&		\
     (c_transport == XPORT_SATA || c_transport == XPORT_ATA))

int
drive_classify(DRIVE *dp) {
    struct cam_device *dev = dp->cam;
    union ccb *ccb;
    cam_proto d_protocol;
    uint8_t   d_type;
    cam_proto c_protocol;
    cam_xport c_transport;

    
    if (f_debug)
	fprintf(stderr, "  [classify]\n");

    ccb = cam_getccb(dev);
    ccb->ccb_h.func_code = XPT_GDEV_TYPE;

    if (cam_send_ccb(dev, ccb) < 0) {
	cam_freeccb(ccb);
	return -1;
    }

    if ((ccb->ccb_h.status & CAM_STATUS_MASK) != CAM_REQ_CMP) {
	cam_freeccb(ccb);
	errno = EIO;
	return -1;
    }

    d_protocol = ccb->cgd.protocol;
    d_type = SID_TYPE(&ccb->cgd.inq_data);

    if (f_debug) {
	fprintf(stderr, "\tpd_type              = %u (%s)\n",
		dev->pd_type, drive_cam_devtype2str(dev->pd_type));
	fprintf(stderr, "\tremovable            = %s\n",
		SID_IS_REMOVABLE(&ccb->cgd.inq_data) ? "Yes" : "No");
	fprintf(stderr, "\tdevice protocol      = %u (%s)\n",
		d_protocol, drive_cam_proto2str(d_protocol));
	fprintf(stderr, "\tdevice type          = %d (%s)\n",
		d_type, drive_cam_devtype2str(d_type));
    }
    cam_freeccb(ccb);

    ccb = cam_getccb(dev);
    ccb->ccb_h.func_code = XPT_PATH_INQ;
    if (cam_send_ccb(dev, ccb) < 0) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: XPT_PATH_INQ failed\n",
		    argv0, dev->given_dev_name, dev->given_unit_number);
	cam_freeccb(ccb);
	return -1;
    }

    if ((ccb->ccb_h.status & CAM_STATUS_MASK) != CAM_REQ_CMP) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: XPT_PATH_INQ not CAM_REQ_CMP\n",
		    argv0, dev->given_dev_name, dev->given_unit_number);
	cam_freeccb(ccb);
	errno = EIO;
	return -1;
    }

    c_protocol  = ccb->cpi.protocol;
    c_transport = ccb->cpi.transport;
    if (f_debug) {
	fprintf(stderr, "\tcontroller protocol  = %d (%s)\n",
		c_protocol, drive_cam_proto2str(c_protocol));
	fprintf(stderr, "\tcontroller transport = %d (%s)\n", 
		c_transport, drive_cam_xport2str(c_transport));
    }


    dp->raw.xport = c_transport;
    
    switch (d_protocol) {
    case PROTO_SCSI:
	if (DRIVE_IS_ATAPI(d_protocol, d_type, c_transport))
	    dp->raw.proto = DRIVE_PROTO_ATAPI;
	else if (d_type == T_DIRECT &&
		 (c_transport == XPORT_USB || c_transport == XPORT_SAS)) {
	    if (drive_scsi_is_satl(dp) == 1)
		dp->raw.proto = DRIVE_PROTO_SATL;
	    else
		dp->raw.proto = DRIVE_PROTO_SCSI;
	} else 
	    dp->raw.proto = DRIVE_PROTO_SCSI;
	break;
    case PROTO_ATA:
	dp->raw.proto = DRIVE_PROTO_ATA;
	break;
    case PROTO_ATAPI:
	dp->raw.proto = DRIVE_PROTO_ATAPI;
	break;
    case PROTO_MMCSD:
	dp->raw.proto = DRIVE_PROTO_MMCSD;
	break;
    case PROTO_NVME:
	dp->raw.proto = DRIVE_PROTO_NVME;
	break;
    default:
	dp->raw.proto = DRIVE_PROTO_UNKNOWN;
    }

    if (dp->raw.proto == DRIVE_PROTO_SATL) {
	int rc = satl_identify(dp);
	if (rc < 0)
	    dp->raw.proto = DRIVE_PROTO_SCSI;
    }
	

    cam_freeccb(ccb);
    return 1;
}

