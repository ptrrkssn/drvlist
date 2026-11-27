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
#include <ctype.h>
#include <unistd.h>

#include "utils.h"
#include "scsi.h"
#include "ata.h"

extern int f_debug;
extern int f_temp;
extern int f_hours;
extern int f_smart;
extern int f_tests;
extern int f_wear;

extern char *argv0;

#define SCSI_RETRIES 1
#define SCSI_TIMEOUT 5000

#define SCSI_LOG_SENSE                       0x4d
#define SCSI_READ_CAPACITY                   0x9e

#define SCSI_BLOCKDEV_CHARACTERISTICS_PAGE   0xb1

#define SCSI_SUPPORTED_LOGS_PAGE             0x00
#define SCSI_SUPPORTED_LOGS_SUBPAGE          0x00

#define SCSI_FORMAT_STATUS_LOG_PAGE          0x08
#define SCSI_FORMAT_STATUS_LOG_SUBPAGE       0x00

#define SCSI_TEMP_LOG_PAGE                   0x0d
#define SCSI_TEMP_LOG_SUBPAGE                0x00

#define SCSI_SELFTEST_RESULTS_LOG_PAGE       0x10
#define SCSI_SELFTEST_RESULTS_LOG_SUBPAGE    0x00

#define SCSI_SSD_MEDIA_LOG_PAGE              0x11
#define SCSI_SSD_MEDIA_LOG_SUBPAGE           0x00

#define SCSI_BACKGROUND_SCAN_LOG_PAGE        0x15
#define SCSI_BACKGROUND_SCAN_LOG_SUBPAGE     0x00

#define SCSI_IE_LOG_PAGE                     0x2f
#define SCSI_IE_LOG_SUBPAGE                  0x00

#define SCSI_START_STOP_CYCLE_LOG_PAGE       0x0e
#define SCSI_START_STOP_CYCLE_LOG_SUBPAGE    0x00

#define LOG_SENSE_BUFFER_SIZE                4096

#define SELFTEST_PARAMETER_DATA_LENGTH       16

/*
 * Four-byte log-parameter header plus 16 bytes of self-test data.
 */
#define SELFTEST_PARAMETER_TOTAL_LENGTH       20

			    
#if 0
#define SVPD_SUPPORTED_VPD_PAGES   0x00
#define SVPD_UNIT_SERIAL_NUMBER    0x80
#define SVPD_DEVICE_IDENTIFICATION 0x83
#define	SVPD_MODE_PAGE_POLICY	   0x87
#define SVPD_ATA_INFORMATION       0x89
#define SVPD_BLOCK_LIMITS          0xb0
#define SVPD_LOGICAL_PROVISIONING  0xb2
#endif


struct log_sense_cdb {
    uint8_t opcode;
    uint8_t byte2;
    uint8_t page;
    uint8_t subpage;
    uint8_t reserved[3];
    uint8_t alloc_len[2];
    uint8_t control;
};


struct log_sense_page {
    struct {
	uint8_t page_code;
	uint8_t subpage;
	uint8_t page_length[2];
    } h;
    uint8_t data[4096];
};

struct log_sense_parameter {
    uint8_t code[2];
    uint8_t control;
    uint8_t length;
    uint8_t data[];
};

struct supported_log_pages {
    unsigned int temp : 1;
    unsigned int ie : 1;
    unsigned int selftest : 1;
    unsigned int ssd_media : 1;
    unsigned int format_status : 1;
    unsigned int background_scan : 1;
    unsigned int start_stop_cycle : 1;
    unsigned int blockdev_chars : 1;
};

void
p_buffer(FILE *fp,
	 uint8_t *buffer,
	 size_t bufsize) {
    int i;
    size_t p = 0;

    if (isatty(fileno(fp))) {
	fprintf(fp, "\t\t\033[1;4m%-4s :", "DATA");
	for (i = 0; i < 16; i++) {
	    if (i == 8)
		putc(' ', fp);
	    fprintf(fp, " %02x", i);
	}
	fprintf(fp, " :");
	for (i = 0; i < 16; i++) {
	    if (i == 8)
		putc(' ', fp);
	    fprintf(fp, " %01x", i);
	}
	fprintf(fp, "\n\033[0m");
    }
    
    while (p < bufsize) {
	fprintf(fp, "\t\t%04zx :", p);
	for (i = 0; i < 16 && p+i < bufsize; i++) {
	    if (i == 8)
		putc(' ', fp);
	    fprintf(fp, " %02x", buffer[p+i]);
	}
	for (; i < 16; i++) {
	    if (i == 8)
		putc(' ', fp);
	    fprintf(fp, "   ");
	}
	fprintf(fp, " :");
	for (i = 0; i < 16 && p+i < bufsize; i++) {
	    if (i == 8)
		putc(' ', fp);
	    fprintf(fp, " %c", isprint(buffer[p+i]) ? buffer[p+i] : '_');
	}
	p += i;
	for (; i < 16; i++) {
	    if (i == 8)
		putc(' ', fp);
	    fprintf(fp, "  ");
	}
	putc('\n', fp);
    }
}


static struct log_sense_parameter *
get_next_parameter(uint8_t *buf,
		   size_t buflen,
		   off_t *off) {
    struct log_sense_parameter *pp;

    
    if (*off+sizeof(*pp) > buflen)
	return NULL;

    pp = (struct log_sense_parameter *) (buf+*off);

    *off += sizeof(*pp);
    
    if (*off+pp->length > buflen)
	return NULL;
    
    *off += pp->length;

    return pp;
}


static int
is_invalid(uint8_t *data,
	   uint8_t length) {
    int i;
    
    for (i = 0; i < length && data[i] == 0xff; i++)
	;

    return i == length;
}



ssize_t
drive_scsi_get_inquiry(DRIVE *dp,
		       uint16_t page,
		       void *buf,
		       size_t bufsize) {
    union ccb *ccb = NULL;
    uint32_t cam_status;
    size_t cam_resid;
    uint8_t page_code = page & 0xff;
    uint8_t evpd = (page >> 8) & 0xff;
    

    memset(buf, 0, bufsize);

    ccb = cam_getccb(dp->cam);
    if (ccb == NULL)
	return -1;
    
    if (f_debug)
	fprintf(stderr, "    [scsi_inquiry 0x%02x, evpd:0x%02x]\n", page_code, evpd);
    
    scsi_inquiry(&ccb->csio,
		 SCSI_RETRIES,
		 NULL,
		 CAM_TAG_ACTION_NONE,
		 (uint8_t *) buf,
		 bufsize,
		 evpd,
		 page_code,
		 SSD_FULL_SIZE,
		 SCSI_TIMEOUT);
    
    /* Disable freezing the device queue */
    ccb->ccb_h.flags |= CAM_DEV_QFRZDIS;
    ccb->ccb_h.flags |= CAM_PASS_ERR_RECOVER;
    
    if (cam_send_ccb(dp->cam, ccb) < 0) {
	if (f_debug)
	    fprintf(stderr,
		    "%s: Error: %s%u: INQUIRY 0x%02x Failed: cam_send_ccb: %s\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number, page_code,
		    strerror(errno));
	cam_freeccb(ccb);
	return -1;
    }
    
    cam_status = (ccb->ccb_h.status & CAM_STATUS_MASK);
    cam_resid = ccb->csio.resid;
    
    cam_freeccb(ccb);
    
    if (cam_status != CAM_REQ_CMP) {
	if (f_debug)
	    fprintf(stderr,
		    "%s: Error: %s%u: INQUIRY 0x%02x Failed: cam_send_ccb: status != CAM_REQ_CMP\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number, page_code);
	errno = EBUSY;
	return -1;
    }
    
    if (cam_resid > bufsize) {
	if (f_debug)
	    fprintf(stderr,
		    "%s: Error: %s%u: INQUIRY 0x%02x Failed: cam_send_ccb: resid > bufsize\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number, page_code);
	errno = EIO;


	return -1;
    }
	
    return bufsize - cam_resid;
}


const char *
scsi_inquiry_version2str(uint8_t version) {
    static char buf[64];
    
    switch (version) {
    case 0x00:
	return "-";
    case 0x01:
	return "CCS";
    case 0x02:
	return "SCSI-2";
    case 0x03:
	return "SPC-1";
    case 0x04:
	return "SPC-2";
    case 0x05:
	return "SPC-3";
    case 0x06:
	return "SPC-4";
    case 0x07:
	return "SPC-5";
    case 0x08:
	return "SPC-6";
    default:
	sprintf(buf, "SPC-%u?", version-2);
	return buf;
    }
}


static const char *
sas_version(uint16_t vd) {
    if (vd >= 0x0be0 && vd < 0x0c00)
        return "SAS";
    if (vd >= 0x0c00 && vd < 0x0c20)
        return "SAS-1.1";
    if (vd >= 0x0c20 && vd < 0x0c40)
        return "SAS-2";
    if (vd >= 0x0c40 && vd < 0x0c60)
        return "SAS-2.1";
    if (vd >= 0x0c60 && vd < 0x0c80)
        return "SAS-3";
    if (vd >= 0x0c80 && vd < 0x0ca0)
        return "SAS-4";

    if (vd >= 0x0ea0 && vd < 0x0ec0)
        return "SPL";
    if (vd >= 0x0ec0 && vd < 0x0ee0)
        return "SPL-2";
    if (vd >= 0x0ee0 && vd < 0x0f00)
        return "SPL-3";
    if (vd >= 0x0f00 && vd < 0x0f20)
        return "SPL-4";

    return "-";
}


int
drive_scsi_parse_inquiry(DRIVE *dp,
			 uint8_t *buffer,
			 size_t bufsize) {
    struct scsi_inquiry_data *inq = (struct scsi_inquiry_data *) buffer;

    
    
    if (f_debug) {
	fprintf(stderr, "\tSCSI INQUIRY: removable   = %s\n",  SID_IS_REMOVABLE(inq) ? "Yes" : "No");
	fprintf(stderr, "\tSCSI INQUIRY: ansi_rev    = %d (%s)\n",  SID_ANSI_REV(inq),
		scsi_inquiry_version2str(SID_ANSI_REV(inq)));

	if (bufsize >= 8)
	    fprintf(stderr, "\tSCSI INQUIRY: vendor      = %.*s\n", (int) sizeof(inq->vendor), inq->vendor);
	if (bufsize >= 16)
	    fprintf(stderr, "\tSCSI INQUIRY: product     = %.*s\n", (int) sizeof(inq->product), inq->product);
	if (bufsize >= 32)
	    fprintf(stderr, "\tSCSI INQUIRY: revision    = %.*s\n", (int) sizeof(inq->revision), inq->revision);
	if (bufsize >= 36)
	    fprintf(stderr, "\tSCSI INQUIRY: vendor      = %.*s\n", (int) (bufsize-36 < 20 ? bufsize-36 : 20), buffer+36);
    }
    
    if (bufsize >= 73) {
	int i;
	uint8_t *bp = &inq->version1[0];
	
	for (i=1; i<8; i++) {
	    uint16_t v = get_be16(bp+i*2);
	    
	    
	    if (v >= 0x0b00 && v <= 0x0cff) {
		dp->xport = strdup(sas_version(v));
		if (f_debug)
		    fprintf(stderr, "\tSCSI INQUIRY: vdesc %d (SAS Version) = 0x%04x (%s)\n", i, v, sas_version(v));
	    } else if (v >= 0x0e00 && v <= 0x0fff) {
		dp->xport = strdup(sas_version(v));
		if (f_debug)
		    fprintf(stderr, "\tSCSI INQUIRY: vdesc %d (SPL Version) = 0x%04x (%s)\n", i, v, sas_version(v));
	    } else {
		if (f_debug)
		    fprintf(stderr, "\tSCSI INQUIRY: vdesc %d               = 0x%04x (%s)\n", i, v, sas_version(v));
	    }
	}
    }
    
    if (f_debug > 1)
	p_buffer(stderr, buffer, bufsize);


    
    return 0;
}

int
drive_scsi_inquiry(DRIVE *dp) {
    uint8_t buffer[252];
    ssize_t res;
    

    res = drive_scsi_get_inquiry(dp, 0, buffer, sizeof(buffer));
    if (res < 0)
	return res;

    drive_scsi_parse_inquiry(dp, buffer, res);
    return 0;
}


int
drive_scsi_get_vpd_page(DRIVE *dp,
			uint8_t page_id,
			uint8_t *buffer,
			size_t bufsize) {
    ssize_t res;
    

    return drive_scsi_get_inquiry(dp,
			     page_id | INQUIRY_EVPD,
			     buffer, bufsize);
    if (res < 0)
	return res;

    return 0;
}

int
drive_scsi_get_vpd_supported_pages(DRIVE *dp,
				   struct scsi_vpd_supported_page_list *pages) {
    ssize_t res;


    res = drive_scsi_get_inquiry(dp,
			     SVPD_SUPPORTED_PAGE_LIST | INQUIRY_EVPD,
			     (uint8_t *) pages,
			     sizeof(*pages));
    if (res < 0)
	return res;

    if (res < 4) {
	fprintf(stderr, "%s: Error: %s%u: INQUIRY VPD 0x%02x Invalid response [res=%zd length=%u]\n",
		argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		SVPD_SUPPORTED_PAGE_LIST,
		res, pages->length);
	errno = EIO;
	return -1;
    }

    if (pages->length+4 > res) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: INQUIRY VPD 0x%02x: Invalid length, adjusting %d -> %zu\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		    SVPD_SUPPORTED_PAGE_LIST,
		    pages->length, res-4);
	
	pages->length = res-4;
    }
    
    return res;
}

int
drive_scsi_has_vpd_page(DRIVE *dp,
			uint8_t page_id) {
    struct scsi_vpd_supported_page_list pages;
    ssize_t res;
    int i;
    int rc = 0;


    res = drive_scsi_get_vpd_supported_pages(dp, &pages);
    if (res < 0)
	return -1;

    if (f_debug)
	fprintf(stderr, "\tINQUIRY VPD PAGES: %d Supported : ",
		pages.length);
    
    for (i = 0; i < pages.length; i++) {
	if (pages.list[i] == page_id)
	    rc = 1;
	if (f_debug)
	    fprintf(stderr, " 0x%02x", pages.list[i]);
    }
    if (f_debug)
	putc('\n', stderr);
    
    return rc;    
}

struct scsi_vpd_sat_ata_information {
    uint8_t device;
    uint8_t page_code; /* 0x89 */
    uint8_t length[2];
    uint8_t reserved1[4];
    uint8_t sat_vendor[8];
    uint8_t sat_product[16];
    uint8_t sat_revision[4];
    uint8_t ata_device_signature[20];
    uint8_t ata_command_code;
    uint8_t reserved2[3];
    uint8_t ata_identify_data[512];
};

int
drive_scsi_parse_vpd_sat_ata_information(DRIVE *dp,
					 struct scsi_vpd_sat_ata_information *satinfo) {
    fprintf(stderr, "\tVPD SAT ATA INFO: sat vendor    = %.8s\n",  satinfo->sat_vendor);
    fprintf(stderr, "\tVPD SAT ATA INFO: sat product   = %.16s\n", satinfo->sat_product);
    fprintf(stderr, "\tVPD SAT ATA INFO: sat revision  = %.4s\n",  satinfo->sat_revision);
    fprintf(stderr, "\tVPD SAT ATA INFO: ata interface = %s\n",    satinfo->ata_device_signature[0] == 0x34 ? "SATA" : "PATA");
    fprintf(stderr, "\tVPD SAT ATA INFO: ata protocol  = %s\n",    satinfo->ata_command_code == 0xec ? "ATA" : "ATAPI");

    drive_ata_parse_identify(dp, (struct ata_params *) &satinfo->ata_identify_data);
    return 0;
}
					 



int
drive_scsi_dump_vpd_pages(DRIVE *dp) {
    struct scsi_vpd_supported_page_list pages;
    int i;
    ssize_t res;
    int rc = 0;
    

    fprintf(stderr, "  [scsi_dump_inquiry_vpd_pages]\n");
    
    res = drive_scsi_get_inquiry(dp,
			     SVPD_SUPPORTED_PAGE_LIST | INQUIRY_EVPD,
			     (uint8_t *) &pages,
			     sizeof(pages));
    if (res < 0)
	return res;

    if (res < 4) {
	fprintf(stderr, "%s: Error: %s%u: INQUIRY 0x%02x Invalid response [res=%zd length=%u]\n",
		argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		SVPD_SUPPORTED_PAGE_LIST,
		res, pages.length);
	errno = EIO;
	return -1;
    }

    if (pages.length+4 < res)
	res = pages.length;
    else
	res -= 4;
    
    fprintf(stderr, "\tINQUIRY VPD PAGES: %ld (%d) Supported : ",
	    res, pages.length);
    
    for (i = 0; i < res; i++) {
	fprintf(stderr, " 0x%02x", pages.list[i]);
    }
    putc('\n', stderr);

    for (i = 0; i < res; i++) {
	uint8_t buffer[568];
	ssize_t res;
	
	res = drive_scsi_get_vpd_page(dp, pages.list[i], buffer, pages.list[i] == 0x89 ? sizeof(buffer) : 252);
	if (res > 0) {
	    if (pages.list[i] == 0x89)
		drive_scsi_parse_vpd_sat_ata_information(dp, (struct scsi_vpd_sat_ata_information *) &buffer);
	    p_buffer(stderr, buffer, res);
	}
    }
    
    return rc;
}


/*
 * Request a LOG SENSE page
 */
ssize_t
drive_scsi_log_sense(DRIVE *dp,
		     uint8_t page,
		     uint8_t subpage,
		     uint8_t *buf,
		     size_t bufsize) {
    union ccb *ccb;
    struct log_sense_cdb *cdb;
    ssize_t len;
    size_t got;
    size_t cam_resid;
    uint32_t cam_status;
    int saved_errno;
    

    if (f_debug)
	fprintf(stderr, "    [scsi_log_sense 0x%02x/0x%02x]\n", page, subpage);

    memset(buf, 0, bufsize);

    ccb = cam_getccb(dp->cam);
    if (!ccb)
	return -1;

    cam_fill_csio(&ccb->csio,
		  SCSI_RETRIES,
		  NULL,
		  CAM_DIR_IN,
		  CAM_TAG_ACTION_NONE,
		  buf,
		  bufsize,
		  SSD_FULL_SIZE,
		  sizeof(*cdb),
		  SCSI_TIMEOUT);
    
    cdb = (struct log_sense_cdb *)&ccb->csio.cdb_io.cdb_bytes;
    memset(cdb, 0, sizeof(*cdb));

    cdb->page = (1 << 6) | page;
    cdb->subpage = subpage;
    cdb->opcode = SCSI_LOG_SENSE;
    put_be16(cdb->alloc_len, bufsize);
    
    ccb->ccb_h.flags |= CAM_DEV_QFRZDIS;
    
    if (cam_send_ccb(dp->cam, ccb) < 0) {
	saved_errno = errno;
	if (f_debug)
	    fprintf(stderr,
		    "%s: Error: %s%u: LOG SENSE 0x%02x/0x%02x Failed: cam_send_ccb: %s\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		    page, subpage, 
		    strerror(errno));
	
        cam_freeccb(ccb);
	errno = saved_errno;
        return -1;
    }

    cam_status = (ccb->ccb_h.status & CAM_STATUS_MASK);
    cam_resid = ccb->csio.resid;
    
    cam_freeccb(ccb);
    
    if (cam_status != CAM_REQ_CMP) {
	if (f_debug) {
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE 0x%02x/0x%02x Failed: CAM status 0x%x\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		    page, subpage, 
		    cam_status);
	    scsi_sense_print(dp->cam, &ccb->csio, stderr);
	}
	errno = EBUSY;
        return -1;
    }

    if (cam_resid > bufsize) {
	errno = EIO;
	return -1;
    }
	
    got = bufsize - cam_resid;

    if (got < 4) {
	fprintf(stderr, "%s: Error: %s%u: LOG SENSE 0x%02x/0x%02x Failed: Truncated LOG SENSE page\n",
		argv0, dp->cam->given_dev_name, dp->cam->given_unit_number, 
		page, subpage);
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

    if ((buf[0] & 0x3f) != page) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE failed: 0x%02x: Invalid Page Code\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		    (buf[0] & 0x3f));

	errno = EINVAL;
	return -1;
    }
    
    len = get_be16(&buf[2]) + 4;

    /* Some sanity checks */
    if (len > bufsize) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE buffer too small: len=%zd > bufsize = %zd\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		    len, bufsize);
	errno = EINVAL;
	return -1;
    }
    
    if (got != len) {
	if (f_debug)
	    fprintf(stderr, "%s: Error: %s%u: LOG SENSE buffer length inconsistent: len=%zd vs got=%zd\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		    len, got);
	errno = EINVAL;
	return -1;
    }

    if (f_debug > 1)
	p_buffer(stderr, buf, len);
    
    return len;
}
	       

int
drive_scsi_log_sense_foreach(DRIVE *dp,
			     uint8_t page,
			     uint8_t subpage,
			     int (*param_handler)(uint16_t code, uint8_t control, uint8_t length, uint8_t *data, void *xp),
			     void *xp) {
    uint8_t buf[LOG_SENSE_BUFFER_SIZE];
    ssize_t len;
    off_t off;
    struct log_sense_parameter *pp;
    int rc = 0;
    
    
    len = drive_scsi_log_sense(dp, page, subpage, buf, sizeof(buf));
    if (len < 0)
	return -1;
    
    /* Parameters begin at byte 4 */
    off = 4;
    while ((pp = get_next_parameter(buf, len, &off)) != NULL) {
	uint16_t code = get_be16(pp->code);
	
	rc = (*param_handler)(code, pp->control, pp->length, pp->data, xp);
	if (rc)
	    break;
    }

    return rc;
}



int
drive_scsi_supported_log_pages(DRIVE *dp,
			       struct supported_log_pages *pages) {
    uint8_t buf[4096];
    ssize_t len;
    off_t off;


    if (f_debug)
	fprintf(stderr, "  [scsi_supported_log_pages]\n");
    
    len = drive_scsi_log_sense(dp,
			       SCSI_SUPPORTED_LOGS_PAGE,
			       SCSI_SUPPORTED_LOGS_SUBPAGE,
			       buf, sizeof(buf));
    if (len < 0)
	return -1;


    if (f_debug) 
	fprintf(stderr, "\t%zu Pages:",
		len-4);
    
    for (off = 4; off < len; off++) {
	uint8_t page = buf[off];
	
	if (f_debug)
	    fprintf(stderr, " 0x%02x", page);
	if (pages) {
	    switch (page) {
	    case SCSI_TEMP_LOG_PAGE:
		pages->temp = 1;
		break;
	    case SCSI_IE_LOG_PAGE:
		pages->ie = 1;
		break;
	    case SCSI_SELFTEST_RESULTS_LOG_PAGE:
		pages->selftest = 1;
		break;
	    case SCSI_SSD_MEDIA_LOG_PAGE:
		pages->ssd_media = 1;
		break;
	    case SCSI_FORMAT_STATUS_LOG_PAGE:
		pages->format_status = 1;
		break;
	    case SCSI_BACKGROUND_SCAN_LOG_PAGE:
		pages->background_scan = 1;
		break;
	    case SCSI_START_STOP_CYCLE_LOG_PAGE:
		pages->start_stop_cycle = 1;
		break;
	    }
	}
    }

    if (f_debug)
	putc('\n', stderr);

    return 0;
}


			 

static int
temperature_handler(uint16_t code,
		    uint8_t control,
		    uint8_t length,
		    uint8_t *data,
		    void *xp) {
    DRIVE *dp = (DRIVE *) xp;
    int rc = 0;


    switch (code) {
    case 0x0000: /* Temperature */
	if (length >= 2) {
	    /*
	     * Temperature parameter.
	     *
	     * First data byte is reserved.
	     * Second is temperature in degrees Celsius.
	     *
	     * 0xff means temperature unavailable.
	     */
	    uint8_t temp = data[1];
	    
	    if (temp != 0xff) {
		if (f_debug)
		    fprintf(stderr, "\tTEMPERATURE LOG: temp=%u C\n",
			    temp);
		dp->raw.temperature = temp;
		rc = 1;
	    }
	}
	break;
	
    case 0x0001: /* Reference Temperature */
	if (length >= 2) {
	    uint8_t temp = data[1];

	    if (f_debug)
		fprintf(stderr, "\tTEMPERATURE LOG: reftemp=%u\n",
			temp);
	}
	break;
	    
    default:
	if (f_debug)
	    fprintf(stderr, "\tTEMPERATURE LOG: code 0x%04x\n",
		    code);
	break;
    } 
    
    return rc;
}


int
drive_scsi_temperature(DRIVE *dp) {
    return drive_scsi_log_sense_foreach(dp,
					SCSI_TEMP_LOG_PAGE, SCSI_TEMP_LOG_SUBPAGE,
					temperature_handler,
					(void *) dp);
}



static int
start_stop_cycle_handler(uint16_t code,
			 uint8_t control,
			 uint8_t length,
			 uint8_t *data,
			 void *xp) {
    DRIVE *dp = (DRIVE *) xp;
    int rc = 0;
    uint64_t val;
    double used = 0.0;
    

    switch (code) {
    case 0x0001: /* Manufacture Date */
	dp->made = strdupget((char *) data, length);
	if (f_debug)
	    fprintf(stderr, "\tSTART-STOP CYCLE LOG: manufacture date = %.*s\n",
		    length, (char *) data);
	break;
	
    case 0x0002: /* Accounting Date */
	if (f_debug)
	    fprintf(stderr, "\tSTART-STOP CYCLE LOG: accounting date  = %.*s\n",
		    length, (char *) data);
	break;
	
    case 0x0003: /* Specified Start-Stop Cycle Count */
	val = get_be(data, length);
	dp->raw.cycles.start_stop.specified = val;
	if (f_debug)
	    fprintf(stderr, "\tSTART-STOP CYCLE LOG: specified start-stop cycle count  = %zu\n", val);
	break;

    case 0x0004: /* Accumulated Start-Stop Cycle Count */
	val = get_be(data, length);
	dp->raw.cycles.start_stop.accumulated = val;
	if (dp->raw.cycles.start_stop.specified != 0) {
	    used = (double) val*100.0 / (double) dp->raw.cycles.start_stop.specified;
	    if (dp->raw.endurance_used == DRIVE_NO_ENDURANCE_USED || used > dp->raw.endurance_used)
		dp->raw.endurance_used = used;
	}
	if (f_debug) {
	    fprintf(stderr, "\tSTART-STOP CYCLE LOG: accumulated start-stop cycle count  = %zu\n", val);
	    if (used > 0)
		fprintf(stderr, "\tSTART-STOP CYCLE LOG: accumulated start-stop cycle life  = %.1f %%\n",
			used);
	}
	break;

    case 0x0005: /* Specified Load-Unload Cycle Count */
	val = get_be(data, length);
	dp->raw.cycles.load_unload.specified = val;
	if (f_debug)
	    fprintf(stderr, "\tSTART-STOP CYCLE LOG: specified load-unload cycle count  = %zu\n", val);
	break;

    case 0x0006: /* Accumulated Load-Unload Cycle Count */
	val = get_be(data, length);
	dp->raw.cycles.load_unload.accumulated = val;
	if (dp->raw.cycles.load_unload.specified != 0) {
	    used = (double) val*100.0 / (double) dp->raw.cycles.load_unload.specified;
	    if (dp->raw.endurance_used == DRIVE_NO_ENDURANCE_USED || used > dp->raw.endurance_used)
		dp->raw.endurance_used = used;
	}
	if (f_debug) {
	    fprintf(stderr, "\tSTART-STOP CYCLE LOG: accumulated load-unload cycle count  = %zu\n", val);
	    if (dp->raw.cycles.load_unload.specified != 0)
		fprintf(stderr, "\tSTART-STOP CYCLE LOG: accumulated load-unload cycle life  = %.1f %%\n",
			used);
	}
	break;

    default:
	if (f_debug)
	    fprintf(stderr, "\tSTART-STOP CYCLE LOG: code 0x%04x\n",
		    code);
	break;
    } 
    
    return rc;
}

int
drive_scsi_start_stop_cycle(DRIVE *dp) {
    return drive_scsi_log_sense_foreach(dp,
					SCSI_START_STOP_CYCLE_LOG_PAGE,
					SCSI_START_STOP_CYCLE_LOG_SUBPAGE,
					start_stop_cycle_handler,
					(void *) dp);
}


static int
ssd_media_handler(uint16_t code,
		  uint8_t control,
		  uint8_t length,
		  uint8_t *data,
		  void *xp) {
    DRIVE *dp = (DRIVE *) xp;
    int rc = 0;

    switch (code) {
    case 0x0001: /* Percentage Used Endurance Indicator */
	if (length >= 4) {
	    /*
	     * Byte 0-2 is reserved.
	     * Byte 3 is Percentage Used Endurance
	     */
	    uint8_t used = data[3];

	    if (f_debug)
		fprintf(stderr, "\tSSD MEDIA LOG: used=%u %%\n",
			used);

	    dp->raw.endurance_used = used;
	    rc = 0;
	}
	break;
	
    default:
	if (f_debug) {
	    int i;
	    fprintf(stderr, "\tSSD MEDIA LOG: code 0x%04x :",
		    code);
	    for (i = 0; i < length; i++)
		fprintf(stderr, " 0x%02x", data[i]);
	    putc('\n', stderr);
	}
	break;
    }

    return rc;
}

int
drive_scsi_endurance_used(DRIVE *dp) {
    return drive_scsi_log_sense_foreach(dp,
					SCSI_SSD_MEDIA_LOG_PAGE,
					SCSI_SSD_MEDIA_LOG_SUBPAGE,
					ssd_media_handler,
					(void *) dp);
}



static int
format_status_handler(uint16_t code,
		      uint8_t control,
		      uint8_t length,
		      uint8_t *data,
		      void *xp) {
    DRIVE *dp = (DRIVE *) xp;
    uint64_t val, poh;
    int rc = 0;

    
    if (is_invalid(data, length))
	return 0;
    
    switch (code) {
    case 0x0004: /* PowerOn Minutes since Format */
	val = get_be(data, length);
	
	if (f_debug)
	    fprintf(stderr, "\tFORMAT STATUS LOG: poweron_minutes=%zu (%zu hours)\n",
		    val, val/60);

	poh = val/60;
	if (dp->raw.power_on_hours == DRIVE_NO_HOURS || poh > dp->raw.power_on_hours)
	    dp->raw.power_on_hours = poh;
	rc = 1;
	break;
	
    case 0x0000: /* Format Data Out */
	if (f_debug) {
	    int i;
	    
	    fprintf(stderr, "\tFORMAT STATUS LOG: format_data_out:");
	    for (i = 0; i < length; i++)
		fprintf(stderr, " 0x%02x", data[i]);
	    putc('\n', stderr);
	}
	break;
    case 0x0001: /* Grown Defects During Certification */
	val = get_be(data, length);
	if (f_debug)
	    fprintf(stderr, "\tFORMAT STATUS LOG: grown_defects=%zu\n", val);
	break;
    case 0x0002: /* Total Blocks Reassigned During Format */
	val = get_be(data, length);
	if (f_debug)
	    fprintf(stderr, "\tFORMAT STATUS LOG: total_blocks_reassigned_during_format=%zu\n", val);
	break;
    case 0x0003: /* Total New Blocks Reassigned */
	val = get_be(data, length);
	if (f_debug)
	    fprintf(stderr, "\tFORMAT STATUS LOG: total_new_blocks_reassigned=%zu\n", val);
	break;
	
    default:
	if (f_debug)
	    fprintf(stderr, "\tFORMAT STATUS LOG: code 0x%04x\n",
		    code);
	break;
    }

    return rc;
}


int
drive_scsi_format_status_poweron(DRIVE *dp) {
    return drive_scsi_log_sense_foreach(dp,
					SCSI_FORMAT_STATUS_LOG_PAGE,
					SCSI_FORMAT_STATUS_LOG_SUBPAGE,
					format_status_handler,
					(void *) dp);
}


static int
ie_status_handler(uint16_t code,
		  uint8_t control,
		  uint8_t length,
		  uint8_t *data,
		  void *xp) {
    DRIVE *dp = (DRIVE *) xp;
    int rc = 0;
    uint8_t asc;
    uint8_t ascq;
    
    
    if (is_invalid(data, length))
	return 0;
    
    switch (code) {
    case 0x0000: /* General Parameter Data */
        if (length < 2)
	    return 0;
	
	asc    = data[0];
	ascq   = data[1];
	
	uint8_t temp_c = 0xff;
	uint8_t trip_c = 0xff;
	uint8_t max_c  = 0xff;
	
	if (length >= 3) {
	    temp_c = data[2];
	    if (f_debug)
		fprintf(stderr, "\tINFORMATIONAL EXCEPTIONS LOG: temp=%u C\n",
			temp_c);
	    
	    if (length >= 4) {
		trip_c = data[3];
		if (f_debug)
		    fprintf(stderr, "\tINFORMATIONAL EXCEPTIONS LOG: trip=%u C\n",
			    trip_c);
		if (length >= 5) {
		    max_c = data[4];
		    if (f_debug)
			fprintf(stderr, "\tINFORMATIONAL EXCEPTIONS LOG: max=%u C\n",
				max_c);
		}
	    }
	}
	
	if (temp_c != 0xff)
	    dp->raw.temperature= temp_c;
	
	/*
	 * No informational exception is currently reported.
	 */
	if (asc == 0 && ascq == 0) {
	    dp->raw.smart_status = DRIVE_SMART_PASSED;
	    rc = 1;
	    break;
	}
	
	/*
	 * Common predictive-failure ASC:
	 *
	 *     ASC 0x5d: Failure prediction threshold exceeded
	 *
	 * ASCQ provides the component-specific detail.
	 */
	if (asc == 0x5d) {
	    dp->raw.smart_status = DRIVE_SMART_FAILED;
	    rc = 1;
	    break;
	}
	
	/*
	 * The device reports another informational exception.
	 * Preserve ASC/ASCQ and report it as a warning.
	 */
	dp->raw.smart_status = DRIVE_SMART_WARNING;
	rc = 1;
	break;
    }

    if (f_debug && rc == 1)
	fprintf(stderr, "\tINFORMATIONAL EXCEPTIONS LOG: ASC=0x%02x ASCQ=0x%02x -> SMART=%s\n",
		asc, ascq, drive_smart_status2str(dp->raw.smart_status));
    
    return rc;
}

int
drive_scsi_ie_status(DRIVE *dp) {
    return drive_scsi_log_sense_foreach(dp,
					SCSI_IE_LOG_PAGE,
					SCSI_IE_LOG_SUBPAGE,
					ie_status_handler,
					(void *) dp);
}



static enum drive_selftest_status
classify_selftest_result(uint8_t result) {
    switch (result) {
    case 0x0:
        return (DRIVE_SELFTEST_PASSED);

    case 0x1:
    case 0x2:
        return (DRIVE_SELFTEST_ABORTED);

    case 0x3:
    case 0x4:
    case 0x5:
    case 0x6:
    case 0x7:
        return (DRIVE_SELFTEST_FAILED);

    case 0xf:
        return (DRIVE_SELFTEST_RUNNING);

    default:
        return (DRIVE_SELFTEST_UNKNOWN);
    }
}

static int
selftest_results_handler(uint16_t code,
			 uint8_t control,
			 uint8_t length,
			 uint8_t *data,
			 void *xp) {
    DRIVE *dp = (DRIVE *) xp;
    int rc = 0;
    uint8_t selftest_code;
    uint8_t selftest_result;
    uint16_t poh;
    static enum drive_selftest_status status;
    
    
    /* Self-test result descriptors contain 16 data bytes */
    if (length < 16)
	return 0;
	
    /*
     * Descriptor byte zero:
     *
     * bits 7-5  self-test code
     * bit  4    reserved
     * bits 3-0  self-test result
     */
    selftest_code = (data[0] >> 5) & 0x07;
    selftest_result = data[0] & 0x0f;
    
    /*
     * Bytes 2-3: accumulated power-on hours.
     */
    poh = get_be16(&data[2]);
    if (f_debug)
	fprintf(stderr, "\tSELFTEST RESULTS LOG: parameter=%u code=%u result=0x%x power_on_hours=%u\n",
		code,
		selftest_code,
		selftest_result,
		poh);
    if (poh == 0xffff)
	return 0;

    status = classify_selftest_result(selftest_result);
    /* Parameter code 1 is normally the newest entry */
    if (code == 0x0001) {
	dp->raw.selftest.status = status;
    }

    if (poh > 0) {
	if (dp->raw.last_self_test == DRIVE_NO_HOURS || poh > dp->raw.last_self_test)
	    dp->raw.last_self_test = poh;

	if (dp->raw.power_on_hours == DRIVE_NO_HOURS || poh > dp->raw.power_on_hours)
	    dp->raw.power_on_hours = poh;
    }
    
    dp->raw.selftest.tests++;
    switch (status) {
    case DRIVE_SELFTEST_PASSED:
	dp->raw.selftest.passed++;
	break;
    case DRIVE_SELFTEST_FAILED:
	dp->raw.selftest.failed++;
	break;
    case DRIVE_SELFTEST_ABORTED:
	dp->raw.selftest.aborted++;
	break;
    case DRIVE_SELFTEST_RUNNING:
	dp->raw.selftest.running++;
	break;
    case DRIVE_SELFTEST_UNKNOWN:
    case DRIVE_SELFTEST_NONE:
	break;
    }
    
    return rc;
}


int
drive_scsi_selftest_results_status(DRIVE *dp) {
   return drive_scsi_log_sense_foreach(dp,
				       SCSI_SELFTEST_RESULTS_LOG_PAGE,
				       SCSI_SELFTEST_RESULTS_LOG_SUBPAGE,
				       selftest_results_handler,
				       (void *) dp);
}


static int
background_scan_handler(uint16_t code,
			uint8_t control,
			uint8_t length,
			uint8_t *data,
			void *xp) {
    DRIVE *dp = (DRIVE *) xp;
    uint32_t pom, poh;
    int rc = 0;
    

    if (f_debug)
	fprintf(stderr, "\tBACKGROUND SCAN: code = 0x%04x\n", code);

    switch (code) {
    case 0x0000:
	pom = get_be32(data);
	if (pom != 0xffffffff) {
	    poh = pom/60;
	    
	    if (dp->raw.power_on_hours == DRIVE_NO_HOURS || poh > dp->raw.power_on_hours) {
		dp->raw.power_on_hours = poh;
		rc = 1;
	    }
	    
	    if (f_debug) {
		fprintf(stderr, "\tBACKGROUND SCAN: power_on_minutes          = %u (0x%08x, %u h)\n", pom, pom, poh);
		fprintf(stderr, "\tBACKGROUND SCAN: scan_status               = 0x%02x\n", data[5]);
		fprintf(stderr, "\tBACKGROUND SCAN: scans_performed           = %u\n", get_be16(data+6));
		fprintf(stderr, "\tBACKGROUND SCAN: medium_scan_progress      = %u\n", get_be16(data+8));
		fprintf(stderr, "\tBACKGROUND SCAN: number_of_bg_medium_scans = %u\n", get_be16(data+10));
	    }
	}
	break;
    }

    return rc;
}


int
drive_scsi_background_scan(DRIVE *dp) {
   return drive_scsi_log_sense_foreach(dp,
				       SCSI_BACKGROUND_SCAN_LOG_PAGE,
				       SCSI_BACKGROUND_SCAN_LOG_SUBPAGE,
				       background_scan_handler,
				       (void *) dp);
}




/*
 * Check if a SCSI drive really is a SATA drive behind a SATL bridge
 */
int
drive_scsi_is_satl(DRIVE *dp) {
    return drive_scsi_has_vpd_page(dp, SVPD_ATA_INFORMATION);
}


ssize_t
drive_scsi_get_read_capacity(DRIVE *dp,
			     uint8_t *buffer,
			     size_t bufsize) {
    union ccb *ccb = NULL;
    uint32_t cam_status;
    size_t cam_resid;
    
    

    memset(buffer, 0, bufsize);

    ccb = cam_getccb(dp->cam);
    if (ccb == NULL)
	return -1;
    
    scsi_read_capacity_16(&ccb->csio,
			  SCSI_RETRIES,
			  NULL,
			  CAM_TAG_ACTION_NONE,
			  0, 0, 0,
			  (uint8_t *) buffer,
			  bufsize,
			  SSD_FULL_SIZE,
			  SCSI_TIMEOUT);
    
    /* Disable freezing the device queue */
    ccb->ccb_h.flags |= CAM_DEV_QFRZDIS;
    ccb->ccb_h.flags |= CAM_PASS_ERR_RECOVER;
    
    if (cam_send_ccb(dp->cam, ccb) < 0) {
	if (f_debug)
	    fprintf(stderr,
		    "%s: Error: %s%u: READ_CAPACITY(16) Failed: cam_send_ccb: %s\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number,
		    strerror(errno));
	cam_freeccb(ccb);
	return -1;
    }
    
    cam_status = (ccb->ccb_h.status & CAM_STATUS_MASK);
    cam_resid = ccb->csio.resid;
    
    cam_freeccb(ccb);
    
    if (cam_status != CAM_REQ_CMP) {
	if (f_debug)
	    fprintf(stderr,
		    "%s: Error: %s%u: SCSI READ CAPACITY(16) Failed: cam_send_ccb: status != CAM_REQ_CMP\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number);
	errno = EBUSY;
	return -1;
    }
    
    if (cam_resid > bufsize) {
	if (f_debug)
	    fprintf(stderr,
		    "%s: Error: %s%u: SCSI READ CAPACITY(16) Failed: cam_send_ccb: resid > bufsize\n",
		    argv0, dp->cam->given_dev_name, dp->cam->given_unit_number);
	errno = EIO;


	return -1;
    }

    return bufsize - cam_resid;
}


int
drive_scsi_parse_read_capacity(DRIVE *dp,
			       uint8_t *buffer,
			       size_t bufsize) {
    char valbuf[64], tmpbuf[64];

    
    if (bufsize != 32)
	return -1;
    
    uint64_t lba = get_be(buffer+0, 8);
    uint32_t lbl = get_be(buffer+8, 4);
    uint8_t rc_basis = (buffer[12] >> 4) & 0x03;
    uint8_t p_type   = (buffer[12] >> 1) & 0x07;
    uint8_t prot_en  = (buffer[12] & 0x01);
    uint8_t pie   = (buffer[13] >> 4) & 0x0f;
    uint8_t lbpbe = (buffer[13] & 0x0f);
    uint8_t lbpme = (buffer[14] >> 7) & 0x01;
    uint8_t lbprz = (buffer[14] >> 6) & 0x01;
    uint8_t lalba = (((uint32_t) buffer[14] & 0x3f) << 8) | (uint32_t) buffer[15];
    uint32_t pbl;
    uint64_t size;
    
    
    pbl = lbl << lbpbe;
    size = lba*lbl;

    if (lbl == pbl)
	sprintf(valbuf, "%sn", bsize2str(lbl, NULL, 0));
    else if (pbl == 4096)
	sprintf(valbuf, "%se", bsize2str(lbl, NULL, 0));
    else
	sprintf(valbuf, "%s/%s",
		bsize2str(lbl, NULL, 0),
		bsize2str(pbl, tmpbuf, sizeof(tmpbuf)));
		
    dp->sector = strdup(valbuf);

    if (!dp->size) {
	dp->size = size2str(size);
    }
    
    if (f_debug) {
	fprintf(stderr, "\tSCSI READ CAPACITY: lba       = %zu\n", lba);
	fprintf(stderr, "\tSCSI READ CAPACITY: size      = %zu (%s)\n", size, size2str(size));
	fprintf(stderr, "\tSCSI READ CAPACITY: lbl       = %u\n", lbl);
	fprintf(stderr, "\tSCSI READ CAPACITY: pbl       = %u\n", pbl);
	fprintf(stderr, "\tSCSI READ CAPACITY: rc_basis  = 0x%02x\n", rc_basis);
	fprintf(stderr, "\tSCSI READ CAPACITY: p_type    = %u\n", p_type);
	fprintf(stderr, "\tSCSI READ CAPACITY: prot_en   = %u\n", prot_en);
	fprintf(stderr, "\tSCSI READ CAPACITY: pie       = %u\n", pie);
	fprintf(stderr, "\tSCSI READ CAPACITY: lbpbe     = %u\n", lbpbe);
	fprintf(stderr, "\tSCSI READ CAPACITY: lbpme     = %u\n", lbpme);
	fprintf(stderr, "\tSCSI READ CAPACITY: lbprz     = %u\n", lbprz);
	fprintf(stderr, "\tSCSI READ CAPACITY: lalba     = %u\n", lalba);
    }

    return 1;
}


ssize_t
drive_scsi_read_capacity(DRIVE *dp) {
    uint8_t buffer[32];
    ssize_t res;

    if (f_debug)
	fprintf(stderr, "    [scsi_read_capacity]\n");
    
    res = drive_scsi_get_read_capacity(dp, buffer, sizeof(buffer));
    if (res < 0)
	return res;

    return drive_scsi_parse_read_capacity(dp, buffer, res);
}

static const char *
type2str(uint8_t type) {
    static char buf[64];
    
    switch (type) {
    case 0x0:
	return "-";
    case 0x1:
	return "CFast";
    case 0x2:
	return "CF";
    case 0x3:
	return "MS";
    case 0x4:
	return "MMC";
    case 0x5:
	return "SD";
    case 0x6:
	return "XQD";
    case 0x7:
	return "UFS";
    default:
	snprintf(buf, sizeof(buf), "#%u", type);
	return buf;
    }
}


ssize_t
drive_scsi_parse_blockdev_characteristics(DRIVE *dp,
					  uint8_t *buffer,
					  size_t bufsize) {
    uint32_t rpm;
    uint8_t type;
    uint8_t form;
    uint16_t buflen;
    

    if (buffer[1] != 0xB1) {
	if (f_debug)
	    fprintf(stderr, "%s%u: scsi blockdev: invalid response\n",
		    dp->cam->given_dev_name, dp->cam->given_unit_number);
	errno = EIO;
	return -1;
    }
	    
    buflen = get_be(buffer+2, 2);

    if (buflen+4 > bufsize) {
	if (f_debug)
	    fprintf(stderr, "%s%u: scsi blockdev: bufsize = %zu, buflen = %u -> %zu\n",
		    dp->cam->given_dev_name, dp->cam->given_unit_number,
		    bufsize, buflen,
		    bufsize-4);
	buflen = bufsize-4;
    }
    
    if (buflen < 2) {
	if (f_debug)
	    fprintf(stderr, "%s%u: scsi blockdev: buflen = %u < 2\n",
		    dp->cam->given_dev_name, dp->cam->given_unit_number, buflen);
	errno = EIO;
	return -1;
    }
    
    rpm  = get_be(buffer+4, 2);
    dp->rpm  = strdup(rpm2str(rpm));
    if (f_debug)
	fprintf(stderr, "\tSCSI BLOCKDEV CHAR: rpm  = %u (%s)\n", rpm, dp->rpm);
    
    if (buflen > 2) {
	type = buffer[6];
	if (f_debug)
	    fprintf(stderr, "\tSCSI BLOCKDEV CHAR: type = %u (%s)\n", type, type2str(type));

	if (buflen > 3) {
	    form = buffer[7] % 0x0F;
	    dp->form = strdup(form2str(form));
	    if (f_debug)
		fprintf(stderr, "\tSCSI BLOCKDEV CHAR: form = %u (%s)\n", form, dp->form);
	}
    }

    
    return 1;
}

ssize_t
drive_scsi_blockdev_characteristics(DRIVE *dp) {
    uint8_t buffer[64];
    ssize_t res;

    if (f_debug)
	fprintf(stderr, "    [scsi_blockdev_characteristics]\n");
    
    res = drive_scsi_get_vpd_page(dp, SCSI_BLOCKDEV_CHARACTERISTICS_PAGE,
				  buffer, sizeof(buffer));
    if (res < 0)
	return res;

    return drive_scsi_parse_blockdev_characteristics(dp, buffer, res);
}




int
drive_scsi_health_status(DRIVE *dp) {
    int rc1 = 0, rc2 = 0, rc3 = 0, rc4 = 0, rc5 = 0, rc6 = 0;
    struct supported_log_pages pages;


    drive_scsi_inquiry(dp);

    if (f_debug > 1) {
	drive_scsi_dump_vpd_pages(dp);
    }
    
    memset(&pages, 0, sizeof(pages));
    drive_scsi_supported_log_pages(dp, &pages);

    /* Get Drive PowerOnHours */
    if (pages.background_scan && f_hours) {
	if (f_debug)
	    fprintf(stderr, "  [background_scan]\n");
	rc1 = drive_scsi_background_scan(dp);
    }

    /* Get SMART Status & Drive Temperature */
    if (pages.ie && (f_smart || f_temp)) {
	if (f_debug)
	    fprintf(stderr, "  [ie]\n");
	rc2 = drive_scsi_ie_status(dp);
    }

    /* Skip if we already got it from IE */
    if (pages.temp && (f_temp && dp->raw.temperature == DRIVE_NO_TEMPERATURE)) {
	if (f_debug)
	    fprintf(stderr, "  [temperature]\n");
	rc3 = drive_scsi_temperature(dp);
    }

    if (pages.selftest && (f_tests || (f_hours && dp->raw.power_on_hours == DRIVE_NO_HOURS))) {
	if (f_debug)
	    fprintf(stderr, "  [selftest]\n");
	rc4 = drive_scsi_selftest_results_status(dp);
    }

    if (pages.ssd_media && f_wear) {
	if (f_debug)
	    fprintf(stderr, "  [ssd]\n");
	rc4 = drive_scsi_endurance_used(dp);
    }

    if (pages.format_status && (f_hours && dp->raw.power_on_hours == DRIVE_NO_HOURS)) {
	if (f_debug)
	    fprintf(stderr, "  [format]\n");
	rc5 = drive_scsi_format_status_poweron(dp);
    }

    if (pages.start_stop_cycle && f_wear) {
	if (f_debug)
	    fprintf(stderr, "  [start-stop cycle]\n");
	rc6 = drive_scsi_start_stop_cycle(dp);
    }

    /* Get sector size (and size and stuff) */
    drive_scsi_read_capacity(dp);

    /* Get rpm */
    if (1 || pages.blockdev_chars) {
	drive_scsi_blockdev_characteristics(dp);
    }
    
    if (rc1 > 0 || rc2 > 0 || rc3 > 0 || rc4 > 0 || rc5 > 0 || rc6 > 0)
	return 1;
    
    if (rc1 == 0 && rc2 == 0 && rc3 == 0 && rc4 == 0 && rc5 == 0 && rc6 == 0)
	return 0;
    
    return -1;
}
