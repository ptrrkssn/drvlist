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

#include "utils.h"
#include "scsi.h"

extern int f_debug;
extern char *argv0;


#define SCSI_LOG_SENSE                       0x4d

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
    unsigned int start_stop_cycle : 1;
};


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

int
drive_scsi_has_vpd_page(DRIVE *dp,
			uint8_t page_id,
			int retry_count,
			int timeout,
			int verbosemode) {
	union ccb *ccb = NULL;
	struct scsi_vpd_supported_page_list sup_pages;
	int i;
	int retval = 0;


	ccb = cam_getccb(dp->cam);
	if (ccb == NULL)
	    return -1;

	memset(&sup_pages, 0, sizeof(sup_pages));

	scsi_inquiry(&ccb->csio,
	 /*retries*/ retry_count,
	  /*cbfcnp*/ NULL,
    /* tag_action */ CAM_TAG_ACTION_NONE, /* MSG_SIMPLE_Q_TAG, */
       /* inq_buf */ (uint8_t *)&sup_pages,
       /* inq_len */ sizeof(sup_pages),
	  /* evpd */ 1,
     /* page_code */ SVPD_SUPPORTED_PAGE_LIST,
     /* sense_len */ SSD_FULL_SIZE,
       /* timeout */ timeout ? timeout : 5000);

	/* Disable freezing the device queue */
	ccb->ccb_h.flags |= CAM_DEV_QFRZDIS;

	if (retry_count != 0)
		ccb->ccb_h.flags |= CAM_PASS_ERR_RECOVER;

	if (cam_send_ccb(dp->cam, ccb) < 0) {
		cam_freeccb(ccb);
		ccb = NULL;
		retval = -1;
		goto bailout;
	}

	if ((ccb->ccb_h.status & CAM_STATUS_MASK) != CAM_REQ_CMP) {
		if (verbosemode != 0)
			cam_error_print(dp->cam, ccb, CAM_ESF_ALL,
					CAM_EPF_ALL, stderr);
		retval = -1;
		goto bailout;
	}

	for (i = 0; i < sup_pages.length; i++) {
		if (sup_pages.list[i] == page_id) {
			retval = 1;
			goto bailout;
		}
	}
bailout:
	if (ccb != NULL)
		cam_freeccb(ccb);

	return (retval);
}

ssize_t
drive_scsi_cmd_log_sense(DRIVE *dp,
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
    

    ccb = cam_getccb(dp->cam);
    if (!ccb)
	return -1;

    memset(buf, 0, bufsize);

    cam_fill_csio(&ccb->csio,
		  1,                      /* retries */
		  NULL,                   /* callback */
		  CAM_DIR_IN | CAM_DEV_QFRZDIS,
		  CAM_TAG_ACTION_NONE,
		  buf,
		  bufsize,
		  SSD_FULL_SIZE,
		  sizeof(*cdb),           /* CDB length */
		  5000);                  /* timeout ms */
    
    cdb = (struct log_sense_cdb *)&ccb->csio.cdb_io.cdb_bytes;
    memset(cdb, 0, sizeof(*cdb));

    cdb->page = (1 << 6) | page;
    cdb->subpage = subpage;
    cdb->opcode = SCSI_LOG_SENSE;
    put_be16(cdb->alloc_len, bufsize);
    
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

    if (cam_resid < 0 ||
	(size_t)ccb->csio.resid > bufsize) {
	errno = EIO;
	return -1;
    }
	
    /*
     * Determine how many bytes CAM actually transferred.
     */
    if (ccb->csio.resid < 0 ||
        (size_t)ccb->csio.resid > bufsize) {
        cam_freeccb(ccb);
        errno = EIO;
        return -1;
    }
    got = bufsize - (size_t)ccb->csio.resid;

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
    
    return len;
}
	       

int
drive_scsi_foreach_log_sense(DRIVE *dp,
			     uint8_t page,
			     uint8_t subpage,
			     int (*param_handler)(uint16_t code, uint8_t control, uint8_t length, uint8_t *data, void *xp),
			     void *xp) {
    uint8_t buf[LOG_SENSE_BUFFER_SIZE];
    ssize_t len;
    off_t off;
    struct log_sense_parameter *pp;
    int rc = 0;
    
    
    len = drive_scsi_cmd_log_sense(dp, page, subpage, buf, sizeof(buf));
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

    
    len = drive_scsi_cmd_log_sense(dp,
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
    return drive_scsi_foreach_log_sense(dp,
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
    return drive_scsi_foreach_log_sense(dp,
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
	    rc = 1;
	}
	break;
	
    default:
	if (f_debug)
	    fprintf(stderr, "\tSSD MEDIA LOG: code 0x%04x\n",
		    code);
	break;
    }

    return rc;
}

int
drive_scsi_endurance_used(DRIVE *dp) {
    return drive_scsi_foreach_log_sense(dp,
					SCSI_SSD_MEDIA_LOG_PAGE,
					SCSI_SSD_MEDIA_LOG_SUBPAGE,
					ssd_media_handler,
					(void *) dp);
}



int
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
	if (dp->raw.power_on_hours == DRIVE_NO_POWER_ON_HOURS || poh > dp->raw.power_on_hours)
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
    return drive_scsi_foreach_log_sense(dp,
					SCSI_FORMAT_STATUS_LOG_PAGE,
					SCSI_FORMAT_STATUS_LOG_SUBPAGE,
					format_status_handler,
					(void *) dp);
}


int
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
    return drive_scsi_foreach_log_sense(dp,
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
    case 0x8:
        return (DRIVE_SELFTEST_FAILED);

    case 0x9:
        return (DRIVE_SELFTEST_RUNNING);

    case 0xf:
        return (DRIVE_SELFTEST_NONE);

    default:
        return (DRIVE_SELFTEST_UNKNOWN);
    }
}

int
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
    if (poh == 0x0000 || poh == 0xffff)
	return 0;
    
    if (dp->raw.power_on_hours == DRIVE_NO_POWER_ON_HOURS || poh > dp->raw.power_on_hours)
	dp->raw.power_on_hours = poh;
    
    /* Parameter code 1 is normally the newest entry */
    if (code == 0x0001)
	dp->raw.selftest.status = classify_selftest_result(selftest_result);
    
    dp->raw.selftest.tests++;
    switch (dp->raw.selftest.status) {
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
   return drive_scsi_foreach_log_sense(dp,
				       SCSI_SELFTEST_RESULTS_LOG_PAGE,
				       SCSI_SELFTEST_RESULTS_LOG_SUBPAGE,
				       selftest_results_handler,
				       (void *) dp);
}




int
drive_scsi_is_satl(DRIVE *dp) {
    return drive_scsi_has_vpd_page(dp, SVPD_ATA_INFORMATION, 1, 0, 0);
}


int
drive_scsi_health_status(DRIVE *dp) {
    int rc1 = 0, rc2 = 0, rc3 = 0, rc4 = 0, rc5 = 0;
    struct supported_log_pages pages;


    if (f_debug)
	fprintf(stderr, "  [supported]\n");

    memset(&pages, 0, sizeof(pages));
    drive_scsi_supported_log_pages(dp, &pages);
    

    if (pages.ie) {
	if (f_debug)
	    fprintf(stderr, "  [ie]\n");
	rc2 = drive_scsi_ie_status(dp);
    }

    /* Skip if we already got it from IE */
    if (pages.temp && dp->raw.temperature == 0xFF) {
	if (f_debug)
	    fprintf(stderr, "  [temp]\n");
	rc1 = drive_scsi_temperature(dp);
    }


    if (pages.selftest) {
	if (f_debug)
	    fprintf(stderr, "  [selftest]\n");
	rc3 = drive_scsi_selftest_results_status(dp);
    }

    if (pages.ssd_media) {
	if (f_debug)
	    fprintf(stderr, "  [ssd]\n");
	rc4 = drive_scsi_endurance_used(dp);
    }

    if (pages.format_status) {
	if (f_debug)
	    fprintf(stderr, "  [format]\n");
	rc5 = drive_scsi_format_status_poweron(dp);
    }

    if (pages.start_stop_cycle) {
	if (f_debug)
	    fprintf(stderr, "  [start-stop cycle]\n");
	drive_scsi_start_stop_cycle(dp);
    }

    if (rc1 > 0 || rc2 > 0 || rc3 > 0 || rc4 > 0 || rc5 > 0)
	return 1;
    
    if (rc1 == 0 && rc2 == 0 && rc3 == 0 && rc4 == 0 && rc5 == 0)
	return 0;
    
    return -1;
}
