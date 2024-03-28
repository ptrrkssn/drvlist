/*
 * drvlist.c
 *
 * A small FreeBSD utility to enumerate installed drives in a system.
 *
 * Copyright (c) 2024 Peter Eriksson <pen@lysator.liu.se>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/sysctl.h>
#include <sys/disk.h>
#include <camlib.h>
#include <cam/scsi/scsi_message.h>
#include <cam/ata/ata_all.h>
#include <cam/mmc/mmc_all.h>
#include <dev/nvme/nvme.h>



int f_verbose = 0;

typedef struct {
    char *ident;
    char *danames;
    char *vendor;
    char *product;
    char *revision;
    char *physpath;
} DISK;

#define MAXDISK 2048

int dc = 0;
int ds = 0;
DISK *dv;


static int
dv_sort_ident(const void *a, const void *b) {
    const DISK *da = (const DISK *) a;
    const DISK *db = (const DISK *) b;
    return strcmp(da->ident, db->ident);
}


char *
strdupcat(char **old,
	  char *add) {
    size_t olen = 0;
    size_t alen = 0;
    
    if (*old)
	olen += strlen(*old);
    
    if (olen > 0)
	++alen;
    alen += strlen(add);

    *old = realloc(*old, olen+alen+1);
    if (!*old)
	return NULL;
    
    if (olen > 0)
	(*old)[olen++] = ',';
    
    strcpy((*old)+olen, add);
    return *old;
}

int
strtrim(char *str,
	int *len) {
    int n = strlen(str);

    while (n > 0 && isspace(str[n-1]))
	--n;
    str[n] = '\0';
    if (n > *len)
	*len = n;
    return n;
}


static int
ata_cam_send(struct cam_device *device, union ccb *ccb)
{
	/* Disable freezing the device queue */
	ccb->ccb_h.flags |= CAM_DEV_QFRZDIS;


	if (cam_send_ccb(device, ccb) < 0) {
		return (1);
	}

	/*
	 * Consider any non-CAM_REQ_CMP status as error and report it here,
	 * unless caller set AP_FLAG_CHK_COND, in which case it is responsible.
	 */
	if (!(ccb->ataio.cmd.flags & CAM_ATAIO_NEEDRESULT) &&
	    (ccb->ccb_h.status & CAM_STATUS_MASK) != CAM_REQ_CMP) {
		return (1);
	}

	return (0);
}


static int
ata_do_cmd(struct cam_device *device, union ccb *ccb, int retries,
	   uint32_t flags, uint8_t protocol, uint8_t ata_flags,
	   uint8_t tag_action, uint8_t command, uint16_t features,
	   u_int64_t lba, uint16_t sector_count, uint8_t *data_ptr,
	   uint16_t dxfer_len, int timeout, int force48bit)
{
	CCB_CLEAR_ALL_EXCEPT_HDR(&ccb->ataio);
	cam_fill_ataio(&ccb->ataio,
		       retries,
		       NULL,
		       flags,
		       tag_action,
		       data_ptr,
		       dxfer_len,
		       timeout);

	if (force48bit || lba > ATA_MAX_28BIT_LBA)
		ata_48bit_cmd(&ccb->ataio, command, features, lba, sector_count);
	else
		ata_28bit_cmd(&ccb->ataio, command, features, lba, sector_count);

	if (ata_flags & AP_FLAG_CHK_COND)
		ccb->ataio.cmd.flags |= CAM_ATAIO_NEEDRESULT;

	return ata_cam_send(device, ccb);
}

int
ata_identify(struct cam_device *cdb,
	     char **vendor,
	     char **model,
	     char **revision) {
    union ccb *ccb;
    struct ata_params apb;
    uint8_t *bp;
    u_int i, error;
    uint8_t command, retry_command;
    
    
    if ((ccb = cam_getccb(cdb)) == NULL) {
	return -1;
    }

    command = ATA_ATA_IDENTIFY;
    retry_command = ATA_ATAPI_IDENTIFY;
    
 retry:
    error = ata_do_cmd(cdb,
		       ccb,
		       1, /*retries*/
		       CAM_DIR_IN, /*flags*/
		       AP_PROTO_PIO_IN, /*protocol*/
		       AP_FLAG_BYT_BLOK_BLOCKS | AP_FLAG_TLEN_SECT_CNT, /*ata_flags*/
		       MSG_SIMPLE_Q_TAG, /*tag_action*/
		       command, /*command*/
		       0, /*features*/
		       0, /*lba*/
		       sizeof(struct ata_params) / 512, /*sector_count*/
		       (uint8_t *)&apb, /*data_ptr*/
		       sizeof(apb), /*dxfer_len*/
		       30000, /* timeout */
		       0 /*force48bit*/);
    
    if (error != 0) {
	if (retry_command != 0) {
	    command = retry_command;
	    retry_command = 0;
	    goto retry;
	}
	return (1);
    }
    
    ata_param_fixup(&apb);
    
    error = 1;
    bp = (uint8_t *) &apb;
    for (i = 0; i < sizeof(apb); i++) {
	if (bp[i] != 0)
	    error = 0;
    }
    
    /* check for invalid (all zero) response */
    if (error != 0) {
	return (error);
    }
    
    cam_freeccb(ccb);
    
    *vendor = strdup("ATA");
    *model = strndup((char *) apb.model, sizeof(apb.model));
    *revision = strndup((char *) apb.revision, sizeof(apb.revision));
    return 0;
}



int
nvme_identify(int fd,
	      char *daname,
	      char *pnbuf) {
    struct nvme_pt_command pt;
    struct nvme_controller_data cdata;
    char *ident = NULL;
    DISK *dp;
    char *cp;
    int i, j;
    
    
    memset(&pt, 0, sizeof(pt));
    pt.cmd.opc = NVME_OPC_IDENTIFY;
    pt.cmd.cdw10 = htole32(1);
    pt.buf = &cdata;
    pt.len = sizeof(cdata);
    pt.is_read = 1;
    
    if (ioctl(fd, NVME_PASSTHROUGH_CMD, &pt) < 0) {
	fprintf(stderr, "NVME ioctl: %s\n", strerror(errno));
	return -1;
    }
	
    if (nvme_completion_is_error(&pt.cpl)) {
	fprintf(stderr, "NVME nvme_completion\n");
	return -1;
    }
	
    ident = strndup(((const char*)cdata.sn), NVME_SERIAL_NUMBER_LENGTH);
    if (!ident) {
	fprintf(stderr, "NVME No IDENTn\n");
	return -1;
    }
    
    for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
	;
    
    dp = &dv[i];
    
    if (i >= dc) {
	dp->vendor = strndup(((const char*)cdata.mn), NVME_MODEL_NUMBER_LENGTH);
	for (j = NVME_MODEL_NUMBER_LENGTH-1; j >= 0 && isspace(dp->vendor[j]) ; j--)
	    ;
	dp->vendor[j+1] = '\0';
	for (cp = dp->vendor; *cp && !isspace(*cp); ++cp)
	    ;
	if (*cp) {
	    *cp++ = '\0';
	    while (isspace(*cp))
		    ++cp;
	    dp->product = strdup(cp);
	}
	dp->revision = strndup(((const char*)cdata.fr), NVME_FIRMWARE_REVISION_LENGTH);
	dp->ident = ident;
	dp->danames = strdup(daname);
	dp->physpath = strdup(pnbuf);
	++dc;
	return 1;
    }
    
    strdupcat(&dp->danames, daname);
    strdupcat(&dp->physpath, pnbuf);
    return 0;
}

	  
int
main(int argc,
     char *argv[]) {
    char *daname, path[2048];
    char *buf = NULL;
    size_t bsize = 0;
    int i, j;
    int rc = 0;
    int identlen = 7;
    int vendorlen = 6;
    int productlen = 7;
    int revisionlen = 4;
    int danameslen = 5;
    int physpathlen = 4;
    

    dv = malloc((ds = 1024)*sizeof(DISK));
    
    for (i = 1; i < argc && argv[i][0] == '-'; i++)
	for (j = 1; argv[i][j]; j++)
	    switch (argv[i][j]) {
	    case 'h':
		printf("Usage: %s [-v]\n", argv[0]);
		exit(0);
	    case 'v':
		f_verbose++;
		break;
	    default:
		fprintf(stderr, "%s: Error: -%c: Invalid switch\n",
			argv[0], argv[i][j]);
		exit(1);
	    }
    
    if (sysctlbyname("kern.disks", NULL, &bsize, NULL, 0) < 0) {
	fprintf(stderr, "%s: Error: Unable to list of drives from kernel: %s\n",
		argv[0], strerror(errno));
	exit(1);
    }

    buf = malloc(bsize);
    if (!buf) {
	fprintf(stderr, "%s: Error: %lu: Memory allocation failure: %s\n",
		argv[0], bsize, strerror(errno));
	exit(1);
    }

    if (sysctlbyname("kern.disks", buf, &bsize, NULL, 0) < 0) {
	fprintf(stderr, "%s: Error: Unable to get list of drives from kernel: %s\n",
		argv[0], strerror(errno));
	exit(1);
    }

    strcpy(path, "/dev/");

    while ((daname = strsep(&buf, " ")) != NULL) {
	int fd, id;
	char *ident = NULL;
	struct cam_device *cam;
	DISK *dp;
	char idbuf[DISK_IDENT_SIZE];
	char pnbuf[MAXPATHLEN];

	if (dc >= ds) {
	    dv = realloc(dv, (ds += 1024)*sizeof(DISK));
	    if (!dv) {
		fprintf(stderr, "%s: Error: Memory allocation: %s\n",
			argv[0], strerror(errno));
		exit(1);
	    }
	}

	strcpy(path+5, daname);
	
	cam = cam_open_device(path, O_RDWR);
	if (cam) {
	    ident = strndup((char *) &cam->serial_num[0], cam->serial_num_len);
	    if (!ident) {
		fprintf(stderr, "%s: Error: Memory allocation failure\n",
			argv[0]);
		exit(1);
	    }

	    for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
		;
	    dp = &dv[i];
	    
	    sprintf(pnbuf, "%s%u bus %u target %u lun %jx",
		    cam->sim_name, cam->sim_unit_number,
		    cam->bus_id,
		    cam->target_id,
		    cam->target_lun);
	    
	    if (sscanf(daname, "nda%u", &id) == 1) {
		    sprintf(path+5, "nvme%d", id);

		fd = open(path, O_RDONLY);
		nvme_identify(fd, daname, pnbuf);
		close(fd);
	    } else {
		if (sscanf(daname, "ada%u", &id) == 1) {
		    ata_identify(cam, &dp->vendor, &dp->product, &dp->revision);
		}
		if (i >= dc) {
		    dp->ident = ident;

		    if (!dp->vendor && cam->inq_data.vendor[0])
			    dp->vendor = strndup(cam->inq_data.vendor, sizeof(cam->inq_data.vendor));
		    if (!dp->product && cam->inq_data.product[0])
			    dp->product = strndup(cam->inq_data.product, sizeof(cam->inq_data.product));
		    if (!dp->revision && cam->inq_data.revision[0])
			    dp->revision = strndup(cam->inq_data.revision, sizeof(cam->inq_data.revision));
		    
		    dp->danames = strdup(daname);
		    dp->physpath = strdup(pnbuf);
		    dc++;
		} else {
		    strdupcat(&dp->danames, daname);
		    strdupcat(&dp->physpath, pnbuf);
		}
	    }
	    
	    cam_close_device(cam);
	    continue;
	}
	
	if (sscanf(daname, "nvd%d", &id) == 1)
	    sprintf(path+5, "nvme%d", id);

	
	/* Non-CAM */
	fd = open(path, O_RDONLY|O_DIRECT, 0);
	if (fd < 0) {
	    fprintf(stderr, "%s: Error: %s: Open: %s\n",
		    argv[0], path, strerror(errno));
	    rc = 1;
	    continue;
	}
	
	
	if (strncmp(daname, "nvd", 3) == 0) {
	    nvme_identify(fd, daname, path+5);
	    continue;
	}

	/* Non NVME - ATA?*/
	fprintf(stderr, "%s: Non CAM, Non NVME\n", daname);

	memset(idbuf, 0, sizeof(idbuf));
	if (ioctl(fd, DIOCGIDENT, idbuf) >= 0)
	    ident = strndup(idbuf, sizeof(idbuf));
	
	if (!ident) {
	    fprintf(stderr, "%s: Error: %s: Unable to get serial number: %s\n",
		    argv[0], daname, strerror(errno));
	    close(fd);
	    rc = 1;
	    continue;
	}
	
	for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
	    ;
	dp = &dv[i];
	
	if (i >= dc) {
	    dp->ident = ident;
	    dp->danames = strdup(daname);
	    ++dc;
	} else {
	    strdupcat(&dp->danames, daname);
	}
	
	close(fd);
    }
    
    if (!dc)
	return 0;
    
    for (i = 0; i < dc; i++) {
	strtrim(dv[i].ident, &identlen);
	strtrim(dv[i].vendor, &vendorlen);
	strtrim(dv[i].product, &productlen);
	strtrim(dv[i].revision, &revisionlen);
	strtrim(dv[i].danames, &danameslen);
	strtrim(dv[i].physpath, &physpathlen);
    }

    qsort(&dv[0], dc, sizeof(dv[0]), dv_sort_ident);

    if (isatty(1)) {
	printf("\033[1;4m%-*s : %-*s : %-*s : %-*s : %-*s",
	       identlen, "IDENT",
	       vendorlen, "VENDOR",
	       productlen, "PRODUCT",
	       revisionlen, "REV.",
	       danameslen, "NAMES");
	if (f_verbose) {
	    printf(" : %-*s",
		   physpathlen, "PATH");
	}
	puts("\033[0m");
    }

    for (i = 0; i < dc; i++) {
	printf("%-*s : %-*s : %-*s : %-*s : %-*s",
	       identlen, dv[i].ident,
	       vendorlen, dv[i].vendor,
	       productlen, dv[i].product,
	       revisionlen, dv[i].revision,
	       danameslen, dv[i].danames);
	if (f_verbose) {
	    printf(" : %s",
		   dv[i].physpath ? dv[i].physpath : "-");
	}
	putchar('\n');
    }

    return rc;
}
