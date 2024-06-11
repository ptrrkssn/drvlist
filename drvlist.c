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
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/sysctl.h>
#include <sys/disk.h>
#include <sys/stat.h>
#include <camlib.h>
#include <cam/scsi/scsi_message.h>
#include <cam/ata/ata_all.h>
#include <cam/mmc/mmc_all.h>
#include <dev/nvme/nvme.h>



int f_verbose = 0;
int f_debug = 0;
int f_phys = 0;
int f_maxwidth = 20;

char *f_sort = NULL;


typedef struct {
    char *ident;
    char *danames;
    char *vendor;
    char *product;
    char *revision;
    char *driver;
    char *path;
    char *phys;
    char *size;
} DISK;

#define MAXDISK 2048

int dc = 0;
int ds = 0;
DISK *dv;


static int
dv_sort_ident(const DISK *da, const DISK *db) {
    return strcmp(da->ident, db->ident);
}

static int
dv_sort_devpath(const DISK *da, const DISK *db) {
    int d;

    d = strcmp(da->driver, db->driver);
    if (d)
	return d;
    
    return strcmp(da->path, db->path);
}

static int
dv_sort(const void *a, const void *b) {
    if (f_sort) {
	if (strcmp(f_sort, "ident") == 0)
	    return dv_sort_ident((const DISK *) a, (const DISK *) b);
    }
    
    return dv_sort_devpath((const DISK *) a, (const DISK *) b);
}


char *
strdupcat(char **old,
	  char *add) {
    size_t olen = 0;
    size_t alen = 0;
    
    if (*old) {
	if (strcmp(*old, add) == 0)
	    return 0;
	
	olen += strlen(*old);
    }
    
    if (olen > 0)
	++alen;
    alen += strlen(add);

    if (*old && strcmp(*old, add) > 0) {
	char *new = malloc(olen+alen+1);
	if (!new)
	    return NULL;
	strcpy(new, add);
	strcat(new, ",");
	strcat(new, *old);
	free(*old);
	*old = new;
    } else {
	*old = realloc(*old, olen+alen+1);
	if (!*old)
	    return NULL;
	
	if (olen > 0)
	    (*old)[olen++] = ',';
	
	strcpy((*old)+olen, add);
    }
    
    return *old;
}

int
strtrim(char *str,
	int *len) {
    int i, j, n;

    
    if (!str)
	return 0;

    /* Remove leading whitespace */
    for (i = 0; str[i] && isspace(str[i]); i++)
	;
    if (i > 0) {
	for (j = 0; str[i]; j++, i++)
	    str[j] = str[i];
	str[j] = '\0';
    }

    /* Remove trailing whitespace */
    n = strlen(str);
    while (n > 0 && isspace(str[n-1]))
	--n;
    str[n] = '\0';
    if (len && n > *len)
	*len = n;
    return n;
}


int strntrim(char *str,
	     int *len,
	     int max) {
    int rlen = strtrim(str, len);

    if (max == 0)
	return rlen;
    
    if (rlen+2 > max) {
	str[max-2] = '.';
	str[max-1] = '.';
	str[max] = '\0';
	rlen = max;
	if (len)
	    *len = rlen;
    }

    return rlen;
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
	      char *driver,
	      char *pnbuf) {
    struct nvme_pt_command pt;
    struct nvme_controller_data cdata;
    char *ident = NULL;
    DISK *dp;
    char *cp;
    int i, j;
    char pbuf[MAXPATHLEN];
    
    
    memset(&pt, 0, sizeof(pt));
    memset(&cdata, 0, sizeof(cdata));
    
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

	if (!pnbuf) {
	    sprintf(pbuf, "pci vendor 0x%04x:0x%04x oui %02x:%02x:%02x controller 0x%04x",
		    cdata.vid, cdata.ssvid,
		    cdata.ieee[0], cdata.ieee[1], cdata.ieee[2],
		    cdata.ctrlr_id);
	    pnbuf = pbuf;
	}

	dp->revision = strndup(((const char*)cdata.fr), NVME_FIRMWARE_REVISION_LENGTH);
	dp->ident = ident;
	dp->danames = strdup(daname);
	dp->driver = strdup(driver);
	dp->path = strdup(pnbuf);
	dp->phys = NULL;
	++dc;
	return 1;
    }
    
    strdupcat(&dp->danames, daname);
    strdupcat(&dp->driver, driver);
    strdupcat(&dp->path, pnbuf);
    return 0;
}


void
p_strip(const char *s) {
    while (*s) {
	putchar(*s);
	if (isspace(*s))
	    while (isspace(*s))
		++s;
	else
	    ++s;
    }
}

char *
size2str(off_t size) {
    char buf[256];
    double ds = size;
    
    if (size < 2000) {
	sprintf(buf, "%lu", size);
	return strdup(buf);
    }

    ds /= 1000;
    if (ds < 2000) {
	sprintf(buf, "%.0fK", ds);
	return strdup(buf);
    }
    
    ds /= 1000;
    if (ds < 2000) {
	sprintf(buf, "%.0fM", ds);
	return strdup(buf);
    }
    
    ds /= 1000;
    if (ds < 2000) {
	sprintf(buf, "%.0fG", ds);
	return strdup(buf);
    }
	
    
    ds /= 1000;
    if (ds < 2000) {
	sprintf(buf, "%.0fT", ds);
	return strdup(buf);
    }
	
    
    ds /= 1000;
    sprintf(buf, "%.0fP", ds);
    return strdup(buf);
}


int
do_device(char *daname) {
    int fd, id, i;
    char *ident = NULL;
    struct cam_device *cam;
    DISK *dp;
    char path[2048];
    char idbuf[DISK_IDENT_SIZE];
    char pnbuf[MAXPATHLEN];
    char drvbuf[MAXPATHLEN];
    char physbuf[MAXPATHLEN];
    off_t msize = 0;

    
    if (dc >= ds) {
	dv = realloc(dv, (ds += 1024)*sizeof(DISK));
	if (!dv)
	    return -1;
    }

    if (strncmp(daname, "/dev/", 5) == 0) {
	strcpy(path, daname);
	daname = path+5;
    } else {
	strcpy(path, "/dev/");
	strcpy(path+5, daname);
    }

    if (1) {
	int fd;
	
	fd = open(path, O_RDONLY);
	if (fd >= 0 && ioctl(fd, DIOCGMEDIASIZE, &msize) >= 0) {
	    if (f_debug)
		fprintf(stderr, "*** path=%s msize=%s (%lu)\n", path, size2str(msize), msize);
	}
	close(fd);
    }    
    
    cam = cam_open_device(path, O_RDWR);
    if (cam) {
	if (f_debug) {
	    fprintf(stderr, "*** path=%s dev=%s%u pass=%s%u\n",
		    path,
		    cam->given_dev_name,
		    cam->given_unit_number,
		    cam->device_name,
		    cam->dev_unit_num);
	}

	physbuf[0] = '\0';
	if (f_phys) {
	    int fd = open(path, O_RDONLY);
	    
	    if (fd >= 0) {
		ioctl(fd, DIOCGPHYSPATH, physbuf);
		close(fd);
	    }
	}
	
	ident = strndup((char *) &cam->serial_num[0], cam->serial_num_len);
	if (!ident)
	    return -1;
	
	for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
	    ;
	dp = &dv[i];
	
	if (f_verbose > 1)
	    sprintf(drvbuf, "%s%u @ bus %u",
		    cam->sim_name, cam->sim_unit_number, cam->bus_id);
	else
	    sprintf(drvbuf, "%s%u",
		    cam->sim_name, cam->sim_unit_number);
	
	sprintf(pnbuf, "scbus %2u target %3u lun %2jx",
		cam->path_id,
		cam->target_id,
		cam->target_lun);
	
	if (sscanf(daname, "nda%u", &id) == 1) {
	    sprintf(path+5, "nvme%d", id);
	    
	    fd = open(path, O_RDONLY);
	    nvme_identify(fd, daname, drvbuf, pnbuf);
	    free(ident);
	    close(fd);
	} else {
	    if (sscanf(daname, "ada%u", &id) == 1) {
		ata_identify(cam, &dp->vendor, &dp->product, &dp->revision);
	    }
	    if (i >= dc) {
		char *cp;
		dp->ident = ident;
		
		
		if (!dp->vendor && cam->inq_data.vendor[0]) {
		    dp->vendor = strndup(cam->inq_data.vendor, sizeof(cam->inq_data.vendor));
		    strtrim(dp->vendor, NULL);
		}
		if (!dp->product && cam->inq_data.product[0]) {
		    dp->product = strndup(cam->inq_data.product, sizeof(cam->inq_data.product));
		    strtrim(dp->product, NULL);
		}

		if (!dp->revision && cam->inq_data.revision[0]) {
		    dp->revision = strndup(cam->inq_data.revision, sizeof(cam->inq_data.revision));
		    strtrim(dp->revision, NULL);
		}

		if (dp->vendor && dp->product) {
		    if ((strcmp(dp->vendor, "ATA") == 0 || strcmp(dp->vendor, "USB") == 0) &&
			(cp = strchr(dp->product, ' ')) != NULL) {
			if (cp[1] != '\0' && !isspace(cp[1])) {
			    free(dp->vendor);
			    *cp++ = '\0';
			    dp->vendor = dp->product;
			    dp->product = strdup(cp);
			}
		    }
		    if ((strcmp(dp->vendor, "ATA") == 0 || strcmp(dp->vendor, "USB") == 0)) {
			    /* Hack... */
			if (strncmp(dp->product, "SSDSC", 5) == 0) {
			    free(dp->vendor);
			    dp->vendor = strdup("INTEL");
			} else if (strncmp(dp->product, "MZ", 2) == 0) {
			    free(dp->vendor);
			    dp->vendor = strdup("SAMSUNG");
			}
		    }
		}
		
		dp->danames = strdup(daname);
		dp->phys = strdup(physbuf);
		dp->driver = strdup(drvbuf);
		dp->path = strdup(pnbuf);
		dp->phys = strdup(physbuf);
		if (msize > 0)
		    dp->size = size2str(msize);
		dc++;
	    } else {
		free(ident);
		strdupcat(&dp->danames, daname);
		strdupcat(&dp->path, pnbuf);
		strdupcat(&dp->driver, drvbuf);
	    }
	}
	
	cam_close_device(cam);
	return 0;
    }

    if (sscanf(daname, "nvd%d", &id) == 1)
	sprintf(path+5, "nvme%d", id);
    
    
    /* Non-CAM */
    fd = open(path, O_RDONLY|O_DIRECT, 0);
    if (fd < 0)
	return -1;
    
    if (strncmp(daname, "nvd", 3) == 0) {
	nvme_identify(fd, daname, path+5, NULL);
	close(fd);
	return 0;
    }

    memset(idbuf, 0, sizeof(idbuf));
    if (ioctl(fd, DIOCGIDENT, idbuf) >= 0)
	ident = strndup(idbuf, sizeof(idbuf));
    
    if (!ident) {
	close(fd);
	return 1;
    }

    physbuf[0] = '\0';
    if (f_phys)
	(void) ioctl(fd, DIOCGPHYSPATH, physbuf);
    
    for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
	;
    dp = &dv[i];
    
    if (i >= dc) {
	dp->ident = ident;
	dp->danames = strdup(daname);
	dp->phys = strdup(physbuf);
	if (msize > 0)
	    dp->size = size2str(msize);
	++dc;
    } else {
	free(ident);
	strdupcat(&dp->danames, daname);
    }
    
    close(fd);
    return 0;
}


int
main(int argc,
     char *argv[]) {
    char *daname;
    char *buf, *bp;
    size_t bsize = 0;
    int i, j;
    int rc = 0;
    int identlen = 7;
    int vendorlen = 6;
    int productlen = 7;
    int revisionlen = 4;
    int danameslen = 5;
    int drvlen = 3;
    int pathlen = 4;
    int physlen = 4;
    int numlen = 1;
    int sizelen = 3;

    dv = calloc((ds = 1024), sizeof(DISK));
    if (!dv) {
	fprintf(stderr, "%s: Error: calloc: %s\n",
		argv[0], strerror(errno));
	exit(1);
    }
    
    for (i = 1; i < argc && argv[i][0] == '-'; i++) {
	for (j = 1; argv[i][j]; j++)
	    switch (argv[i][j]) {
	    case 'h':
		printf("Usage: %s [-v] [-p] [-S<sort>] [-W<maxwidth>] [<devices>]\n", argv[0]);
		exit(0);
	    case 'S':
		if (argv[i][j+1])
		    f_sort = strdup(argv[i]+j+1);
		else if (argv[i+1][0] && argv[i+1][0] != '-')
		    f_sort = strdup(argv[++i]);
		else {
		    fprintf(stderr, "%s: Error: Missing value for -S\n", argv[0]);
		    exit(1);
		}
		goto NextArg;
	    case 'W':
		if (argv[i][j+1] && sscanf(argv[i]+j+1, "%d", &f_maxwidth) != 1) {
		    if (argv[i+1][0] && argv[i+1][0] != '-' && sscanf(argv[i+1], "%d", &f_maxwidth) != 1) {
			fprintf(stderr, "%s: Error: Missing value for -W\n", argv[0]);
			exit(1);
		    }
		}
		goto NextArg;
	    case 'v':
		f_verbose++;
		if (f_verbose > 1)
		    f_maxwidth = 0;
		break;
	    case 'p':
		f_phys++;
		break;
	    case 'd':
		f_debug++;
		break;
	    default:
		fprintf(stderr, "%s: Error: -%c: Invalid switch\n",
			argv[0], argv[i][j]);
		exit(1);
	    }
    NextArg:;
    }

    if (i >= argc) {
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

	bp = buf;
	while ((daname = strsep(&bp, " ")) != NULL) {
	    rc = do_device(daname);
	    if (rc < 0) {
		fprintf(stderr, "%s: Error: %s: Unable to access: %s\n", argv[0], daname, strerror(errno));
		exit(1);
	    } else if (rc > 0) {
		fprintf(stderr, "%s: Error: %s: Skipped\n", argv[0], daname);
	    }
	}

	free(buf);
    } else {
	for (; i < argc; i++) {
	    rc = do_device(argv[i]);
	    if (rc < 0) {
		fprintf(stderr, "%s: Error: %s: Unable to access: %s\n",
			argv[0], argv[i], strerror(errno));
		exit(1);
	    } else if (rc > 0) {
		fprintf(stderr, "%s: Error: %s: Skipped\n",
			argv[0], argv[i]);
	    }
	}
    }
    
    if (!dc)
	return 0;
    
    for (i = 0; i < dc; i++) {
	strntrim(dv[i].ident, &identlen, f_maxwidth);
	strntrim(dv[i].vendor, &vendorlen, f_maxwidth);
	strntrim(dv[i].product, &productlen, f_maxwidth);
	strntrim(dv[i].revision, &revisionlen, f_maxwidth);
	strntrim(dv[i].danames, &danameslen, f_maxwidth);
	strntrim(dv[i].driver, &drvlen, f_maxwidth);
	strntrim(dv[i].path, &pathlen, f_maxwidth);
	strntrim(dv[i].phys, &physlen, f_maxwidth);
	strntrim(dv[i].size, &sizelen, f_maxwidth);
    }

    numlen = (int) (log10(dc)+1);
    qsort(&dv[0], dc, sizeof(dv[0]), dv_sort);

    if (isatty(1)) {
	printf("\033[1;4m%*s : %-*s : %-*s : %-*s : %-*s : %*s : %-*s",
	       numlen, "#",
	       vendorlen, "VENDOR",
	       productlen, "PRODUCT",
	       revisionlen, "REV.",
	       identlen, "IDENT",
	       sizelen, "SIZE",
	       danameslen, "NAMES");
	if (f_phys) {
	    printf(" : %-*s",
		   physlen, "PHYS");
	}
	if (f_verbose) {
	    printf(" : %-*s : %-*s",
		   drvlen, "DRV.",
		   pathlen, "PATH");
	}
	puts("\033[0m");
    }

    for (i = 0; i < dc; i++) {
	printf("%*u : %-*s : %-*s : %-*s : %-*s : %*s : %-*s",
	       numlen, i+1,
	       vendorlen, dv[i].vendor ? dv[i].vendor : "?",
	       productlen, dv[i].product ? dv[i].product : "?",
	       revisionlen, dv[i].revision ? dv[i].revision : "?",
	       identlen, dv[i].ident,
	       sizelen, dv[i].size ? dv[i].size : "?",
	       danameslen, dv[i].danames);
	if (f_phys) {
	    printf(" : %.*s",
		   physlen, dv[i].phys ? dv[i].phys : "");
	}
	if (f_verbose) {
	    printf(" : %-*s : ",
		   drvlen, dv[i].driver);
	    if (dv[i].path)
		p_strip(dv[i].path);
	    else
		putchar('-');
	}
	putchar('\n');
    }

    return rc;
}
