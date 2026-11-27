/* 
 * drvlist.c
 *
 * A small FreeBSD utility to enumerate installed drives in a system.
 *
 * Copyright (c) 2024-2026 Peter Eriksson <pen@lysator.liu.se>
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
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/sysctl.h>
#include <sys/disk.h>
#include <sys/stat.h>

#include <camlib.h>
#include <cam/cam_ccb.h>
#include <cam/scsi/scsi_message.h>
#include <cam/ata/ata_all.h>
#include <cam/mmc/mmc_all.h>
#include <dev/nvme/nvme.h>

#include "scsi.h"
#include "ata.h"
#include "nvme.h"
#include "satl.h"

#include "utils.h"
#include "drive.h"


char *argv0 = "drvlist";
char *version = "2.0";

int f_verbose = 0;
int f_debug = 0;
int f_sort = 0;
int f_dir  = 1;
int f_multipath = 0;
int f_colwidth = 0;
int f_width = 0;
int f_yaml = 0;

int t_width  = 0;
int t_height = 0;


char *f_sep = " : ";


int f_vendor = 1;
int f_product = 1;
int f_version = 1;
int f_ident = 1;
int f_size = 1;
int f_sector = 0;
int f_rpm = 0;
int f_temp = 0;
int f_smart = 0;
int f_tests = 0;
int f_hours = 0;
int f_wear = 0;
int f_made = 0;
int f_class = 0;
int f_proto = 0;
int f_xport = 0;
int f_name = 1;
int f_ctrl = 0;
int f_path = 0;
int f_phys = 0;
int f_form = 0;






int
get_health_status(DRIVE *dp) {
    int rc;

    
    if (!dp) {
	errno = EINVAL;
	return -1;
    }

    switch (dp->raw.class) {
    case DRIVE_CLASS_SCSI:
	rc = drive_scsi_health_status(dp);
	break;
    case DRIVE_CLASS_SATL:
	rc = satl_health_status(dp);
	if (!rc)
	    rc = drive_scsi_health_status(dp);
	break;
    case DRIVE_CLASS_ATA:
	rc = drive_ata_health_status(dp);
	break;
    case DRIVE_CLASS_NVME:
	rc = nvme_health_status(dp);
	break;
    default:
	errno = ENOENT;
	rc = -1;
    }

    return rc;
}



static void
spin(void) {
    static char dials[] = "|/-\\";
    static int pos = 0;
#if 0
    static time_t now, last = 0;
#endif
    if (!isatty(fileno(stderr)))
	return;
#if 0
    time(&now);
    if (!last)
	last = now;
    if (last == now)
	return;
#endif
    putc(dials[pos], stderr);
    putc('\b', stderr);
    fflush(stderr);
    pos = (pos+1)%4;
#if 0
    last = now;
#endif
}


struct drive_column {
    char *name;
    int width;
    off_t offset;
    int *flag;
} columns[] = {
#define C_NAME 1
    { "NAME",     -5, offsetof(DRIVE, name),    &f_name },    
#define C_VENDOR 2
    { "VENDOR",   -6, offsetof(DRIVE, vendor),  &f_vendor },
#define C_PRODUCT 3
    { "PRODUCT",  -7, offsetof(DRIVE, product), &f_product },
#define C_VERSION 4
    { "VERSION",  -7, offsetof(DRIVE, version), &f_version },    
#define C_IDENT 5
    { "IDENT",    -5, offsetof(DRIVE, ident),   &f_ident },    
#define C_SIZE 6
    { "SIZE",      4, offsetof(DRIVE, size),    &f_size },    
#define C_SECTOR 7
    { "SECTOR",    6, offsetof(DRIVE, sector),  &f_sector },    
#define C_TEMP 8
    { "TEMP",      4, offsetof(DRIVE, temp),    &f_temp },    
#define C_SMART 9
    { "SMART",    -5, offsetof(DRIVE, smart),   &f_smart },    
#define C_TESTS 10
    { "TESTS",    -5, offsetof(DRIVE, tests),   &f_tests },    
#define C_HOURS 11
    { "HOURS",     5, offsetof(DRIVE, hours),   &f_hours },    
#define C_WEAR 12
    { "WEAR",      4, offsetof(DRIVE, wear),    &f_wear },    
#define C_MADE 13
    { "MADE",     -4, offsetof(DRIVE, made),    &f_made },    
#define C_CLASS 14
    { "CLASS",    -5, offsetof(DRIVE, class),   &f_class },    
#define C_PROTO 15
    { "PROTO",    -5, offsetof(DRIVE, proto),   &f_proto },    
#define C_XPORT 16
    { "XPORT",    -5, offsetof(DRIVE, xport),   &f_xport },    
#define C_RPM 17
    { "RPM",      -3, offsetof(DRIVE, rpm),     &f_rpm },    
#define C_FORM 18
    { "FORM",     -4, offsetof(DRIVE, form),    &f_form },    
#define C_CTRL 19
    { "CTRL",     -4, offsetof(DRIVE, ctrl),    &f_ctrl },    
#define C_PATH 20
    { "PATH",     -4, offsetof(DRIVE, path),    &f_path },    
#define C_PHYS 21
    { "PHYS",     -4, offsetof(DRIVE, phys),    &f_phys },    
};

	
#define NCOLUMNS (sizeof(columns)/sizeof(columns[0]))



int
terminal_size(int *x,
	      int *y) {
    struct winsize win = { 0, 0, 0, 0 };

    if (ioctl(1, TIOCGWINSZ, &win) < 0)
	return -1;

    if (x)
	*x = win.ws_col;
    if (y)
	*y = win.ws_row;

    return 0;
}


int
do_device(char *dname) {
    int fd, i, rc = -1;
    char *ident = NULL;
    struct cam_device *cam;
    DRIVE *dp;
    char path[2048];
    char idbuf[DISK_IDENT_SIZE+1];
    char pnbuf[MAXPATHLEN];
    char drvbuf[MAXPATHLEN];
    char physbuf[MAXPATHLEN];
    off_t msize = 0;
    int have_selftest = 0;
    

    if (f_debug)
	fprintf(stderr, "[%s]\n", dname);

    spin();
    
    /* Increase size of drive table */
    if (dc >= ds) {
	dv = realloc(dv, (ds += 1024)*sizeof(DRIVE));
	if (!dv) {
	    fprintf(stderr, "%s: Error: Memory allocation failure: %s\n",
		    argv0, strerror(errno));
	    exit(1);
	}
    }

    
    /* Get device name and path */
    if (strncmp(dname, "/dev/", 5) == 0) {
	strcpy(path, dname);
	dname = path+5;
    } else {
	strcpy(path, "/dev/");
	strcpy(path+5, dname);
    }


	
    /* Open a descriptor we can do ioctl()'s on */
    fd = open(path, O_RDONLY);
    if (fd < 0) {
	fprintf(stderr, "%s: Error: %s (%s): open(O_RDONLY): %s\n",
		argv0, path, path, strerror(errno));
	exit(1);
    }

    /* Get drive size */
    if (ioctl(fd, DIOCGMEDIASIZE, &msize) < 0) {
#if 0
	/* Ignore errors */
	fprintf(stderr, "%s: Error: %s: ioctl(DIOCGMEDIASIZE): %s\n",
		argv0, path, strerror(errno));
#endif
    }

    
    cam = cam_open_device(path, O_RDWR);
    if (cam) {
	sprintf(drvbuf, "%s%u:%u",
		cam->sim_name, cam->sim_unit_number, cam->bus_id);

	sprintf(pnbuf, "scbus %2u target %3u lun %2jx",
		cam->path_id,
		cam->target_id,
		cam->target_lun);

	/* Get device serial number */
	if (cam->serial_num_len > 0) {
	    ident = strdupget((char *) &cam->serial_num[0], cam->serial_num_len);
	}

	if (ident && f_multipath) {
	    /* Locate multipath alternate names */
	    for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
		;
	} else
	    i = dc;
	dp = &dv[i];

	if (f_debug) {
	    fprintf(stderr, "  [cam]\n");
	    if (cam->device_path[0])
		fprintf(stderr, "\tCAM path      = %s\n", cam->device_path);
	    fprintf(stderr, "\tCAM devname   = %s%u\n", cam->given_dev_name, cam->given_unit_number);
	    fprintf(stderr, "\tCAM passname  = %s%u\n", cam->device_name, cam->dev_unit_num);
	    fprintf(stderr, "\tCAM sim       = %s%u\n", cam->sim_name, cam->sim_unit_number);
	    fprintf(stderr, "\tCAM serial    = %.*s\n", cam->serial_num_len, cam->serial_num);
	    fprintf(stderr, "\tCAM target    = %u lun %lu\n", cam->target_id, cam->target_lun);
	    fprintf(stderr, "\tCAM path_id   = %u\n", cam->path_id);
	    fprintf(stderr, "\tCAM bus_id    = %u\n", cam->bus_id);
	    drive_scsi_parse_inquiry(dp, (uint8_t *) &cam->inq_data, sizeof(cam->inq_data));
	}
	
	if (i < dc) {
	    /* Already probed drive, located via alternative path */

	    if (f_debug)
		fprintf(stderr, "*** %s%u : %s%u: Alternate path found\n",
			cam->given_dev_name, cam->given_unit_number,
			cam->sim_name, cam->sim_unit_number);

	    if (ident)
		free(ident);

	    /* Add alternate names */
	    strdupcat(&dp->name, dname);
	    strdupcat(&dp->path, pnbuf);
	    strdupcat(&dp->ctrl, drvbuf);

	    cam_close_device(cam);
	    close(fd);
	} else {
	    /* New drive */

	    drive_init(dp);
	    
	    dp->cam = cam;
	    dp->ident = ident;
	    
	    dp->name = strdup(dname);
	    dp->path = strdup(pnbuf);
	    if (msize > 0)
		dp->size = size2str(msize);
	    dp->ctrl = strdup(drvbuf);

	    drive_classify(dp);

	    /* Just in case get_drive_type fails */
	    if (dp->raw.class == DRIVE_CLASS_UNKNOWN) {
		if (strcmp(cam->given_dev_name, "nda") == 0)
		    dp->raw.class = DRIVE_CLASS_NVME;
		else if (strcmp(cam->given_dev_name, "ada") == 0)
		    dp->raw.class = DRIVE_CLASS_ATA;
		else if (strcmp(cam->given_dev_name, "da") == 0) {
		    dp->raw.class = DRIVE_CLASS_SCSI;
		}
	    }
		    
	    if (dp->raw.class == DRIVE_CLASS_NVME) {
		/* NVME device */
		char npath[1024];

		/* Check to make sure it is a local NVME controller */
		if (strcmp(cam->sim_name, "nvme") == 0) {
		    int nfd;
		    
		    sprintf(npath, "/dev/%s%u", cam->sim_name, cam->sim_unit_number);
		    
		    nfd = open(npath, O_RDONLY);
		    if (nfd < 0) {
			fprintf(stderr, "%s: Error: %s: open(O_RDONLY): %sn",
				argv0, npath, strerror(errno));
			close(fd);
			return -1;
		    }
		    nvme_identify_fd(nfd, &dp->vendor, &dp->product, &dp->version, NULL, &dp->phys, &dp->proto, &have_selftest);
		    dp->flags.have_selftest = have_selftest;
		    close(nfd);
		    
		} else {
		    if (f_debug) 
			fprintf(stderr, "*** %s%u -> %s%u: Non-local NVME device\n",
				cam->given_dev_name, cam->given_unit_number, 
				cam->sim_name, cam->sim_unit_number);
		}
		
	    } else {
		/* DA (SAS/SCSI) or ADA (SATA) drive */
		char *cp;

		
		/* Get physical path (does not work on NDA/NVME */
		physbuf[0] = '\0';
		if (f_phys) {
		    
		    memset(physbuf, 0, sizeof(physbuf));
		    if (ioctl(fd, DIOCGPHYSPATH, physbuf) < 0) {
#if 0
			fprintf(stderr, "%s: Error: %s (%d): ioctl(DIOCGPHYSPATH): %s\n",
				argv0, path, fd, strerror(errno));
#endif
		    }
		    if (physbuf[0]) {
			dp->phys = strdupget(physbuf, sizeof(physbuf));
		    }
		}

		if (!dp->vendor || !dp->product || !dp->version) {
		    if (dp->raw.class == DRIVE_CLASS_ATA) {
			drive_ata_identify(dp);
		    }
		}

		switch (dp->raw.class) {
		case DRIVE_CLASS_MMCSD:
		    fprintf(stderr, "[mmcsd]\n");
		    break;
		default:
		    /* Probably only da (SAS/SCSI) drives */
		    if (!dp->proto)
			dp->proto = strdup(scsi_inquiry_version2str(SID_ANSI_REV(&cam->inq_data)));
		    
		    if (!dp->vendor && cam->inq_data.vendor[0])
			dp->vendor = strdupget(cam->inq_data.vendor, sizeof(cam->inq_data.vendor));
		    
		    if (!dp->product && cam->inq_data.product[0])
			dp->product = strdupget(cam->inq_data.product, sizeof(cam->inq_data.product));
		    
		    if (!dp->version && cam->inq_data.revision[0])
			dp->version = strdupget(cam->inq_data.revision, sizeof(cam->inq_data.revision));
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
		
	    }

	    dp->class = strdup(drive_class2str(dv[i].raw.class));
	    dp->xport = strdup(drive_cam_xport2str(dv[i].raw.xport));
	    
	    rc = get_health_status(dp);
	    if (rc > 0) {
		char buf[256];
		
		if (dp->raw.temperature != DRIVE_NO_TEMPERATURE) {
		    sprintf(buf, "%u", dp->raw.temperature);
		    dp->temp = strdup(buf);
		}

		if (dp->raw.power_on_hours != DRIVE_NO_HOURS) {
		    sprintf(buf, "%zu", dp->raw.power_on_hours);
		    dp->hours = strdup(buf);
		}

		if (dp->raw.endurance_used != DRIVE_NO_ENDURANCE_USED) {
		    sprintf(buf, "%d%%", dp->raw.endurance_used);
		    dp->wear = strdup(buf);
		} 
		
		dp->smart = strdup(drive_smart_status2str(dp->raw.smart_status));

		if (f_tests > 1 &&
		    !(dp->raw.selftest.status == DRIVE_SELFTEST_UNKNOWN ||
		      dp->raw.selftest.status == DRIVE_SELFTEST_NONE)) {
		    char buf[2048];
		    size_t len;

		    
		    len = snprintf(buf, sizeof(buf), "%s - %u/%u OK",
				   drive_selftest_status2str(dp->raw.selftest.status),
				   dp->raw.selftest.passed,
				   dp->raw.selftest.tests);
		    if (dp->raw.selftest.aborted)
			len += snprintf(buf+len, sizeof(buf)-len, ", %u Aborted", dp->raw.selftest.aborted);
		    if (dp->raw.selftest.running)
			len += snprintf(buf+len, sizeof(buf)-len, ", %u Running", dp->raw.selftest.running);
		    
		    dp->tests = strdup(buf);
		} else
		    dp->tests = strdup(drive_selftest_status2str(dp->raw.selftest.status));
	    }
	    dc++;
	} 
	
	cam_close_device(cam);
	close(fd);
	return 0;
    } else {
	/* Non-CAM device */
	int res;
	if (f_debug > 1)
	    fprintf(stderr, "*** %s: Non-CAM Device\n", path);

	memset(idbuf, 0, sizeof(idbuf));
	res = ioctl(fd, DIOCGIDENT, idbuf);
	if (res < 0) {
	    fprintf(stderr, "%s: Error: %s: ioctl(DIOCGIDENT): %s\n",
		    argv0, path, strerror(errno));
	    exit(1);
	}

	ident = strdupget(idbuf, sizeof(idbuf));
	if (!ident) {
	    fprintf(stderr, "%s: Error: %s: Memory allocation: %s\n",
		    argv0, path, strerror(errno));
	    exit(1);
	}
	fprintf(stderr, "*** IDENT FROM DIOCGIDENT: %s\n", ident);
	
	if (f_multipath) {
	    for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
		;
	} else
	    i = dc;
	dp = &dv[i];
	
	physbuf[0] = '\0';
	if (f_phys) {
	    if (ioctl(fd, DIOCGPHYSPATH, physbuf) < 0) {
#if 0
		fprintf(stderr, "%s: Error: %s (%d): ioctl(DIOCGPHYSPATH): %s\n",
			argv0, path, fd, strerror(errno));
#endif
	    }
	}
	
	if (i >= dc) {
	    /* New device */
	    dp->ident = ident;
	    dp->name = strdup(dname);
	    dp->phys = strdup(physbuf);
	    if (msize > 0)
		dp->size = size2str(msize);
	    ++dc;
	} else {
	    free(ident);
	    strdupcat(&dp->name, dname);
	}

    }
    
    close(fd);
    return 0;
}


int
p_column(FILE *fp,
	 const char *key,
	 const char *value,
	 int width,
	 const char *separator) {
    if (!value)
	value = "-";

    if (f_yaml) {
	return fprintf(fp, " %s: \"%s\"\n", key, value);
    } else {
	if (width < 0)
	    return fprintf(fp, "%s%-*s", separator ? separator : "", -width, value);
	else
	    return fprintf(fp, "%s%*s", separator ? separator : "", width, value);
    }
}


int
column_name2id(const char *name) {
    int i;

    for (i = 0; i < NCOLUMNS && strcasecmp(name, columns[i].name) != 0; i++)
	;

    if (i < NCOLUMNS)
	return i+1;

    return 0;
}

const char *
column_id2name(int id) {
    if (id < 1 || id > NCOLUMNS)
	return NULL;
    
    return columns[id-1].name;
}


int
column_get_flag(int id,
	       int *state) {
    int *ip;
    
      if (id < 1 || id > NCOLUMNS)
	return -1;

    ip = columns[id-1].flag;
    if (state)
	*state = *ip;
    
    return *ip;
}

int
column_set_flag(int id,
	      int state) {
    int *ip;
    
    if (id < 1 || id > NCOLUMNS)
	return -1;

    ip = columns[id-1].flag;
    *ip = state;
    return *ip;
}

int
column_enable_flag(int id) {
    int f;

    if (column_get_flag(id, &f) < 0)
	return -1;

    if (!f)
	return column_set_flag(id, 1);

    return f;
}


int
column_str2flags(char *str) {
    int id;
    char *cp;
    int f = 1;
	

    if (*str != '+' && *str != '-') {
	int i;

	for (i = 1; i <= NCOLUMNS; i++)
	    column_set_flag(i, 0);
    }
    
    cp = strtok(str, ",");
    while (cp) {
	if (*cp == '-') {
	    f = 0;
	    ++cp;
	} else if (*cp == '+') {
	    f = 1;
	    ++cp;
	}
	
	char *ep = strrchr(cp, '=');
	if (ep) {
	    *ep++ = '\0';
	    if (sscanf(ep, "%d", &f) < 1)
		return -1;
	}

	if (strcasecmp(cp, "ALL") == 0) {
	    int i;
	    
	    for (i = 1; i <= NCOLUMNS; i++)
		column_set_flag(i, f);
	} else {
	    id = column_name2id(cp);
	    if (!id)
		return -1;
	    column_set_flag(id, f);
	}
	
	cp = strtok(NULL, ",");
    }

    return 0;
}

static int safe_strcmp(const char *a,
		       const char *b) {
    if (!a && !b)
	return 0;

    if (!a && b)
	return -1;

    if (a && !b)
	return 1;

    return strcmp(a, b);
}

static int
drive_sort_devpath(const DRIVE *da,
		   const DRIVE *db) {
    int d;

    d = safe_strcmp(da->ctrl, db->ctrl) * f_dir;
    if (d)
	return d;
    
    return safe_strcmp(da->path, db->path) * f_dir;
}

static int
drive_sort(const void *a,
	   const void *b) {
    if (f_sort) {
	off_t offset = columns[f_sort-1].offset;
	DRIVE *ad = (DRIVE *) a;
	DRIVE *bd = (DRIVE *) b;
	
	char *as = * (char **) ((uint8_t *) ad + offset);
	char *bs = * (char **) ((uint8_t *) bd + offset);

	return safe_strcmp(as, bs) * f_dir;
    }
    
    return drive_sort_devpath((const DRIVE *) a, (const DRIVE *) b);
}


void
usage(FILE *fp) {
    int j;

    
    fprintf(fp, "Usage:\n");
    fprintf(fp, "    %s [<options>] [<dev-1> [.. <dev-N>]\n", argv0);

    fprintf(fp, "\nOptions:\n");
    fprintf(fp, "    -h              Display usage information\n");
    fprintf(fp, "    -S <column>     Sort using column\n");
    fprintf(fp, "    -C <columns>    Columns to display\n");
    fprintf(fp, "    -W <width>      Force terminal width\n");
    fprintf(fp, "    -D <level>      Enable internal debugging\n");
    fprintf(fp, "    -F <separator>  Columns separator [%s]\n", f_sep);
    fprintf(fp, "    -r              Reverse sort order\n");
    fprintf(fp, "    -v              Enable (increase) verbose output\n");
    fprintf(fp, "    -m              Enable multipath support\n");

    fprintf(fp, "\nNotes:\n");
    fprintf(fp, "   Columns for '-C' is a comma separated list optionally\n");
    fprintf(fp, "   prefixed with '+' or '-' in order to add or remove columns\n");
    
    fprintf(fp, "\nColumns:");
    for (j = 0; j < NCOLUMNS; j++) {
	if ((j & 7) == 0)
	    fprintf(fp, "\n   ");
	fprintf(fp, " %s", columns[j].name);
    }
    putc('\n', fp);
    
    fprintf(fp, "\nVersion:\n");
    fprintf(fp, "    %s (Built: %s %s)\n", version, __DATE__, __TIME__);
    
    fprintf(fp, "\nCopyright:\n");
    fprintf(fp, "    2026 Peter Eriksson <pen@lysator.liu.se>\n");
}

int
main(int argc,
     char *argv[]) {
    char *name;
    char *buf, *bp;
    size_t bsize = 0;
    int i, j;
    int rc = 0;
    int numlen = 1;
    int c_width = 0;
    int seplen = 3;
    
    
    argv0 = argv[0];
    terminal_size(&t_width, &t_height);

    dv = calloc((ds = 1024), sizeof(DRIVE));
    if (!dv) {
	fprintf(stderr, "%s: Error: calloc: %s\n",
		argv[0], strerror(errno));
	exit(1);
    }
    
    for (i = 1; i < argc && argv[i][0] == '-'; i++) {
	for (j = 1; argv[i][j]; j++)
	    switch (argv[i][j]) {
	    case 'h':
		usage(stdout);
		exit(0);
		
	    case 'S':
		if (argv[i][j+1]) {
		    f_sort = column_name2id(argv[i]+j+1);
		    if (f_sort < 0) {
			fprintf(stderr, "%s: Error: %s: Invalid column name for '-S'\n",
				argv[0], argv[i]+j+1);
			exit(1);
		    }
		    column_enable_flag(f_sort);
		    goto NextArg;
		}

		if (argv[i+1] && argv[i+1][0] != '-') {
		    f_sort = column_name2id(argv[++i]);
		    if (f_sort < 0) {
			fprintf(stderr, "%s: Error: %s: Invalid column name for '-S'\n",
				argv[0], argv[i]);
			exit(1);
		    }
		    column_enable_flag(f_sort);
		    goto NextArg;
		}

		fprintf(stderr, "%s: Error: Missing value for -S\n", argv[0]);
		exit(1);
		
	    case 'C':
		if (argv[i][j+1]) {
		    if (column_str2flags(argv[i]+j+1) < 0) {
			fprintf(stderr, "%s: Error: %s: Invalid column name(s) for '-C'\n",
				argv[0], argv[i]+j+1);
			exit(1);
		    }
		    goto NextArg;
		}

		if (argv[i+1] && argv[i+1][0] != '-') {
		    if (column_str2flags(argv[++i]) < 0) {
			fprintf(stderr, "%s: Error: %s: Invalid column name(s) for '-C'\n",
				argv[0], argv[i]);
			exit(1);
		    }
		    goto NextArg;
		}

		fprintf(stderr, "%s: Error: Missing value for -S\n", argv[0]);
		exit(1);
		
	    case 'W':
		if (argv[i][j+1] && sscanf(argv[i]+j+1, "%d", &f_width) == 1)
		    goto NextArg;
		    
		if (argv[i+1] && argv[i+1][0] != '-' && sscanf(argv[i+1], "%d", &f_width) == 1)
		    goto NextArg;
		
		fprintf(stderr, "%s: Error: Missing value for -W\n", argv[0]);
		exit(1);
		
	    case 'D':
		if (argv[i][j+1] && sscanf(argv[i]+j+1, "%d", &f_debug) == 1)
		    goto NextArg;
		
		if (argv[i+1] && argv[i+1][0] != '-' && sscanf(argv[++i], "%d", &f_debug) == 1)
		    goto NextArg;
		
		f_debug++;
		break;
		
	    case 'F':
		if (argv[i][j+1]) {
		    f_sep = strdup(argv[i]+j+1);
		    goto NextArg;
		}
		
		if (argv[i+1] && argv[i+1][0] != '-') {
		    f_sep = strdup(argv[++i]);
		    goto NextArg;
		}
		
		f_sep = " : ";
		break;

	    case 'r':
		f_dir = -1;
		break;
		
	    case 'v':
		f_verbose++;
		break;

	    case 'y':
		f_yaml++;
		break;

	    case 'm':
		f_multipath++;
		break;
		
	    default:
		fprintf(stderr, "%s: Error: -%c: Invalid switch\n",
			argv[0], argv[i][j]);
		exit(1);
	    }
    NextArg:;
    }

    /* Auto mode, must check all data sources to get the total columns widths */
    if (f_verbose == 1) {
	for (j = 0; j < NCOLUMNS; j++) {
	    column_set_flag(j+1, 1);
	}
    } else if (f_verbose > 1) {
	f_temp++;
	if (f_verbose > 2) {
	    f_smart++;
	    f_tests++;
	    if (f_verbose > 3) {
		f_hours++;
		if (f_verbose > 4) {
		    f_wear++;
		    f_made++;
		    if (f_verbose > 5) {
			f_ctrl++;
			if (f_verbose > 6) {
			    f_class++;
			    f_proto++;
			    f_xport++;
			    if (f_verbose > 7) {
				f_path++;
				if (f_verbose > 8) {
				    f_phys++;
				}
			    }
			}
		    }
		}
	    }
	}
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
	while ((name = strsep(&bp, " ")) != NULL) {
	    rc = do_device(name);
	    if (rc < 0) {
		fprintf(stderr, "%s: Error: %s: Unable to access: %s\n", argv[0], name, strerror(errno));
		exit(1);
	    } else if (rc > 0) {
		fprintf(stderr, "%s: Error: %s: Skipped\n", argv[0], name);
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

    seplen = strlen(f_sep);
    for (i = 0; i < dc; i++) {
	for (j = 0; j < NCOLUMNS; j++) {
	    struct drive_column *dfp = &columns[j];
	    int w;
	    char *cp;

	    cp = * (char **) ((uint8_t *) &dv[i] + dfp->offset);

	    w = (dfp->width >= 0 ? dfp->width : -dfp->width);
	    strntrim(cp, &w, f_colwidth);
	    dfp->width = (dfp->width >= 0 ? w : -w);
	}
    }

    numlen = (int) (log10(dc)+1);
    c_width = numlen;

    int x_width = (f_width > 0 && f_width < t_width) ? f_width : t_width;

    /* Automatically select columns to show based on terminal width */
    if (f_verbose == 1 && x_width > 0) {
	for (j = 0; j < NCOLUMNS; j++) {
	    int w = columns[j].width;

	    c_width += (w >= 0 ? w : -w)+seplen;
	    if (c_width > x_width)
		column_set_flag(j+1, 0);
	}
    }
    
    qsort(&dv[0], dc, sizeof(dv[0]), drive_sort);

    if (f_yaml)
	printf("---\n");
    else if (isatty(1)) {
	p_column(stdout, NULL, "#", numlen, "\033[1;4m");
	for (j = 0; j < NCOLUMNS; j++) {
	    struct drive_column *dfp = &columns[j];
	    
	    if (!*columns[j].flag)
		continue;
	    p_column(stdout, NULL, dfp->name, dfp->width, f_sep);
	}
	puts("\033[0m");
    }

    for (i = 0; i < dc; i++) {
	if (!f_yaml)
	    printf("%*u", numlen, i+1);
	
	for (j = 0; j < NCOLUMNS; j++) {
	    struct drive_column *dfp = &columns[j];
	    char *cp;
	    
	    if (!*columns[j].flag)
		continue;
	    
	    cp = * (char **) ((uint8_t *) &dv[i] + dfp->offset);
	    if (f_yaml)
		putc((j ? ' ' : '-'), stdout);
	    p_column(stdout, dfp->name, cp, dfp->width, f_sep);
	}
	putchar('\n');
    }

    return rc;
}
