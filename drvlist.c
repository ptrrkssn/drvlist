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

int f_verbose = 0;
int f_debug = 0;
int f_temp = 0;
int f_smart = 0;
int f_tests = 0;
int f_hours = 0;
int f_proto = 1;
int f_xport = 1;
int f_driver = 0;
int f_path = 0;
int f_phys = 0;
int f_made = 0;
int f_endurance = 0;
int f_multipath = 0;
int f_maxwidth = 22;

char *f_sep = " : ";
char *f_sort = NULL;





int
get_health_status(DRIVE *dp) {
    int rc;

    
    if (!dp) {
	errno = EINVAL;
	return -1;
    }

    switch (dp->raw.proto) {
    case DRIVE_PROTO_SCSI:
	rc = drive_scsi_health_status(dp);
	break;
    case DRIVE_PROTO_SATL:
	rc = satl_health_status(dp);
	if (!rc)
	    rc = drive_scsi_health_status(dp);
	break;
    case DRIVE_PROTO_ATA:
	rc = ata_health_status(dp);
	break;
    case DRIVE_PROTO_NVME:
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

    if (!isatty(fileno(stderr)))
	return;
    
    putc(dials[pos], stderr);
    putc('\b', stderr);
    fflush(stderr);
    pos = (pos+1)%4;
}
    

int
do_device(char *dname) {
    int fd, i, rc = -1;
    char *ident = NULL;
    struct cam_device *cam;
    DRIVE *dp;
    char path[2048];
    char idbuf[DISK_IDENT_SIZE];
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
	if (f_debug > 1) {
	    fprintf(stderr, "*** CAM Device: path=%s serial=%*s dev=%s%u pass=%s%u sim=%s%u\n",
		    path,
		    cam->serial_num_len, cam->serial_num,
		    cam->given_dev_name,
		    cam->given_unit_number,
		    cam->device_name,
		    cam->dev_unit_num,
		    cam->sim_name,
		    cam->sim_unit_number);
	}

	if (f_verbose > 1)
	    sprintf(drvbuf, "%s%u @ bus %u",
		    cam->sim_name, cam->sim_unit_number, cam->bus_id);
	else
	    sprintf(drvbuf, "%s%u",
		    cam->sim_name, cam->sim_unit_number);

#if 0
	cam_path_string(cam, pnbuf, sizeof(pnbuf));
#else
	sprintf(pnbuf, "scbus %2u target %3u lun %2jx",
		cam->path_id,
		cam->target_id,
		cam->target_lun);
#endif
	
	/* Get device serial number */
	ident = cam->serial_num_len > 0 ? strdupget((char *) &cam->serial_num[0], cam->serial_num_len) : NULL;
	
	if (ident && f_multipath) {
	    /* Locate multipath alternate names */
	    for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
		;
	} else
	    i = dc;
	dp = &dv[i];
	
	if (i < dc) {
	    /* Already probed drive, located via alternative path */

	    if (f_debug)
		fprintf(stderr, "*** %s%u : %s%u: Alternate path found\n",
			cam->given_dev_name, cam->given_unit_number,
			cam->sim_name, cam->sim_unit_number);

	    if (ident)
		free(ident);

	    /* Add alternate names */
	    strdupcat(&dp->names, dname);
	    strdupcat(&dp->path, pnbuf);
	    strdupcat(&dp->driver, drvbuf);

	    cam_close_device(cam);
	    close(fd);
	} else {
	    /* New drive */

	    drive_init(dp);
	    
	    dp->cam = cam;
	    dp->ident = ident;
	    
	    dp->names = strdup(dname);
	    dp->path = strdup(pnbuf);
	    if (msize > 0)
		dp->size = size2str(msize);
	    dp->driver = strdup(drvbuf);

	    drive_classify(dp);

	    /* Just in case get_drive_type fails */
	    if (dp->raw.proto == DRIVE_PROTO_UNKNOWN) {
		if (strcmp(cam->given_dev_name, "nda") == 0)
		    dp->raw.proto = DRIVE_PROTO_NVME;
		else if (strcmp(cam->given_dev_name, "ada") == 0)
		    dp->raw.proto = DRIVE_PROTO_ATA;
		else if (strcmp(cam->given_dev_name, "da") == 0) {
		    dp->raw.proto = DRIVE_PROTO_SCSI;
		}
	    }
		    
	    if (dp->raw.proto == DRIVE_PROTO_NVME) {
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
		    nvme_identify_fd(nfd, &dp->vendor, &dp->product, &dp->revision, NULL, &dp->phys, &have_selftest);
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
		    if (ioctl(fd, DIOCGPHYSPATH, physbuf) < 0) {
#if 0
			fprintf(stderr, "%s: Error: %s (%d): ioctl(DIOCGPHYSPATH): %s\n",
				argv0, path, fd, strerror(errno));
#endif
		    }
		    if (physbuf[0]) 
			dp->phys = strdup(physbuf);
		}

		if (!dp->vendor || !dp->product || !dp->revision) {
		    if (dp->raw.proto == DRIVE_PROTO_ATA) {
			/* ATA device */
			ata_identify(dp, &dp->vendor, &dp->product, &dp->revision, &have_selftest);
			dp->flags.have_selftest = have_selftest;
		    }
		}

		switch (dp->raw.proto) {
		case DRIVE_PROTO_MMCSD:
		    fprintf(stderr, "[mmcsd]\n");
		    break;
		default:
		    /* Probably only da (SAS/SCSI) drives */
		    if (!dp->vendor && cam->inq_data.vendor[0])
			dp->vendor = strdupget(cam->inq_data.vendor, sizeof(cam->inq_data.vendor));
		    
		    if (!dp->product && cam->inq_data.product[0])
			dp->product = strdupget(cam->inq_data.product, sizeof(cam->inq_data.product));
		    
		    if (!dp->revision && cam->inq_data.revision[0])
			dp->revision = strdupget(cam->inq_data.revision, sizeof(cam->inq_data.revision));
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

	    dp->proto = strdup(drive_proto2str(dv[i].raw.proto));
	    dp->xport = strdup(drive_cam_xport2str(dv[i].raw.xport));
	    
	    rc = get_health_status(dp);
	    if (rc > 0) {
		char buf[256];
		
		if (dp->raw.temperature != DRIVE_NO_TEMPERATURE) {
		    sprintf(buf, "%u", dp->raw.temperature);
		    dp->temp = strdup(buf);
		}

		if (dp->raw.power_on_hours != DRIVE_NO_POWER_ON_HOURS) {
		    sprintf(buf, "%zu", dp->raw.power_on_hours);
		    dp->hours = strdup(buf);
		}

		if (dp->raw.endurance_used != DRIVE_NO_ENDURANCE_USED) {
		    sprintf(buf, "%d%%", dp->raw.endurance_used);
		    dp->used = strdup(buf);
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

	if (f_debug > 1)
	    fprintf(stderr, "*** %s: Non-CAM Device\n", path);

	memset(idbuf, 0, sizeof(idbuf));
	if (ioctl(fd, DIOCGIDENT, idbuf) < 0) {
	    fprintf(stderr, "%s: Error: %s: ioctl(DIOCGIDENT): %s",
		    argv0, path, strerror(errno));
	    exit(1);
	} else {
	    ident = strdupget(idbuf, sizeof(idbuf));
	    if (!ident) {
		fprintf(stderr, "%s: Error: %s: Memory allocation: %s\n",
			argv0, path, strerror(errno));
		exit(1);
	    }
	}
	
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
	    dp->names = strdup(dname);
	    dp->phys = strdup(physbuf);
	    if (msize > 0)
		dp->size = size2str(msize);
	    ++dc;
	} else {
	    free(ident);
	    strdupcat(&dp->names, dname);
	}

    }
    
    close(fd);
    return 0;
}

int
p_field(const char *value,
	int  width,
	const char *separator) {
    if (!value)
	value = "-";
    
    if (width < 0)
	return printf("%s%-*s", separator ? separator : "", -width, value);
    else
	return printf("%s%*s", separator ? separator : "", width, value);
}


int
main(int argc,
     char *argv[]) {
    char *name;
    char *buf, *bp;
    size_t bsize = 0;
    int i, j;
    int rc = 0;
    int identlen = 7;
    int vendorlen = 6;
    int productlen = 7;
    int revisionlen = 4;
    int nameslen = 5;
    int drvlen = 3;
    int pathlen = 4;
    int physlen = 4;
    int numlen = 1;
    int sizelen = 4;
    int templen = 4;
    int smartlen = 5;
    int testslen = 5;
    int hourslen = 5;
    int protolen = 5;
    int xportlen = 5;
    int usedlen = 4;
    int madelen = 4;


    argv0 = argv[0];

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
		printf("Usage: %s [-vpts] [-S<sort>] [-W<maxwidth>] [<devices>]\n", argv[0]);
		exit(0);
		
	    case 'S':
		if (argv[i][j+1]) {
		    f_sort = strdup(argv[i]+j+1);
		    goto NextArg;
		}

		if (argv[i+1] && argv[i+1][0] != '-') {
		    f_sort = strdup(argv[++i]);
		    goto NextArg;
		}

		fprintf(stderr, "%s: Error: Missing value for -S\n", argv[0]);
		exit(1);
		
	    case 'W':
		if (argv[i][j+1] && sscanf(argv[i]+j+1, "%d", &f_maxwidth) == 1)
		    goto NextArg;
		    
		if (argv[i+1] && argv[i+1][0] != '-' && sscanf(argv[i+1], "%d", &f_maxwidth) == 1)
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
		
	    case 'v':
		f_verbose++;
		if (f_verbose > 2)
		    f_maxwidth = 0;
		break;
		
	    case 'w':
		f_maxwidth = 0;
		break;
		
	    case 'm':
		f_multipath++;
		break;
		
	    case 'p':
		f_phys++;
		f_maxwidth = 0;
		break;
		
	    case 't':
		f_temp++;
		break;
		
	    case 's':
		f_smart++;
		f_tests++;
		break;
		
	    case 'd':
		f_driver++;
		break;
		
	    case 'e':
		f_endurance++;
		break;
		
	    default:
		fprintf(stderr, "%s: Error: -%c: Invalid switch\n",
			argv[0], argv[i][j]);
		exit(1);
	    }
    NextArg:;
    }

    if (f_verbose > 0) {
	f_temp++;
	if (f_verbose > 1) {
	    f_smart++;
	    f_tests++;
	    if (f_verbose > 2) {
		f_hours++;
		if (f_verbose > 3) {
		    f_endurance++;
		    if (f_verbose > 4) {
			f_driver++;
			if (f_verbose > 5) {
			    f_path++;
			    if (f_verbose > 6) {
				f_phys++;
				if (f_verbose > 7) {
				    f_made++;
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
    
    for (i = 0; i < dc; i++) {
	strntrim(dv[i].ident, &identlen, f_maxwidth);
	strntrim(dv[i].vendor, &vendorlen, f_maxwidth);
	strntrim(dv[i].product, &productlen, f_maxwidth);
	strntrim(dv[i].revision, &revisionlen, f_maxwidth);
	strntrim(dv[i].names, &nameslen, f_maxwidth);
	strntrim(dv[i].driver, &drvlen, f_maxwidth);
	strntrim(dv[i].path, &pathlen, f_maxwidth);
	strntrim(dv[i].phys, &physlen, f_maxwidth);
	strntrim(dv[i].size, &sizelen, f_maxwidth);
	strntrim(dv[i].proto, &protolen, f_maxwidth);
	strntrim(dv[i].xport, &xportlen, f_maxwidth);
	strntrim(dv[i].temp, &templen, f_maxwidth);
	strntrim(dv[i].smart, &smartlen, f_maxwidth);
	strntrim(dv[i].tests, &testslen, f_maxwidth);
	strntrim(dv[i].hours, &hourslen, f_maxwidth);
	strntrim(dv[i].used, &usedlen, f_maxwidth);
	strntrim(dv[i].made, &madelen, f_maxwidth);
    }

    numlen = (int) (log10(dc)+1);
    qsort(&dv[0], dc, sizeof(dv[0]), drive_sort);

    if (isatty(1)) {
	p_field("#", numlen, "\033[1;4m");
	p_field("VENDOR", -vendorlen, f_sep);
	p_field("PRODUCT", -productlen, f_sep);
	p_field("REV.", -revisionlen, f_sep);
	p_field("IDENT", -identlen, f_sep);
	p_field("SIZE", sizelen, f_sep);
	
	if (f_temp)
	    p_field("TEMP", templen, f_sep);

	if (f_smart)
	    p_field("SMART", -smartlen, f_sep);

	if (f_tests)
	    p_field("TESTS", -testslen, f_sep);

	if (f_hours)
	    p_field("HOURS", hourslen, f_sep);

	if (f_endurance)
	    p_field("USED", usedlen, f_sep);

	if (f_proto)
	    p_field("PROTO", -protolen, f_sep);
	
	if (f_xport)
	    p_field("XPORT", -xportlen, f_sep);
	
	if (f_made)
	    p_field("MADE", -madelen, f_sep);
	
	p_field("NAMES", -nameslen, f_sep);
	
	if (f_driver)
	    p_field("DRV.", -drvlen, f_sep);

	if (f_path)
	    p_field("PATH", -pathlen, f_sep);
	
	if (f_phys)
	    p_field("PHYS", -physlen, f_sep);

	puts("\033[0m");
    }

    for (i = 0; i < dc; i++) {
	printf("%*u", numlen, i+1);
	p_field(dv[i].vendor, -vendorlen, f_sep);
	p_field(dv[i].product, -productlen, f_sep);
	p_field(dv[i].revision, -revisionlen, f_sep);
	p_field(dv[i].ident, -identlen, f_sep);
	p_field(dv[i].size, sizelen, f_sep);

	if (f_temp)
	    p_field(dv[i].temp, templen, f_sep);

	if (f_smart)
	    p_field(dv[i].smart, -smartlen, f_sep);
		   
	if (f_tests)
	    p_field(dv[i].tests, -testslen, f_sep);

	if (f_hours)
	    p_field(dv[i].hours, hourslen, f_sep);

	if (f_endurance)
	    p_field(dv[i].used, usedlen, f_sep);

	if (f_proto)
	    p_field(dv[i].proto, -protolen, f_sep);
	
	if (f_xport)
	    p_field(dv[i].xport, -xportlen, f_sep);
		
	if (f_made)
	    p_field(dv[i].made, -madelen, f_sep);
		
	p_field(dv[i].names, -nameslen, f_sep);
	       
	if (f_driver)
	    p_field(dv[i].driver, -drvlen, f_sep);

	if (f_path)
	    p_field(dv[i].path, -pathlen, f_sep);
	
	if (f_phys)
	    p_field(dv[i].phys, -physlen, f_sep);

	putchar('\n');
    }

    return rc;
}
