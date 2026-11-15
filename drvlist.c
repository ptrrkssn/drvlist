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
#include <cam/scsi/scsi_message.h>
#include <cam/ata/ata_all.h>
#include <cam/mmc/mmc_all.h>
#include <dev/nvme/nvme.h>

#include "scsi.h"
#include "ata.h"
#include "nvme.h"

#include "utils.h"

#include "smart.h"
#include "tests.h"


char *argv0 = "drvlist";

int f_verbose = 0;
int f_debug = 0;
int f_phys = 0;
int f_temp = 0;
int f_smart = 0;
int f_tests = 0;
int f_poweron = 0;
int f_driver = 0;
int f_multipath = 0;
int f_maxwidth = 22;

char *f_sep = " : ";
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
    char *temp;
    char *smart;
    char *tests;
    char *poweron;
} DRIVE;

#define MAXDRIVE 2048

int dc = 0;
int ds = 0;
DRIVE *dv;


static int
dv_sort_ident(const DRIVE *da, const DRIVE *db) {
    return strcmp(da->ident, db->ident);
}

static int
dv_sort_devpath(const DRIVE *da, const DRIVE *db) {
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
	    return dv_sort_ident((const DRIVE *) a, (const DRIVE *) b);
    }
    
    return dv_sort_devpath((const DRIVE *) a, (const DRIVE *) b);
}




int
get_health_status(struct cam_device *dev,
		  struct health_result *result,
		  int have_selftest) {
    int rc;

    
    if (!dev || !result) {
	errno = EINVAL;
	return -1;
    }

    if (strcmp(dev->given_dev_name, "da") == 0)
	rc = scsi_health_status(dev, result, have_selftest);
    else if (strcmp(dev->given_dev_name, "ada") == 0)
	rc = ata_health_status(dev, result, have_selftest);
    else if (strcmp(dev->given_dev_name, "nda") == 0)
	rc = nvme_health_status(dev, result, have_selftest);
    else {
	errno = ENOENT;
	rc = -1;
    }

    return rc;
}

int
get_drive_temperature(struct cam_device *dev,
		      double *temp) {
    int rc;
    
    
    if (!dev || !temp) {
	errno = EINVAL;
	return -1;
    }

    if (strcmp(dev->given_dev_name, "da") == 0)
	rc = scsi_drive_temperature(dev, temp);
    else if (strcmp(dev->given_dev_name, "ada") == 0)
	rc = ata_drive_temperature(dev, temp);
    else if (strcmp(dev->given_dev_name, "nda") == 0)
	rc = nvme_drive_temperature(dev, temp);
    else {
	errno = ENOENT;
	rc = -1;
    }

    return rc;
}

int
get_smart_status(struct cam_device *dev,
		 struct smart_result *result) {
    int rc;

    
    if (!dev || !result) {
	errno = EINVAL;
	return -1;
    }

    if (strcmp(dev->given_dev_name, "da") == 0)
	rc = scsi_smart_status(dev, result);
    else if (strcmp(dev->given_dev_name, "ada") == 0)
	rc = ata_smart_status(dev, result);
    else if (strcmp(dev->given_dev_name, "nda") == 0)
	rc = nvme_smart_status(dev, result);
    else {
	errno = ENOENT;
	rc = -1;
    }

    return rc;
}

int
get_tests_result(struct cam_device *dev,
		 struct tests_result *result) {
    int rc;

    
    if (!dev || !result) {
	errno = EINVAL;
	return -1;
    }

    if (strcmp(dev->given_dev_name, "da") == 0)
	rc = scsi_tests_status(dev, result);
    else if (strcmp(dev->given_dev_name, "ada") == 0)
	rc = ata_tests_status(dev, result);
    else if (strcmp(dev->given_dev_name, "nda") == 0)
	rc = nvme_tests_status(dev, result);
    else {
	errno = ENOENT;
	rc = -1;
    }

    return rc;
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
    struct health_result health;
    int have_selftest = 0;
    

    /* Increase size of drive table */
    if (dc >= ds) {
	dv = realloc(dv, (ds += 1024)*sizeof(DRIVE));
	if (!dv)
	    return -1;
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

    if (fd >= 0) {
	/* Get drive size */
	if (ioctl(fd, DIOCGMEDIASIZE, &msize) < 0) {
	    fprintf(stderr, "%s: Error: %s: ioctl(DIOCGMEDIASIZE): %s\n",
		    argv0, path, strerror(errno));
	}
    }

    
    cam = cam_open_device(path, O_RDWR);
    if (cam) {
	if (f_debug) {
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
    
	sprintf(pnbuf, "scbus %2u target %3u lun %2jx",
		cam->path_id,
		cam->target_id,
		cam->target_lun);
	
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
	    strdupcat(&dp->danames, dname);
	    strdupcat(&dp->path, pnbuf);
	    strdupcat(&dp->driver, drvbuf);
	} else {
	    /* New drive */
	    
	    dp->ident = ident;
	    dp->danames = strdup(dname);
	    dp->path = strdup(pnbuf);
	    if (msize > 0)
		dp->size = size2str(msize);
	    dp->driver = strdup(drvbuf);
	    
	    if (strcmp(cam->given_dev_name, "nda") == 0) {
		/* NVME device */
		char npath[1024];

		/* Check to make sure it is a local NVME controller */
		if (strcmp(cam->sim_name, "nvme") == 0) {
		    int nfd;
		    int have_selftest = 0;
		    
		    sprintf(npath, "/dev/%s%u", cam->sim_name, cam->sim_unit_number);
		    
		    nfd = open(npath, O_RDONLY);
		    if (nfd < 0) {
			fprintf(stderr, "%s: Error: %s: open(O_RDONLY): %sn",
				argv0, npath, strerror(errno));
			close(fd);
			return -1;
		    }
		    nvme_identify_fd(nfd, &dp->vendor, &dp->product, &dp->revision, NULL, &dp->phys, &have_selftest);
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

		have_selftest = 1;
		/* Get physical path (does not work on NDA/NVME */
		physbuf[0] = '\0';
		if (f_phys) {
		    if (ioctl(fd, DIOCGPHYSPATH, physbuf) < 0) {
			fprintf(stderr, "%s: Error: %s (%d): ioctl(DIOCGPHYSPATH): %s\n",
				argv0, path, fd, strerror(errno));
		    }
		    dp->phys = strdup(physbuf);
		}

		
		if (strcmp(cam->given_dev_name, "ada") == 0) {
		    /* SATA device */
		    ata_identify(cam, &dp->vendor, &dp->product, &dp->revision);
		} 

		/* Probably only da (SAS/SCSI) drives */
		if (!dp->vendor && cam->inq_data.vendor[0])
		    dp->vendor = strdupget(cam->inq_data.vendor, sizeof(cam->inq_data.vendor));
		
		if (!dp->product && cam->inq_data.product[0])
		    dp->product = strdupget(cam->inq_data.product, sizeof(cam->inq_data.product));
		
		if (!dp->revision && cam->inq_data.revision[0])
		    dp->revision = strdupget(cam->inq_data.revision, sizeof(cam->inq_data.revision));

		
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


	    rc = get_health_status(cam, &health, have_selftest);
	    if (rc > 0) {
		char buf[256];
		
		if (health.drive.temperature > 0) {
		    sprintf(buf, "%.1f", health.drive.temperature);
		    dp->temp = strdup(buf);
		}

		if (health.drive.power_on_hours > 0) {
		    sprintf(buf, "%zu", health.drive.power_on_hours);
		    dp->poweron = strdup(buf);
		}
		
		dp->smart = strdup(health_smart_status2str(health.smart.status));
		dp->tests = strdup(health_tests_status2str(health.tests.status));
	    }

	} 
	
	cam_close_device(cam);
	close(fd);
	dc++;
	return 0;
    } else {
	/* Non-CAM device */

	fprintf(stderr, "*** %s: Non-CAM Device\n", path);

	memset(idbuf, 0, sizeof(idbuf));
	if (ioctl(fd, DIOCGIDENT, idbuf) < 0) {
	    fprintf(stderr, "%s: Error: %s: ioctl(DIOCGIDENT): %s",
		    argv0, path, strerror(errno));
	} else
	    ident = strdupget(idbuf, sizeof(idbuf));
	
	if (!ident) {
	    close(fd);
	    return 1;
	}
	
	if (ident && f_multipath) {
	    for (i = 0; i < dc && strcmp(dv[i].ident, ident); i++)
		;
	} else
	    i = dc;
	dp = &dv[i];
	
	physbuf[0] = '\0';
	if (f_phys) {
	    if (ioctl(fd, DIOCGPHYSPATH, physbuf) < 0) {
		fprintf(stderr, "%s: Error: %s (%d): ioctl(DIOCGPHYSPATH): %s\n",
			argv0, path, fd, strerror(errno));
	    }
	}
	
	if (i >= dc) {
	    /* New device */
	    dp->ident = ident;
	    dp->danames = strdup(dname);
	    dp->phys = strdup(physbuf);
	    if (msize > 0)
		dp->size = size2str(msize);
	    ++dc;
	} else {
	    free(ident);
	    strdupcat(&dp->danames, dname);
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
    int sizelen = 4;
    int templen = 4;
    int smartlen = 5;
    int testslen = 5;
    int poweronlen = 7;


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
	    case 'D':
		if (argv[i][j+1] && sscanf(argv[i]+j+1, "%d", &f_debug) != 1) {
		    if (argv[i+1][0] && argv[i+1][0] != '-' && sscanf(argv[i+1], "%d", &f_debug) != 1) {
			fprintf(stderr, "%s: Error: Missing value for -D\n", argv[0]);
			exit(1);
		    }
		} else
		    f_debug++;
		goto NextArg;
	    case 'F':
		if (argv[i][j+1]) {
		    f_sep = strdup(argv[i]+j+1);
		    goto NextArg;
		} else if (argv[i+1] && argv[i+1][0] != '-') {
		    f_sep = strdup(argv[++i]);
		    goto NextArg;
		} else {
		    f_sep = " : ";
		}
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
		f_poweron++;
		if (f_verbose > 3) {
		    f_driver++;
		    if (f_verbose > 4) {
			f_phys++;
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
	strntrim(dv[i].temp, &templen, f_maxwidth);
	strntrim(dv[i].smart, &smartlen, f_maxwidth);
	strntrim(dv[i].tests, &testslen, f_maxwidth);
	strntrim(dv[i].poweron, &poweronlen, f_maxwidth);
    }

    numlen = (int) (log10(dc)+1);
    qsort(&dv[0], dc, sizeof(dv[0]), dv_sort);

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

	if (f_poweron)
	    p_field("POWERON", poweronlen, f_sep);

	p_field("NAMES", -danameslen, f_sep);
	
	if (f_driver) {
	    p_field("DRV.", -drvlen, f_sep);
	    p_field("PATH", -pathlen, f_sep);
	}
	
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

	if (f_poweron)
	    p_field(dv[i].poweron, poweronlen, f_sep);

	p_field(dv[i].danames, -danameslen, f_sep);
	       
	if (f_driver) {
	    p_field(dv[i].driver, -drvlen, f_sep);
	    p_field(dv[i].path, -pathlen, f_sep);
	}
	
	if (f_phys)
	    p_field(dv[i].phys, -physlen, f_sep);

	putchar('\n');
    }

    return rc;
}
