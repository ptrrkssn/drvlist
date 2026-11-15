#ifndef DRVLIST_SCSI_H
#define DRVLIST_SCSI_H 1

#include <cam/cam.h>

#include "smart.h"
#include "tests.h"

#if 0
#define SCSI_LOG_SENSE               0x4D
#define SCSI_IE_LOG_PAGE             0x2F
#define SCSI_TEMP_LOG_PAGE           0x0D
#define SCSI_SELFTEST_RESULTS_PAGE   0x10
#endif

extern int
scsi_drive_temperature(struct cam_device *dev,
		       double *temp_c);

extern int
scsi_smart_status(struct cam_device *dev,
		  struct smart_result *result);

extern int
scsi_tests_status(struct cam_device *dev,
		  struct tests_result *result);

extern int
scsi_health_status(struct cam_device *dev,
		   struct health_result *result,
		   int have_selftest);

#endif
