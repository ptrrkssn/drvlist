
#include "health.h"

const char *
health_smart_status2str(enum health_smart_status status) {
    switch (status) {
    case HEALTH_SMART_UNKNOWN:
	return "UNKNOWN";
    case HEALTH_SMART_PASSED:
        return "PASSED";
    case HEALTH_SMART_NONE:
        return "-";
    case HEALTH_SMART_WARNING:
        return "WARNING";
    case HEALTH_SMART_FAILED:
        return "FAILED";
    }

    return "?";
}

const char *
health_tests_status2str(enum health_tests_status status) {
    switch (status) {
    case HEALTH_TESTS_UNKNOWN:
	return "UNKNOWN";
    case HEALTH_TESTS_NONE:
	return "-";
    case HEALTH_TESTS_RUNNING:
	return "RUNNING";
    case HEALTH_TESTS_PASSED:
	return "PASSED";
    case HEALTH_TESTS_FAILED:
	return "FAILED";
    case HEALTH_TESTS_ABORTED:
	return "ABORTED";
    }
    return "?";
}
