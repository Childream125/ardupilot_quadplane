#pragma once

<<<<<<< HEAD
#define THISFIRMWARE "AP_Periph V1.0"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 1,0,0,FIRMWARE_VERSION_TYPE_OFFICIAL
=======
#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "AP_Periph V1.0dev"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 1,0,0,FIRMWARE_VERSION_TYPE_DEV
>>>>>>> myquadplane

#define FW_MAJOR 1
#define FW_MINOR 0
#define FW_PATCH 0
<<<<<<< HEAD
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
=======
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV
>>>>>>> myquadplane
