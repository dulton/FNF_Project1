
#ifndef __MODEL_CONF_H__
#define __MODEL_CONF_H__

#include "version.h"
#include "ui_sel.h"

#define _JA2104
#define SN_HEAD "I6"
#define _EXT_EFFIO
#define SDK_PLATFORM_HI3521
#define _HI3520A_EXT

#define GPIO_PLAT_TYPE6
#define GPIO_KEYPAD_MATRIX

#define HD_DISPLAY_SIZE_OPTIMISED

#define HWVER_MAJ (1)
#define HWVER_MIN (0)
#define HWVER_REV (0)
#define HWVER_EXT ""

#define MAX_REF_CH (32)
#define MAX_CAM_CH (4)
#define ALL_CAM_BITMASK (0xf)
#define MAX_AUD_CH (2)
#define ALL_AUD_BITMASK (0x3)
#define MAX_SENSOR_CH (4)
#define ALL_SENSOR_BITMASK (0xf)
#define MAX_ALARM_CH (1)
#define ALL_ALARM_BITMASK (0x1)

#define MAX_HDD_CNT (4)

#define MAX_TELECTRL_CH (4)

#define MAX_CIF_CNT (4)
#define MAX_D1_CNT (4)

#define MAX_D1_LIVE_FPS (30)
#define MAX_CIF_LIVE_FPS (30)

#define MAX_D1_ENC_FPS (30)
#define MAX_CIF_ENC_FPS (30)
#define MAX_NET_ENC_FPS (16)
#define MAX_D1_ENC_BPS (2048)
#define MAX_CIF_ENC_BPS (512)

#define MAX_ENC_CH (4)
#define MAX_DEC_CH (4)
#define MAX_PLAYBACK_CH (4)

#if defined(_EXT_JUANOEM)
#define FIRMWARE_MAGIC "JUAN HI3521 JA-R2104 FIRMWARE COPYRIGHT BY JUAN"
#define FIRMWARE_FILE_NAME_PREFIX "FWHJ2104_"
#else
#define FIRMWARE_MAGIC "JUAN HI3521 JA2104 FIRMWARE DESIGNED BY LAW"
#define FIRMWARE_FILE_NAME_PREFIX "FWHI2104_"
#endif

#define DEVICE_FEATURE_NO "112226"

#endif //__MODEL_CONF_H__

