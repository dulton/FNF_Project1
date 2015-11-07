#ifndef __HDDVR_CODE_H__
#define __HDDVR_CODE_H__

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define CHECK_SIZE(n, x) (((n) >= 0) && ((n) < ARRAY_SIZE(x)))

unsigned int  HDDVR_i2c_WriteByte(unsigned char ucChipAddr, unsigned char ucRegAddr, unsigned char ucRegValue);
unsigned char HDDVR_i2c_ReadByte(unsigned char ucChipAddr, unsigned char ucRegAddr);

int HDDVR_Init(int Fmt);
int HDDVR_Exit(void);

typedef enum {
    HDVIDEO_UNKNOWN = -1,
    HDVIDEO_SD720H25FPS,
    HDVIDEO_SD720H30FPS,
    HDVIDEO_SD960H25FPS,
    HDVIDEO_SD960H30FPS,
    HDVIDEO_HD720P25FPS,
    HDVIDEO_HD720P30FPS,
    HDVIDEO_HD720P50FPS,
    HDVIDEO_HD720P60FPS,
    HDVIDEO_HD1080P25FPS,
    HDVIDEO_HD1080P30FPS,
    HDVIDEO_CNT,
    HDVIDEO_SD960H,
    HDVIDEO_HD720P,
    HDVIDEO_HD1080P,
	HDVIDEO_CNT2,
} HD_INPUT_FORMAT;

typedef enum {
	HDDVR_VSUPPORT_SD960H  = 0,
	HDDVR_VSUPPORT_HD720P,
	HDDVR_VSUPPORT_HD1080P,
} HDDVR_VSUPPORT_ENUM;

#define HDVLOSS_ERROR	(-1)
#define HDVLOSS_NOVIDEO  (1)
#define HDVLOSS_INVIDEO  (0)

int HDDVR_InputFormatCheck(int Chnl);
int HDDVR_InputFormatSetting(int Chnl, HD_INPUT_FORMAT Frmt);

int HDDVR_GetFormatInfo(HD_INPUT_FORMAT Frmt, int * SizW, int * SizH, int * Fps);
int HDDVR_VSupportCheck(void);

int HDDVR_VideoSetColor(int Chnl, int nHue, int nBrightness, int nContrast, int nSaturation, int nSharpness);
int HDDVR_VideoLoss(void);
int HDDVR_VideoLossChnl(int Chnl);
int HDDVR_VideoLossFix(int Chnl, int Fix);
int HDDVR_VideoStVideoFix(int Chnl);

char * HDDVR_GetMODELType();
char * HDDVR_GetCHNLType(int Chnl);

int HDDVR_AudioLiveLoop(int Chnl);
int HDDVR_AudioVolume(int Chnl, int Vol);
int HDDVR_AudioPlayBack(int Chnl);

/*����˳���Ѿ������Ʊ�����˳��Ҫע��*/
typedef enum {
	HDDVR_UTC_CMD_UP = 0,
	HDDVR_UTC_CMD_DOWN,
	HDDVR_UTC_CMD_LEFT,
	HDDVR_UTC_CMD_RIGHT,	
	HDDVR_UTC_CMD_IRIS_OPEN,
	HDDVR_UTC_CMD_IRIS_CLOSE,
	HDDVR_UTC_CMD_FOCUS_NEAR,
	HDDVR_UTC_CMD_FOCUS_FAR,
	HDDVR_UTC_CMD_ZOOM_WIDE,
	HDDVR_UTC_CMD_ZOOM_TELE,
	HDDVR_UTC_CMD_OSD,
	HDDVR_UTC_CMD_RESET,
	HDDVR_UTC_CMD_SET, 
	HDDVR_UTC_CMD_COUNT,

	HDDVR_UTC_CMD_MOTOR_STOP = HDDVR_UTC_CMD_OSD,
	HDDVR_UTC_CMD_LEFT_UP,
	HDDVR_UTC_CMD_LEFT_DOWN,
	HDDVR_UTC_CMD_RIGHT_UP,
	HDDVR_UTC_CMD_RIGHT_DOWN,
	HDDVR_UTC_CMD_PRESET_SET,
	HDDVR_UTC_CMD_PRESET_CLR,
	HDDVR_UTC_CMD_PRESET_CALL,
	
	HDDVR_HM_CMD_STOP,
	HDDVR_HM_CMDD_DOWN,
	HDDVR_HM_CMD_UP,
	HDDVR_HM_CMD_LEFT,
	HDDVR_HM_CMD_RIGHT,
	HDDVR_HM_CMD_ZOOM_TELE,
	HDDVR_HM_CMD_ZOOM_WIDE,
	HDDVR_HM_CMD_IRIS_SUM,
	HDDVR_HM_CMD_IRIS_ADD,
	HDDVR_HM_CMD_FOCUS_NEAR,
	HDDVR_HM_CMD_FOCUS_FAR,

	HDDVR_HM_CMD_PRE1,
	HDDVR_HM_CMD_PRE16,
	HDDVR_HM_CMD_PRE128,
	HDDVR_HM_CMD_PRE255,
	HDDVR_HM_CMD_PRE,
	HDDVR_HM_CMD_SETPRE1,
	HDDVR_HM_CMD_SETPRE16,
	HDDVR_HM_CMD_SETPRE128,
	HDDVR_HM_CMD_SETPRE255,
	HDDVR_HM_CMD_SETPRE,
	HDDVR_HM_CMD_CLRPRE,

	HDDVR_HM_CMD_COUNT,
}HDDVR_UTC_CMD;

int HDDVR_UTC_Init();
int HDDVR_UTC_Send(int Chnl, int Cmd, unsigned char value);
int HDDVR_UTC_Exit();

#endif //__HDDVR_CODE_H__

