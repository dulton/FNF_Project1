#include "conf.h"  //Import MAX_CAM_CH Definition
#include "hddvr_code.h"

#include "bsp/decoder_drv.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>

static pthread_mutex_t gI2C_Locker;
#define I2C_InLock() pthread_mutex_lock(&gI2C_Locker)
#define I2C_UnLock() pthread_mutex_unlock(&gI2C_Locker)

enum {
	HDTVI_CHIP_TP2802C = 0x280200,
	HDTVI_CHIP_TP2802D = 0x280201,
	HDTVI_CHIP_TP2804  = 0x280400,
	HDTVI_CHIP_TP2806  = 0x280401,
	HDTVI_CHIP_TP2822  = 0x282200,
	HDTVI_CHIP_TP2823  = 0x282300,
	HDTVI_CHIP_TP2824  = 0x282400,
};

enum {
    TP28XX_MUX_1CH,    //148.5M mode
    TP28XX_MUX_2CH,    //148.5M mode
    TP28XX_DDR_2CH,    //297M mode, HDTVI_CHIP_TP2822/23 feature
    TP28XX_DDR_4CH,    //future support
};

int const HDTVI_CHN_TBL[MAX_CAM_CH][2] = { //The Real Video&Audio Channel Order
#if (MAX_CAM_CH == 4)
	{0, 2}, {0, 3}, {0, 0}, {0, 1},
#endif
#if (MAX_CAM_CH == 8)
	{0, 2}, {0, 3}, {0, 0}, {0, 1},
	{1, 2}, {1, 3}, {1, 0}, {1, 1},
#endif
#if (MAX_CAM_CH == 16)
	{0, 2}, {0, 3}, {0, 0}, {0, 1},
	{1, 2}, {1, 3}, {1, 0}, {1, 1},
	{2, 2}, {2, 3}, {2, 0}, {2, 1},
	{3, 2}, {3, 3}, {3, 0}, {3, 1},
#endif
};

#define HDTVI_ADDR0 (0x88)
#define HDTVI_ADDR1 (0x8A)
#define HDTVI_ADDR2 (0x8C)
#define HDTVI_ADDR3 (0x8E)
static unsigned char HDTVI_ADDR[4] = { HDTVI_ADDR0, HDTVI_ADDR1, HDTVI_ADDR2, HDTVI_ADDR3, };
static int HDTVI_ChipVersion = 0;
static int gHDTVI_VSTD = 0; //0: PAL, 1: NTSC

///***************************************************************///
enum {
	TP28XX_PAGE_V0 = 0,
	TP28XX_PAGE_V1,
	TP28XX_PAGE_V2,
	TP28XX_PAGE_V3,
	TP28XX_PAGE_VALL,
	TP28XX_PAGE_AALL = 9,
};

//TVI ptz
unsigned int i2c_addr, tmp, ret = 0;

typedef struct PTZ_packet
{
    unsigned char header;
    unsigned char addr;
    unsigned char cmd;
    int           sum;
    unsigned char data[4];
} PTZ_packet;

PTZ_packet ptz;
unsigned char tilt_speed=0x01;
unsigned char pan_speed=0x01;
unsigned char presetNUM=0x5F;

static void tp28xx_set_reg_page(int addr, int chnl)
{
	switch(chnl) {
	case TP28XX_PAGE_V0:    HDDVR_i2c_WriteByte(addr, 0x40, 0x00); break;  // VIN1 registers
	case TP28XX_PAGE_V1:    HDDVR_i2c_WriteByte(addr, 0x40, 0x01); break;  // VIN2 registers
	case TP28XX_PAGE_V2:    HDDVR_i2c_WriteByte(addr, 0x40, 0x02); break;  // VIN3 registers
	case TP28XX_PAGE_V3:    HDDVR_i2c_WriteByte(addr, 0x40, 0x03); break;  // VIN4 registers
	case TP28XX_PAGE_VALL:  HDDVR_i2c_WriteByte(addr, 0x40, 0x04); break;  // Write All VIN1-4 regsiters
	case TP28XX_PAGE_AALL:  HDDVR_i2c_WriteByte(addr, 0x40, 0x40); break;  // Audio

	default:                HDDVR_i2c_WriteByte(addr, 0x40, 0x04); break;
	}
	
}
///***************************************************************///

//#define SAMPLE_16K  //if no define, default 8K
#define DATA_16BIT  //if no define, default 8BIT

//HDTVI_CHIP_TP2802D EQ for short cable option
#define TP2802D_EQ_SHORT 0x0d
#define TP2802D_CGAIN_SHORT 0x74

#define BT1120_HEADER_8BIT   0x00 //reg0x02 bit3 0=BT1120,
#define BT656_HEADER_8BIT   0x08 //reg0x02 bit3 1=656,
#define SAV_HEADER_1MUX     BT656_HEADER_8BIT//BT1120_HEADER_8BIT
#define DEFAULT_FORMAT      TP2802_720P25

static int output = TP28XX_MUX_2CH;

void tp28xx_reg_write_table(unsigned char addr, const unsigned char *RegSet)
{
	unsigned char index, val;

	while ((RegSet[0] != 0xFF) || (RegSet[1]!= 0xFF)) {			// 0xff, 0xff is end of data
		index = *RegSet;
		val = *(RegSet+1);

		HDDVR_i2c_WriteByte(addr, index, val);

		RegSet+=2;
	}
}

///***************************************************************///

enum {
	TP2802_1080P25  =  0x03,
	TP2802_1080P30  =  0x02,
	TP2802_720P25   =  0x05,
	TP2802_720P30   =  0x04,
	TP2802_720P50   =  0x01,
	TP2802_720P60   =  0x00,
	TP2802_SD       =  0x06,
	INVALID_FORMAT  =  0x07,
	TP2802_720P25V2 =  0x0D,
	TP2802_720P30V2 =  0x0C,
	TP2802_PAL      =  0x08,
	TP2802_NTSC     =  0x09,
};

// Video Format
const unsigned char tbl_tp2802_1080p25_raster[] = {
	0x15, 0x03,
	0x16, 0xD3,
	0x17, 0x80,
	0x18, 0x29,
	0x19, 0x38,
	0x1A, 0x47,
	0x1B, 0x00,
	0x1C, 0x0A,
	0x1D, 0x50,

	0xFF, 0xFF,
};

const unsigned char tbl_tp2802_1080p30_raster[] = {
	0x15, 0x03,
	0x16, 0xD3,
	0x17, 0x80,
	0x18, 0x29,
	0x19, 0x38,
	0x1A, 0x47,
	0x1B, 0x00,
	0x1C, 0x08,
	0x1D, 0x98,

	0xFF, 0xFF,
};

const unsigned char tbl_tp2802_720p25_raster[] = {
	0x15, 0x13,
	0x16, 0x16,
	0x17, 0x00,
	0x18, 0x19,
	0x19, 0xD0,
	0x1A, 0x25,
	0x1B, 0x00,
	0x1C, 0x0F,
	0x1D, 0x78,

	0xFF, 0xFF,
};

const unsigned char tbl_tp2802_720p30_raster[] = {
	0x15, 0x13,
	0x16, 0x16,
	0x17, 0x00,
	0x18, 0x19,
	0x19, 0xD0,
	0x1A, 0x25,
	0x1B, 0x00,
	0x1C, 0x0C,
	0x1D, 0xE4,

	0xFF, 0xFF,
};

const unsigned char tbl_tp2802_720p50_raster[] = {
	0x15, 0x13,
	0x16, 0x16,
	0x17, 0x00,
	0x18, 0x19,
	0x19, 0xD0,
	0x1A, 0x25,
	0x1B, 0x00,
	0x1C, 0x07,
	0x1D, 0xBC,

	0xFF, 0xFF,
};

const unsigned char tbl_tp2802_720p60_raster[] = {
	0x15, 0x13,
	0x16, 0x16,
	0x17, 0x00,
	0x18, 0x19,
	0x19, 0xD0,
	0x1A, 0x25,
	0x1B, 0x00,
	0x1C, 0x06,
	0x1D, 0x72,

	0xFF, 0xFF,
};

const unsigned char tbl_tp2802_pal_raster[] = {
	0x15, 0x13,
	0x16, 0x5f,
	0x17, 0xbc,
	0x18, 0x17,
	0x19, 0x20,
	0x1A, 0x17,
	0x1B, 0x00,
	0x1C, 0x09,
	0x1D, 0x48,

	0xFF, 0xFF,
};

const unsigned char tbl_tp2802_ntsc_raster[] = {
	0x15, 0x13,
	0x16, 0x4e,
	0x17, 0xbc,
	0x18, 0x15,
	0x19, 0xf0,
	0x1A, 0x07,
	0x1B, 0x00,
	0x1C, 0x09,
	0x1D, 0x38,

	0xFF, 0xFF,
};

static int tp28xx_video_set_mode(int addr, int chnl, int fmt)
{
	int err = 0;
	unsigned char tmp = 0;

	tp28xx_set_reg_page(addr, chnl);

    tmp = HDDVR_i2c_ReadByte(addr, 0x02);
	switch(fmt) {
	case TP2802_1080P25:
		tp28xx_reg_write_table(addr, tbl_tp2802_1080p25_raster); //1080P25
		tmp &=0xF8;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_1080P30:
		tp28xx_reg_write_table(addr, tbl_tp2802_1080p30_raster); //1080P30
		tmp &=0xF8;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_720P25:
		tp28xx_reg_write_table(addr, tbl_tp2802_720p25_raster); //720P25
		tmp &=0xF8;
		tmp |=0x02;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_720P30:
		tp28xx_reg_write_table(addr, tbl_tp2802_720p30_raster); //720P30
		tmp &=0xF8;
		tmp |=0x02;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_720P50:
		tp28xx_reg_write_table(addr, tbl_tp2802_720p50_raster); //720P50
		tmp &=0xF8;
		tmp |=0x02;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_720P60:
		tp28xx_reg_write_table(addr, tbl_tp2802_720p60_raster); //720P60
		tmp &=0xF8;
		tmp |=0x02;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_720P25V2:
		tp28xx_reg_write_table(addr, tbl_tp2802_720p50_raster); //720P50
		tmp &=0xF8;
		tmp |=0x02;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_720P30V2:
		tp28xx_reg_write_table(addr, tbl_tp2802_720p60_raster); //720P60
		tmp &=0xF8;
		tmp |=0x02;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_PAL:
		tp28xx_reg_write_table(addr, tbl_tp2802_pal_raster); //PAL
		tmp &=0xF8;
		tmp |=0x07;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;
	case TP2802_NTSC:
		tp28xx_reg_write_table(addr, tbl_tp2802_ntsc_raster); //NTSC
		tmp &=0xF8;
		tmp |=0x07;
		HDDVR_i2c_WriteByte(addr, 0x02, tmp);
		break;

	default:
		err = -1;
		break;
	}

	return err;
}

static int tp28xx_video_get_mode(int addr, int chnl, int *fmt)
{
	unsigned char tmpFmt;
	unsigned char tmpPrg;
	int tmpRet = INVALID_FORMAT;

	tp28xx_set_reg_page(addr, chnl);

	tmpFmt  = HDDVR_i2c_ReadByte(addr, 0x03);
	tmpFmt &= 0x7; /* [2:0] - CVSTD */

	tmpPrg  = HDDVR_i2c_ReadByte(addr, 0x01);
	tmpPrg  = (tmpPrg & 0x2) >> 1; /* [2] - NINTL */

	switch(tmpFmt) {
		case TP2802_1080P25:
		case TP2802_1080P30:
		case TP2802_720P25:
		case TP2802_720P30:
		case TP2802_720P50:
		case TP2802_720P60:
		case TP2802_720P25V2:
		case TP2802_720P30V2:
			if(tmpPrg == 1) { //PROGRESSIVE VIDEO INPUT
				tmpRet = tmpFmt;
			}
		break;

		case TP2802_SD:
		case TP2802_PAL:
		case TP2802_NTSC:
			if(tmpPrg == 0) { //INTERLACED VIDEO INPUT
				tmpRet = tmpFmt;
			}
		case INVALID_FORMAT:
			break;
	}

	return tmpRet;
}

static int tp28xx_get_videoloss(int addr, int chnl)
{
	unsigned char tmp;

	tp28xx_set_reg_page(addr, chnl);

	tmp = HDDVR_i2c_ReadByte(addr, 0x01);
	tmp = (tmp & 0x80) >> 7; /* [7] - VDLOSS */
	if(!tmp) {
		if(0x08 == HDDVR_i2c_ReadByte(addr, 0x2f)) { //TEST Register
			tmp = HDDVR_i2c_ReadByte(addr, 0x04);
			if(tmp < 0x30) tmp = 0;
			else tmp = 1;
		}

	}

	return tmp;
}
///***************************************************************///

#define TP28XX_BRIGHTNESS       0x10
#define TP28XX_CONTRAST         0x11
#define TP28XX_SATURATION       0x12
#define TP28XX_HUE              0X13
#define TP28XX_SHARPNESS        0X14
#if 0
static int tp28xx_video_set_color(int addr, int chnl, int colorcomp, int colorvalue)
{
	unsigned char tmp;

	tp28xx_set_reg_page(addr, chnl);

	HDDVR_i2c_WriteByte(addr, TP28XX_BRIGHTNESS, brightness);
	HDDVR_i2c_WriteByte(addr, TP28XX_CONTRAST,   contrast);
	HDDVR_i2c_WriteByte(addr, TP28XX_SATURATION, saturation);
	HDDVR_i2c_WriteByte(addr, TP28XX_HUE,        hue);
	HDDVR_i2c_WriteByte(addr, TP28XX_SHARPNESS, (sharpness & 0x1F));

	return 0;
}
#endif

///***************************************************************///
static void tp28xx_ptz_init(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0xC8, 0x21); //change default 40bit to 34bit
	HDDVR_i2c_WriteByte(addr, 0xE0, 0x21);

}

static void tp28xx_ptz_mode(unsigned char addr, unsigned char ch)
{
    unsigned int tmp;

    static const unsigned char PTZ_reg1[4]={0xC6,0xDE,0xC6,0xDE};
    static const unsigned char PTZ_reg2[4]={0xC7,0xDF,0xC7,0xDF};
    static const unsigned char PTZ_bank[4]={0x00,0x00,0x10,0x10};

    tmp = HDDVR_i2c_ReadByte(addr, 0x40);
    tmp &= 0xef;
    tmp |=PTZ_bank[ch];
    HDDVR_i2c_WriteByte(addr, 0x40, tmp); //reg bank1 switch for 2822
    tmp = HDDVR_i2c_ReadByte(addr, 0xf5); //check TVI 1 or 2
	if((tmp >>ch) & 0x01) {
		HDDVR_i2c_WriteByte(addr, PTZ_reg1[ch], 0x33);
		HDDVR_i2c_WriteByte(addr, PTZ_reg2[ch], 0xf0);
	}
	else {
		HDDVR_i2c_WriteByte(addr, PTZ_reg1[ch], 0x19);
		HDDVR_i2c_WriteByte(addr, PTZ_reg2[ch], 0x78);
	}
}

static int tp28xx_ptz_sendcmd(int addr, int chnl, PTZ_packet ptzcmd, unsigned char value)
{
	unsigned char tmp = 0;
	for(tmp=0;tmp<4;tmp++){
		printf("ptzcmd.data[%d]=%x\n",tmp, ptzcmd.data[tmp]);
	}
	if((HDTVI_CHIP_TP2822 ==  HDTVI_ChipVersion)
	|| (HDTVI_CHIP_TP2823 ==  HDTVI_ChipVersion)
	|| (HDTVI_CHIP_TP2824 ==  HDTVI_ChipVersion)) {
		tp28xx_ptz_mode(addr, chnl);

		tmp = HDDVR_i2c_ReadByte(addr, 0x40);
		tmp &=0xef;
		HDDVR_i2c_WriteByte(addr, 0x40, tmp); //data buffer bank0 switch for 2822
        
		// TX disable
		HDDVR_i2c_WriteByte(addr, 0xBB, HDDVR_i2c_ReadByte(addr,0xBB) & ~(0x01<<(chnl)));

		//line1
		HDDVR_i2c_WriteByte(addr, 0x56 + chnl*10 , 0x02);
		HDDVR_i2c_WriteByte(addr, 0x57 + chnl*10 , ptzcmd.header);
		HDDVR_i2c_WriteByte(addr, 0x58 + chnl*10 , ptzcmd.addr);
		HDDVR_i2c_WriteByte(addr, 0x59 + chnl*10 , ptzcmd.cmd);
		HDDVR_i2c_WriteByte(addr, 0x5A + chnl*10 , ptzcmd.data[0]);		
		//line2
		HDDVR_i2c_WriteByte(addr, 0x5B + chnl*10 , 0x02);
		HDDVR_i2c_WriteByte(addr, 0x5C + chnl*10 , ptzcmd.data[1]);
		HDDVR_i2c_WriteByte(addr, 0x5D + chnl*10 , ptzcmd.data[2]);
		HDDVR_i2c_WriteByte(addr, 0x5E + chnl*10 , ptzcmd.data[3]);
		HDDVR_i2c_WriteByte(addr, 0x5F + chnl*10 , ptzcmd.sum);

		// TX enable
		HDDVR_i2c_WriteByte(addr, 0xBB, (0x01<<(chnl)));
	}
	else if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
		HDDVR_i2c_WriteByte(addr, 0x40, 0x00); //bank switch for D

		tmp = HDDVR_i2c_ReadByte(addr, 0xf5); //check TVI 1 or 2
		if((tmp >> chnl) & 0x01) {
			HDDVR_i2c_WriteByte(addr, 0x53, 0x33);
			HDDVR_i2c_WriteByte(addr, 0x54, 0xf0);
		}
		else {
			 HDDVR_i2c_WriteByte(addr, 0x53, 0x19);
			 HDDVR_i2c_WriteByte(addr, 0x54, 0x78);
		}

		// TX disable
		HDDVR_i2c_WriteByte(addr, 0xBB, HDDVR_i2c_ReadByte(addr,0xBB) & ~(0x01<<(chnl)));

		//line1
		HDDVR_i2c_WriteByte(addr, 0x56 + chnl*10 , 0x02);
		HDDVR_i2c_WriteByte(addr, 0x57 + chnl*10 , ptzcmd.header);
		HDDVR_i2c_WriteByte(addr, 0x58 + chnl*10 , ptzcmd.addr);
		HDDVR_i2c_WriteByte(addr, 0x59 + chnl*10 , ptzcmd.cmd);
		HDDVR_i2c_WriteByte(addr, 0x5A + chnl*10 , ptzcmd.data[0]);
		//line2
		HDDVR_i2c_WriteByte(addr, 0x5B + chnl*10 , 0x02);
		HDDVR_i2c_WriteByte(addr, 0x5C + chnl*10 , ptzcmd.data[1]);
		HDDVR_i2c_WriteByte(addr, 0x5D + chnl*10 , ptzcmd.data[2]);
		HDDVR_i2c_WriteByte(addr, 0x5E + chnl*10 , ptzcmd.data[3]);
		HDDVR_i2c_WriteByte(addr, 0x5F + chnl*10 , ptzcmd.sum);

		// TX enable
		HDDVR_i2c_WriteByte(addr, 0xBB, (0x01<<(chnl)));
	}
	else if(HDTVI_CHIP_TP2802C == HDTVI_ChipVersion) {
		//line1
		HDDVR_i2c_WriteByte(addr, 0x56 + chnl*10 , 0x02);
		HDDVR_i2c_WriteByte(addr, 0x57 + chnl*10 , ptzcmd.header);
		HDDVR_i2c_WriteByte(addr, 0x58 + chnl*10 , ptzcmd.addr);
		HDDVR_i2c_WriteByte(addr, 0x59 + chnl*10 , ptzcmd.cmd);
		HDDVR_i2c_WriteByte(addr, 0x5A + chnl*10 , ptzcmd.data[0]);
		//line2
		HDDVR_i2c_WriteByte(addr, 0x5B + chnl*10 , 0x02);
		HDDVR_i2c_WriteByte(addr, 0x5C + chnl*10 , ptzcmd.data[1]);
		HDDVR_i2c_WriteByte(addr, 0x5D + chnl*10 , ptzcmd.data[2]);
		HDDVR_i2c_WriteByte(addr, 0x5E + chnl*10 , ptzcmd.data[3]);
		HDDVR_i2c_WriteByte(addr, 0x5F + chnl*10 , ptzcmd.sum);

		// TX enable
		HDDVR_i2c_WriteByte(addr, 0x7e, 0x20|(0x01<<(chnl)));
	}

	return 0;
}

static int tp28xx_ptz_recvcmd(int addr, int chnl, char buf[], int siz)
{
	unsigned char tmp = 0;

	if((HDTVI_CHIP_TP2822 == HDTVI_ChipVersion)
	|| (HDTVI_CHIP_TP2823 == HDTVI_ChipVersion)
	|| (HDTVI_CHIP_TP2824 == HDTVI_ChipVersion)) {
		tmp  = HDDVR_i2c_ReadByte(addr, 0x40);
		tmp &=0xef;
		HDDVR_i2c_WriteByte(addr, 0x40, tmp); //bank switch for 2822
	}
	else if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
		HDDVR_i2c_WriteByte(addr, 0x40, 0x00); //bank switch for D
	}

	// line1
	buf[0] = HDDVR_i2c_ReadByte(addr, 0x8C + chnl*10);
	buf[1] = HDDVR_i2c_ReadByte(addr, 0x8D + chnl*10);
	buf[2] = HDDVR_i2c_ReadByte(addr, 0x8E + chnl*10);
	buf[3] = HDDVR_i2c_ReadByte(addr, 0x8F + chnl*10);

	//line2
	buf[4] = HDDVR_i2c_ReadByte(addr, 0x91 + chnl*10);
	buf[5] = HDDVR_i2c_ReadByte(addr, 0x92 + chnl*10);
	buf[6] = HDDVR_i2c_ReadByte(addr, 0x93 + chnl*10);
	buf[7] = HDDVR_i2c_ReadByte(addr, 0x94 + chnl*10);

	return 0;
}

///***************************************************************///
static void tp2802_manual_agc(int addr, int chnl)
{
    unsigned int agc, tmp;

    HDDVR_i2c_WriteByte(addr, 0x2F, 0x02);
    agc = HDDVR_i2c_ReadByte(addr, 0x04);
    printf("AGC=0x%04x ch%02x\r\n", agc, chnl);
	agc += HDDVR_i2c_ReadByte(addr, 0x04);
	agc += HDDVR_i2c_ReadByte(addr, 0x04);
	agc += HDDVR_i2c_ReadByte(addr, 0x04);
	agc &= 0x3f0;
	agc >>=1;
    if(agc > 0x1ff) agc = 0x1ff;

	printf("AGC=0x%04x ch%02x\r\n", agc, chnl);
	HDDVR_i2c_WriteByte(addr, 0x08, agc&0xff);
	tmp = HDDVR_i2c_ReadByte(addr, 0x06);
	tmp &=0xf9;
	tmp |=(agc>>7)&0x02;
	tmp |=0x04;
	HDDVR_i2c_WriteByte(addr, 0x06,tmp);
}

static const unsigned char SYS_MODE[4]={0x01,0x02,0x04,0x08};
static const unsigned char SYS_AND[4] ={0xfe,0xfd,0xfb,0xf7};
static const unsigned char CLK_MODE[4]={0x01,0x10,0x01,0x10};
static const unsigned char CLK_ADDR[4]={0xfa,0xfa,0xfb,0xfb};
static const unsigned char CLK_AND[4] ={0xf8,0x8f,0xf8,0x8f};

static void tp282x_sysclk_v2(int addr, int chnl)
{
    unsigned char tmp;
    tmp  = HDDVR_i2c_ReadByte(addr, 0xf5);
    tmp |= SYS_MODE[chnl];
    HDDVR_i2c_WriteByte(addr, 0xf5, tmp);
    if((TP28XX_MUX_2CH == output) || (TP28XX_DDR_4CH == output)) {
        HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
    }
    else if(TP28XX_MUX_1CH == output) {
        HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
        tmp  = HDDVR_i2c_ReadByte(addr,CLK_ADDR[chnl]);
        tmp &= CLK_AND[chnl];
        tmp |= CLK_MODE[chnl];
        HDDVR_i2c_WriteByte(addr, CLK_ADDR[chnl], tmp);
    }
}

static void tp282x_sysclk_v1(int addr, int chnl)
{
    unsigned char tmp;
    tmp  = HDDVR_i2c_ReadByte(addr, 0xf5);
    tmp &= SYS_AND[chnl];
    HDDVR_i2c_WriteByte(addr, 0xf5, tmp);
    if((TP28XX_MUX_2CH == output) || (TP28XX_DDR_4CH == output)) {
        HDDVR_i2c_WriteByte(addr, 0x35, 0x45);
    }
    else if(TP28XX_MUX_1CH == output || TP28XX_DDR_2CH == output) {
        HDDVR_i2c_WriteByte(addr, 0x35, 0x05);
        tmp  = HDDVR_i2c_ReadByte(addr,CLK_ADDR[chnl]);
        tmp &= CLK_AND[chnl];
        HDDVR_i2c_WriteByte(addr, CLK_ADDR[chnl], tmp);
    }
}

static void tp28xx_reset_default(int addr, int chnl)
{
	unsigned char tmp = 0;

    if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion) {
		tmp = HDDVR_i2c_ReadByte(addr, 0x06);
		tmp &= 0xfb;
		HDDVR_i2c_WriteByte(addr, 0x06, tmp);
		HDDVR_i2c_WriteByte(addr, 0x07, 0x40);
		//HDDVR_i2c_WriteByte(addr, 0x35, 0x45);
		HDDVR_i2c_WriteByte(addr, 0x26, 0x01);
		tp282x_sysclk_v1(addr, chnl);
	}
    else if(HDTVI_CHIP_TP2822 == HDTVI_ChipVersion) {
		tmp = HDDVR_i2c_ReadByte(addr, 0x06);
		tmp &= 0xfb;
		HDDVR_i2c_WriteByte(addr, 0x06, tmp);
		HDDVR_i2c_WriteByte(addr, 0x07, 0x40);
		//HDDVR_i2c_WriteByte(addr, 0x35, 0x45);
		tp282x_sysclk_v1(addr, chnl);
	}
	else if(HDTVI_CHIP_TP2802C == HDTVI_ChipVersion) {
		HDDVR_i2c_WriteByte(addr, 0x2F, 0x08);
		HDDVR_i2c_WriteByte(addr, 0x07, 0x0d);
		HDDVR_i2c_WriteByte(addr, 0x3a, 0x02);
	}
    else if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
		HDDVR_i2c_WriteByte(addr, 0x3A, 0x01);
		HDDVR_i2c_WriteByte(addr, 0x0B, 0xC0);
		HDDVR_i2c_WriteByte(addr, 0x07, 0xC0);
		HDDVR_i2c_WriteByte(addr, 0x2e, 0x70);
		HDDVR_i2c_WriteByte(addr, 0x39, 0x42);
		HDDVR_i2c_WriteByte(addr, 0x09, 0x24);
		tmp = HDDVR_i2c_ReadByte(addr, 0x06);   // soft reset and auto agc when cable is unplug
		tmp &= 0x7b;
		HDDVR_i2c_WriteByte(addr, 0x06, tmp);

		tmp = HDDVR_i2c_ReadByte(addr, 0xf5);
		tmp &= SYS_AND[chnl];
		HDDVR_i2c_WriteByte(addr, 0xf5, tmp);
		tmp = HDDVR_i2c_ReadByte(addr,CLK_ADDR[chnl]);
		tmp &= CLK_AND[chnl];
		HDDVR_i2c_WriteByte(addr, CLK_ADDR[chnl], tmp);
    }
    else if(HDTVI_CHIP_TP2824 == HDTVI_ChipVersion) {
		//HDDVR_i2c_WriteByte(addr, 0x07, 0x40);
		//HDDVR_i2c_WriteByte(addr, 0x0B, 0x40);
		//HDDVR_i2c_WriteByte(addr, 0x3A, 0x88);
		HDDVR_i2c_WriteByte(addr, 0x26, 0x01);
    }
}

static void tp2823_ntsc_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x53);
	HDDVR_i2c_WriteByte(addr, 0x0d, 0x10);
	HDDVR_i2c_WriteByte(addr, 0x20, 0xa0);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x12);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x68);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x5e);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x62);
	HDDVR_i2c_WriteByte(addr, 0x31, 0xbb);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x96);
	HDDVR_i2c_WriteByte(addr, 0x33, 0xc0);
	HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x10);
}

static void tp2823_pal_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x53);
	HDDVR_i2c_WriteByte(addr, 0x0d, 0x11);
	HDDVR_i2c_WriteByte(addr, 0x20, 0xb0);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x02);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x60);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x5e);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x7a);
	HDDVR_i2c_WriteByte(addr, 0x31, 0x4a);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x4d);
	HDDVR_i2c_WriteByte(addr, 0x33, 0xf0);
	HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x10);
}

static void tp2823_v1_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x43);
	HDDVR_i2c_WriteByte(addr, 0x0d, 0x10);
	HDDVR_i2c_WriteByte(addr, 0x20, 0x60);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x02);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x30);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x70);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
	HDDVR_i2c_WriteByte(addr, 0x31, 0xbb);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x2e);
	HDDVR_i2c_WriteByte(addr, 0x33, 0x90);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x30);
}

static void tp2823_v2_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x53);
	HDDVR_i2c_WriteByte(addr, 0x0d, 0x10);
	HDDVR_i2c_WriteByte(addr, 0x20, 0x60);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x02);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x30);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x70);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
	HDDVR_i2c_WriteByte(addr, 0x31, 0xbb);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x2e);
	HDDVR_i2c_WriteByte(addr, 0x33, 0x90);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x20);
}

static void tp282x_audio_dataset(int addr)
{
    unsigned int tmp;
    tmp = HDDVR_i2c_ReadByte(addr, 0x40);
    tmp |=0x40;
	HDDVR_i2c_WriteByte(addr, 0x40, tmp);

	HDDVR_i2c_WriteByte(addr, 0x00, 0x01); //channel
	HDDVR_i2c_WriteByte(addr, 0x01, 0x02);
	HDDVR_i2c_WriteByte(addr, 0x02, 0x03);
	HDDVR_i2c_WriteByte(addr, 0x03, 0x04);
	HDDVR_i2c_WriteByte(addr, 0x04, 0x11);

#ifdef DATA_16BIT
	HDDVR_i2c_WriteByte(addr, 0x17, 0x00);
	HDDVR_i2c_WriteByte(addr, 0x1B, 0x01);
#else
	HDDVR_i2c_WriteByte(addr, 0x17, 0x04);
	HDDVR_i2c_WriteByte(addr, 0x1B, 0x41);
#endif

#ifdef SAMPLE_16K
	HDDVR_i2c_WriteByte(addr, 0x18, 0x11);
#else
	HDDVR_i2c_WriteByte(addr, 0x18, 0x10);
#endif

	HDDVR_i2c_WriteByte(addr, 0x19, 0x0F);
	HDDVR_i2c_WriteByte(addr, 0x1A, 0x15);

	HDDVR_i2c_WriteByte(addr, 0x37, 0x20);
	HDDVR_i2c_WriteByte(addr, 0x38, 0x38);
	if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion) {
		HDDVR_i2c_WriteByte(addr, 0x3E, 0x06);
	}
	if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion) {
		HDDVR_i2c_WriteByte(addr, 0x3E, 0x00);
	}
    HDDVR_i2c_WriteByte(addr, 0x3d, 0x01);//audio reset

	HDDVR_i2c_WriteByte(addr, 0x3C, 0x20);
	HDDVR_i2c_WriteByte(addr, 0x3C, 0x00);

	HDDVR_i2c_WriteByte(addr, 0x1D, 0x08);
	HDDVR_i2c_WriteByte(addr, 0x1D, 0x00);

    tmp &=0xBF;
	HDDVR_i2c_WriteByte(addr, 0x40, tmp);
}

////////////////////////////////////////////////////////////////
static void tp2824_ntsc_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x43);

	HDDVR_i2c_WriteByte(addr, 0x0d, 0x10);
	HDDVR_i2c_WriteByte(addr, 0x20, 0xa0);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x12);
	HDDVR_i2c_WriteByte(addr, 0x2b, 0x70);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x68);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x5e);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x62);
	HDDVR_i2c_WriteByte(addr, 0x31, 0xbb);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x96);
	HDDVR_i2c_WriteByte(addr, 0x33, 0xc0);
	HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x84);
}

static void tp2824_pal_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x53);

	HDDVR_i2c_WriteByte(addr, 0x0d, 0x11);
	HDDVR_i2c_WriteByte(addr, 0x20, 0xb0);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x02);
	HDDVR_i2c_WriteByte(addr, 0x2b, 0x70);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x60);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x5e);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x7a);
	HDDVR_i2c_WriteByte(addr, 0x31, 0x4a);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x4d);
	HDDVR_i2c_WriteByte(addr, 0x33, 0xf0);
	HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x84);
}

static void tp2824_v1_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x14, 0x00);
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x03);
	HDDVR_i2c_WriteByte(addr, 0x0d, 0x10);
	HDDVR_i2c_WriteByte(addr, 0x20, 0x60);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x02);
	HDDVR_i2c_WriteByte(addr, 0x2b, 0x58);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x30);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x70);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
	HDDVR_i2c_WriteByte(addr, 0x31, 0xbb);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x2e);
	HDDVR_i2c_WriteByte(addr, 0x33, 0x90);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x8C);
}

static void tp2824_v2_dataset(int addr)
{
	HDDVR_i2c_WriteByte(addr, 0x0c, 0x03);
	HDDVR_i2c_WriteByte(addr, 0x0d, 0x10);
	HDDVR_i2c_WriteByte(addr, 0x20, 0x60);
	HDDVR_i2c_WriteByte(addr, 0x26, 0x02);
	HDDVR_i2c_WriteByte(addr, 0x2b, 0x58);
	HDDVR_i2c_WriteByte(addr, 0x2d, 0x30);
	HDDVR_i2c_WriteByte(addr, 0x2e, 0x70);

	HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
	HDDVR_i2c_WriteByte(addr, 0x31, 0xbb);
	HDDVR_i2c_WriteByte(addr, 0x32, 0x2e);
	HDDVR_i2c_WriteByte(addr, 0x33, 0x90);
	HDDVR_i2c_WriteByte(addr, 0x39, 0x88);
}

static void tp282x_video_output(int addr)
{
    HDDVR_i2c_WriteByte(addr, 0xF5, 0x00);

    if(TP28XX_MUX_2CH == output) {
        //2CH-MUX output
        HDDVR_i2c_WriteByte(addr, 0xF5, 0x0f);
        HDDVR_i2c_WriteByte(addr, 0xF6, 0x10);
        HDDVR_i2c_WriteByte(addr, 0xF7, 0x32);
        HDDVR_i2c_WriteByte(addr, 0x40, 0x04);
        HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
        HDDVR_i2c_WriteByte(addr, 0x02, 0xca);  //BT656 header
    }
    else if(TP28XX_MUX_1CH == output) {
        //1CH-MUX output
        HDDVR_i2c_WriteByte(addr, 0xF6, 0x00);
        HDDVR_i2c_WriteByte(addr, 0xF7, 0x11);
        HDDVR_i2c_WriteByte(addr, 0x40, 0x04);
        HDDVR_i2c_WriteByte(addr, 0x35, 0x05);
        HDDVR_i2c_WriteByte(addr, 0x02, 0xc2|SAV_HEADER_1MUX); //BT1120/BT656 header
    }
    else if(TP28XX_DDR_2CH == output) {
        HDDVR_i2c_WriteByte(addr, 0x45, 0x54); //PLL 297M
        HDDVR_i2c_WriteByte(addr, 0xF4, 0x60); //output clock 148.5M
        HDDVR_i2c_WriteByte(addr, 0xF6, 0x18);
        HDDVR_i2c_WriteByte(addr, 0xF7, 0x32);
        HDDVR_i2c_WriteByte(addr, 0x40, 0x04);
        HDDVR_i2c_WriteByte(addr, 0x35, 0x05);
        HDDVR_i2c_WriteByte(addr, 0x02, 0xca);  //BT656 header
    }
}

static void tp2824_4mux_output(int addr)  //FIXME
{
    if(TP28XX_DDR_4CH == output) {
        HDDVR_i2c_WriteByte(addr, 0xF5, 0x0f);
        HDDVR_i2c_WriteByte(addr, 0x45, 0x54); //PLL 297M
        HDDVR_i2c_WriteByte(addr, 0xF4, 0x60); //output clock 148.5M
        HDDVR_i2c_WriteByte(addr, 0xF5, 0x0f);
        HDDVR_i2c_WriteByte(addr, 0xF6, 0x18); //
        HDDVR_i2c_WriteByte(addr, 0xF7, 0x10);
        HDDVR_i2c_WriteByte(addr, 0xF8, 0x10); //
        HDDVR_i2c_WriteByte(addr, 0xF9, 0x10);
        HDDVR_i2c_WriteByte(addr, 0x50, 0xB2); //
        HDDVR_i2c_WriteByte(addr, 0x51, 0xB2);
        HDDVR_i2c_WriteByte(addr, 0x52, 0xB2); //
        HDDVR_i2c_WriteByte(addr, 0x53, 0xB2);
        HDDVR_i2c_WriteByte(addr, 0x40, 0x04);
        HDDVR_i2c_WriteByte(addr, 0x35, 0x25);
        HDDVR_i2c_WriteByte(addr, 0x02, 0xca);  //BT656 header
    }
}

static void tp28xx_channelid(int addr) //FIXME
{
    HDDVR_i2c_WriteByte(addr, 0x40, 0x00);
    HDDVR_i2c_WriteByte(addr, 0x34, 0x10);
    HDDVR_i2c_WriteByte(addr, 0x40, 0x01);
    HDDVR_i2c_WriteByte(addr, 0x34, 0x11);
    HDDVR_i2c_WriteByte(addr, 0x40, 0x02);
    HDDVR_i2c_WriteByte(addr, 0x34, 0x10);
    HDDVR_i2c_WriteByte(addr, 0x40, 0x03);
    HDDVR_i2c_WriteByte(addr, 0x34, 0x11);
}

static void tp28xx_reg_comm_init(int addr)
{
    tp28xx_set_reg_page(addr, TP28XX_PAGE_VALL);
    if(HDTVI_CHIP_TP2824 == HDTVI_ChipVersion) {
        HDDVR_i2c_WriteByte(addr, 0x07, 0xC0);
        HDDVR_i2c_WriteByte(addr, 0x0B, 0xC0);
        //HDDVR_i2c_WriteByte(addr, 0x3A, 0x70);
        HDDVR_i2c_WriteByte(addr, 0x26, 0x01);
        HDDVR_i2c_WriteByte(addr, 0x22, 0x35);
        HDDVR_i2c_WriteByte(addr, 0x39, 0x8C);

        HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
        HDDVR_i2c_WriteByte(addr, 0x31, 0xBB);
        HDDVR_i2c_WriteByte(addr, 0x32, 0x2E);
        HDDVR_i2c_WriteByte(addr, 0x33, 0x90);

        HDDVR_i2c_WriteByte(addr, 0x4D, 0x0f);
        HDDVR_i2c_WriteByte(addr, 0x4E, 0x0f);

        //channel ID
        tp28xx_channelid(addr);

        //MUX output
        tp282x_video_output(addr);
        tp2824_4mux_output(addr);
        HDDVR_i2c_WriteByte(addr, 0xFA, 0x88);
        HDDVR_i2c_WriteByte(addr, 0xFB, 0x88);

        //bank 1 for TX/RX 3,4
        HDDVR_i2c_WriteByte(addr, 0x40, 0x10);
        tp28xx_ptz_init(addr);
        //bank 0 for TX/RX 1,2
        HDDVR_i2c_WriteByte(addr, 0x40, 0x04);
        tp28xx_ptz_init(addr);
        HDDVR_i2c_WriteByte(addr, 0x7E, 0x0F);   //TX channel enable
        HDDVR_i2c_WriteByte(addr, 0xB9, 0x0F);   //RX channel enable

        tp282x_audio_dataset(addr);
    }
    else if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion) {
 	    HDDVR_i2c_WriteByte(addr, 0x0b, 0x60);
	    HDDVR_i2c_WriteByte(addr, 0x22, 0x34);
	    HDDVR_i2c_WriteByte(addr, 0x23, 0x44);
	    HDDVR_i2c_WriteByte(addr, 0x38, 0x40);
        HDDVR_i2c_WriteByte(addr, 0x26, 0x01);

        HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
        HDDVR_i2c_WriteByte(addr, 0x31, 0xBB);
        HDDVR_i2c_WriteByte(addr, 0x32, 0x2E);
        HDDVR_i2c_WriteByte(addr, 0x33, 0x90);
        //HDDVR_i2c_WriteByte(addr, 0x35, 0x45);
	    HDDVR_i2c_WriteByte(addr, 0x4D, 0x03);
        HDDVR_i2c_WriteByte(addr, 0x4E, 0x33);

        //output
        tp282x_video_output(addr);
        HDDVR_i2c_WriteByte(addr, 0xFA, 0x00);

        //channel ID
        tp28xx_channelid(addr);

        //bank 1 for TX/RX 3,4
        HDDVR_i2c_WriteByte(addr, 0x40, 0x10);
        tp28xx_ptz_init(addr);
        //bank 0 for TX/RX 1,2
        HDDVR_i2c_WriteByte(addr, 0x40, 0x04);
        tp28xx_ptz_init(addr);
        HDDVR_i2c_WriteByte(addr, 0x7E, 0x0F);   //TX channel enable
        HDDVR_i2c_WriteByte(addr, 0xB9, 0x0F);   //RX channel enable

        tp282x_audio_dataset(addr);
    }
    else if(HDTVI_CHIP_TP2822 == HDTVI_ChipVersion) {
	    //HDDVR_i2c_WriteByte(addr, 0x07, 0xC0);
	    //HDDVR_i2c_WriteByte(addr, 0x0B, 0xC0);
	    HDDVR_i2c_WriteByte(addr, 0x22, 0x38);
	    HDDVR_i2c_WriteByte(addr, 0x2E, 0x60);

        HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
        HDDVR_i2c_WriteByte(addr, 0x31, 0xBB);
        HDDVR_i2c_WriteByte(addr, 0x32, 0x2E);
        HDDVR_i2c_WriteByte(addr, 0x33, 0x90);

        //HDDVR_i2c_WriteByte(addr, 0x35, 0x45);
	    HDDVR_i2c_WriteByte(addr, 0x39, 0x00);
	    HDDVR_i2c_WriteByte(addr, 0x3A, 0x01);
	    HDDVR_i2c_WriteByte(addr, 0x3D, 0x08);
	    HDDVR_i2c_WriteByte(addr, 0x4D, 0x03);
        HDDVR_i2c_WriteByte(addr, 0x4E, 0x33);

        //2CH-MUX output
        tp282x_video_output(addr);
        HDDVR_i2c_WriteByte(addr, 0xFA, 0x88);

        //channel ID
        tp28xx_channelid(addr);

        //bank 1 for TX/RX 3,4
        HDDVR_i2c_WriteByte(addr, 0x40, 0x10);
        tp28xx_ptz_init(addr);
        //bank 0 for TX/RX 1,2
        HDDVR_i2c_WriteByte(addr, 0x40, 0x00);
        tp28xx_ptz_init(addr);
        HDDVR_i2c_WriteByte(addr, 0x7E, 0x0F);   //TX channel enable
        HDDVR_i2c_WriteByte(addr, 0xB9, 0x0F);   //RX channel enable

        HDDVR_i2c_WriteByte(addr, 0x40, 0x04); //all ch reset
        HDDVR_i2c_WriteByte(addr, 0x3D, 0x00);
	}
    else if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
        HDDVR_i2c_WriteByte(addr, 0x02, 0xC0|SAV_HEADER_1MUX); //8 bit BT1120/BT656 mode
	    HDDVR_i2c_WriteByte(addr, 0x07, 0xC0);
	    HDDVR_i2c_WriteByte(addr, 0x0B, 0xC0);
	    HDDVR_i2c_WriteByte(addr, 0x2b, 0x4a);
	    HDDVR_i2c_WriteByte(addr, 0x2E, 0x60);

        HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
        HDDVR_i2c_WriteByte(addr, 0x31, 0xBB);
        HDDVR_i2c_WriteByte(addr, 0x32, 0x2E);
        HDDVR_i2c_WriteByte(addr, 0x33, 0x90);

	    HDDVR_i2c_WriteByte(addr, 0x23, 0x50);
	    HDDVR_i2c_WriteByte(addr, 0x39, 0x42);
	    HDDVR_i2c_WriteByte(addr, 0x3A, 0x01);
	    HDDVR_i2c_WriteByte(addr, 0x4d, 0x0f);
        HDDVR_i2c_WriteByte(addr, 0x4e, 0xff);

        //now TP2801A just support 2 lines, to disable line3&4, else IRQ is in trouble.
        HDDVR_i2c_WriteByte(addr, 0x40, 0x01);
        HDDVR_i2c_WriteByte(addr, 0x50, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x51, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x52, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x7F, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x80, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x81, 0x00);

        //0x50~0xb2 need bank switch
        HDDVR_i2c_WriteByte(addr, 0x40, 0x00);
        //TX total 34bit, valid load 32bit
        HDDVR_i2c_WriteByte(addr, 0x50, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x51, 0x0b);
        HDDVR_i2c_WriteByte(addr, 0x52, 0x0c);
        HDDVR_i2c_WriteByte(addr, 0x53, 0x19);
        HDDVR_i2c_WriteByte(addr, 0x54, 0x78);
        HDDVR_i2c_WriteByte(addr, 0x55, 0x21);
        HDDVR_i2c_WriteByte(addr, 0x7e, 0x0f);   //

        // RX total 40bit, valid load 39bit
        HDDVR_i2c_WriteByte(addr, 0x7F, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x80, 0x07);
        HDDVR_i2c_WriteByte(addr, 0x81, 0x08);
        HDDVR_i2c_WriteByte(addr, 0x82, 0x04);
        HDDVR_i2c_WriteByte(addr, 0x83, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x84, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x85, 0x60);
        HDDVR_i2c_WriteByte(addr, 0x86, 0x10);
        HDDVR_i2c_WriteByte(addr, 0x87, 0x06);
        HDDVR_i2c_WriteByte(addr, 0x88, 0xBE);
        HDDVR_i2c_WriteByte(addr, 0x89, 0x39);
        HDDVR_i2c_WriteByte(addr, 0x8A, 0x27);   //
        HDDVR_i2c_WriteByte(addr, 0xB9, 0x0F);   //RX channel enable
	}
	else if(HDTVI_CHIP_TP2802C == HDTVI_ChipVersion) {
		// PLLs
		const unsigned char tbl_tp2802c_common_pll[] = {
			0x42, 0x00,
			0x43, 0x12,
			0x44, 0x07,
			0x45, 0x49,

			0xFF, 0xFF,
		};

		// Output Enables
		const unsigned char tbl_tp2802c_common_oe[] = {
			0x4B, 0x10,
			0x4C, 0x32,
			0x4D, 0x0F,
			0x4E, 0xFF,
			//0x4F, 0x0F,
			//0x50, 0x00,
			//0x51, 0x0A,
			//0x52, 0x0B,
			//0x53, 0x1F,
			//0x54, 0xFA,
			//0x55, 0x27,

			0xFF, 0xFF,
		};

		// Rx Common
		const unsigned char tbl_tp2802c_common_rx[] = {
			0x7E, 0x2F,
			0x7F, 0x00,
			0x80, 0x07,
			0x81, 0x08,
			0x82, 0x04,
			0x83, 0x00,
			0x84, 0x00,
			0x85, 0x60,
			0x86, 0x10,
			0x87, 0x06,
			0x88, 0xBE,
			0x89, 0x39,
			0x8A, 0xA7,

			0xFF, 0xFF,
		};

		// IRQ Common
		const unsigned char tbl_tp2802c_common_irq[] = {
			0xB3, 0xFA,
			0xB4, 0x1C,
			0xB5, 0x0F,
			0xB6, 0xFF,
			0xB7, 0x00,
			0xB8, 0x00,

			0xFF, 0xFF,
		};

        HDDVR_i2c_WriteByte(addr, 0x06, 0xC0); //8bit BT1120 mode only
        HDDVR_i2c_WriteByte(addr, 0x07, 0x0D);
        HDDVR_i2c_WriteByte(addr, 0x08, 0xE0);
        HDDVR_i2c_WriteByte(addr, 0x26, 0x12);
        HDDVR_i2c_WriteByte(addr, 0x2A, 0x44);
        HDDVR_i2c_WriteByte(addr, 0x2C, 0x0A);
        HDDVR_i2c_WriteByte(addr, 0x2E, 0x60);
        HDDVR_i2c_WriteByte(addr, 0x2F, 0x08);

        HDDVR_i2c_WriteByte(addr, 0x30, 0x48);
        HDDVR_i2c_WriteByte(addr, 0x31, 0xBB);
        HDDVR_i2c_WriteByte(addr, 0x32, 0x2E);
        HDDVR_i2c_WriteByte(addr, 0x33, 0x90);

        HDDVR_i2c_WriteByte(addr, 0x38, 0xC0);
        HDDVR_i2c_WriteByte(addr, 0x39, 0xCA);

        HDDVR_i2c_WriteByte(addr, 0x3A, 0x02);
        HDDVR_i2c_WriteByte(addr, 0x3D, 0x20);
        HDDVR_i2c_WriteByte(addr, 0x3E, 0x02);

        // PLLs, Start address 0x42, Size = 4B
        tp28xx_reg_write_table(addr, tbl_tp2802c_common_pll);

        // Rx Common, Start address 0x7E, Size = 13B
        tp28xx_reg_write_table(addr, tbl_tp2802c_common_rx);

        // IRQ Common, Start address 0xB3, Size = 6B
        //tp28xx_reg_write_table(addr, tbl_tp2802c_common_irq);

        // Output Enables, Start address 0x4B, Size = 11B
        tp28xx_reg_write_table(addr, tbl_tp2802c_common_oe);

        //TX total 34bit, valid load 32bit
        HDDVR_i2c_WriteByte(addr, 0x50, 0x00);
        HDDVR_i2c_WriteByte(addr, 0x51, 0x0b);
        HDDVR_i2c_WriteByte(addr, 0x52, 0x0c);
        HDDVR_i2c_WriteByte(addr, 0x53, 0x19);
        HDDVR_i2c_WriteByte(addr, 0x54, 0x78);
        HDDVR_i2c_WriteByte(addr, 0x55, 0x21);
        //HDDVR_i2c_WriteByte(addr, 0x56, 0x02); //should be 0 when disable sending
        //HDDVR_i2c_WriteByte(addr, 0x7e, 0x2f);
    }
}

int HDTVI_ChipGetMap(int Chnl, int *RealChip, int *RealChnl, int *RealAddr)
{
	int tmpRealChip;
	int tmpRealChnl;
	int tmpRealAddr;

	if(!CHECK_SIZE(Chnl, HDTVI_CHN_TBL)) {
		return -1;
	}

	tmpRealChip = HDTVI_CHN_TBL[Chnl][0];
	tmpRealChnl = HDTVI_CHN_TBL[Chnl][1];
	tmpRealAddr = HDTVI_ADDR[tmpRealChip];

	if(RealChip) {
		*RealChip = tmpRealChip;
	}
	if(RealChnl) {
		*RealChnl = tmpRealChnl%4;
	}
	if(RealAddr) {
		*RealAddr = tmpRealAddr;
	}

	return 0;
}

enum {
    VIDEO_UNPLUG,
    VIDEO_IN,
    VIDEO_LOCKED,
    VIDEO_UNLOCK,
};

enum {
	VIDEO_SCAN_AUTO = 0,
	VIDEO_SCAN_MANUAL,
};

enum {
	VIDEO_FORCE_RESET_DISABLE = 0,
	VIDEO_FORCE_RESET_ENABLE,
};

#define FLAG_LOSS           0x80
#define FLAG_TVI_LOCKED     0x20
#define FLAG_CVBS_LOCKED    0x60
#define MAX_COUNT           0xffff //FIXME

static int HDTVI_Format[MAX_CAM_CH];
static int HDTVI_Status[MAX_CAM_CH];
static int HDTVI_Counts[MAX_CAM_CH];
static int HDTVI_MlGain[MAX_CAM_CH][4];
static int HDTVI_AutoScan[MAX_CAM_CH];
static int HDTVI_ForceRst[MAX_CAM_CH];

static int SleepWithFlagCheck(int * WaitFlag, int Unit, int Numb)
{
	while(Numb --) {
		if(0 == (*WaitFlag)) {
			return -1;
		}
		usleep(Unit);
	}

	return 0;
}

static void *tp28xx_video_proc_loop(void *param)
{
    unsigned int status, cvstd, gain, agc, tmp, flag_locked;
	int * proc_lock = (int *)param;

    const unsigned char UV_offset[16]={0x00,0x00,0x04,0x04,0x08,0x08,0x0c,0x0c, \
									0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};
    //const unsigned char UV_offset[16]={0x00,0x00,0x0b,0x0b,0x16,0x16,0x21,0x21,  \
	//								0x21,0x21,0x21,0x21,0x21,0x21,0x21,0x21};
    //const unsigned char AGC_table[16]={0x50,0x50,0x60,0x60,0x70,0x80,0x90,0xa0, \
	//								0xb0,0xc0,0xd0,0xd0,0xd0,0xd0,0xd0,0xd0};
    const unsigned char TP2802D_CGAIN[16]   ={0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70, \
									0x70,0x70,0x70,0x60,0x50,0x40,0x38,0x30};
    const unsigned char TP2802D_CGAINMAX[16]={0x4a,0x4a,0x4a,0x4a,0x4a,0x4a,0x4a,0x4a, \
									0x4a,0x4a,0x4a,0x44,0x44,0x44,0x44,0x44};
    const unsigned char TP2823_CGAINMAX[16] ={0x4a,0x47,0x46,0x45,0x44,0x44,0x43,0x43, \
									0x42,0x42,0x42,0x42,0x40,0x40,0x40,0x40};

	int Chnl = 0;
	for(Chnl = 0; Chnl < ARRAY_SIZE(HDTVI_Format); Chnl ++) {
		HDTVI_Format[Chnl] = INVALID_FORMAT;
		HDTVI_Status[Chnl] = VIDEO_UNPLUG;
		HDTVI_Counts[Chnl] = 0;
		HDTVI_AutoScan[Chnl] = VIDEO_SCAN_AUTO;
		HDTVI_ForceRst[Chnl] = VIDEO_FORCE_RESET_DISABLE;

		memset(HDTVI_MlGain, 0, sizeof(HDTVI_MlGain));
	}

	while(*proc_lock) {
	if(VIDEO_SCAN_AUTO != HDTVI_AutoScan[0]) {
		continue;
	}

	for(Chnl = 0; Chnl < ARRAY_SIZE(HDTVI_CHN_TBL); Chnl ++) {
		int tmpRealChip;
		int tmpRealChnl;
		int tmpRealAddr;

		if(0 != HDTVI_ChipGetMap(Chnl, &tmpRealChip, &tmpRealChnl, &tmpRealAddr)) {
			continue;
		}

		I2C_InLock();
		tp28xx_set_reg_page(tmpRealAddr, tmpRealChnl);
		status = HDDVR_i2c_ReadByte(tmpRealAddr, 0x01);
		if(0x08 == HDDVR_i2c_ReadByte(tmpRealAddr, 0x2f)) {
			tmp = HDDVR_i2c_ReadByte(tmpRealAddr, 0x04);
			if(tmp >= 0x30)
				status |= 0x80;
		}
		I2C_UnLock();

		//state machine for video checking
		if(status & FLAG_LOSS) {
			if(VIDEO_UNPLUG == HDTVI_Status[Chnl]) {//still no video
				if(HDTVI_Counts[Chnl] < MAX_COUNT)
					HDTVI_Counts[Chnl]++;
				continue;
			}
			else {//switch to no video
				HDTVI_Status[Chnl] = VIDEO_UNPLUG;
				HDTVI_Counts[Chnl] = 0;
				HDTVI_Format[Chnl] = INVALID_FORMAT;

				I2C_InLock();
				tp28xx_set_reg_page(tmpRealAddr, tmpRealChnl);
				tp28xx_reset_default(tmpRealAddr, tmpRealChnl);
				I2C_UnLock();
				printf("video loss ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
			}
		}
		else {
			if((TP2802_PAL == HDTVI_Format[Chnl]) || (TP2802_NTSC == HDTVI_Format[Chnl]))
				flag_locked = FLAG_CVBS_LOCKED;
			else
				flag_locked = FLAG_TVI_LOCKED;

			if(flag_locked == (status & flag_locked)) {//video locked
				if(VIDEO_LOCKED == HDTVI_Status[Chnl]) {
					if(HDTVI_Counts[Chnl] < MAX_COUNT)
						HDTVI_Counts[Chnl]++;
				}
				else if(VIDEO_UNPLUG == HDTVI_Status[Chnl]) {
					HDTVI_Status[Chnl] = VIDEO_IN;
					HDTVI_Counts[Chnl] = 0;
					printf("video in ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
				}
				else if(HDTVI_Format[Chnl] != INVALID_FORMAT) {
					HDTVI_Status[Chnl] = VIDEO_LOCKED;
					HDTVI_Counts[Chnl] = 0;
					printf("video locked ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
				}
			}
			else {//video in but unlocked
				if(VIDEO_UNPLUG == HDTVI_Status[Chnl]) {
					HDTVI_Status[Chnl] = VIDEO_IN;
					HDTVI_Counts[Chnl] = 0;
					printf("video in ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
				}
				else if(VIDEO_LOCKED == HDTVI_Status[Chnl]) {
					HDTVI_Status[Chnl] = VIDEO_UNLOCK;
					HDTVI_Counts[Chnl] = 0;
					printf("video unstable ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
				}
				else {
					if(HDTVI_Counts[Chnl] < MAX_COUNT) {
						HDTVI_Counts[Chnl]++;
					}
					if((VIDEO_UNLOCK == HDTVI_Status[Chnl]) && (HDTVI_Counts[Chnl] > 2)) {
						HDTVI_Status[Chnl] = VIDEO_IN;
						HDTVI_Counts[Chnl] = 0;

						I2C_InLock();
						tp28xx_set_reg_page(tmpRealAddr, tmpRealChnl);
						tp28xx_reset_default(tmpRealAddr, tmpRealChnl);
						I2C_UnLock();

						printf("video unlocked ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
					}
				}
			}

			if(HDTVI_ForceRst[Chnl]) {//manual reset for V1/2 switching
				HDTVI_Status[Chnl] = VIDEO_UNPLUG;
				HDTVI_Counts[Chnl] = 0;
				HDTVI_Format[Chnl] = INVALID_FORMAT;
				HDTVI_ForceRst[Chnl] = 0;

				I2C_InLock();
				tp28xx_set_reg_page(tmpRealAddr, tmpRealChnl);
				tp28xx_reset_default(tmpRealAddr, tmpRealChnl);
				I2C_UnLock();

				printf("video reset ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
			}
		}

		I2C_InLock();
		if(VIDEO_IN == HDTVI_Status[Chnl]) {
			if(HDTVI_Counts[Chnl] == 0) {
				if((HDTVI_CHIP_TP2822 == HDTVI_ChipVersion)
				|| (HDTVI_CHIP_TP2823 == HDTVI_ChipVersion)
				|| (HDTVI_CHIP_TP2824 == HDTVI_ChipVersion)) {
					tp282x_sysclk_v1(tmpRealAddr, tmpRealChnl);
				}
			}

			tp28xx_set_reg_page(tmpRealAddr, tmpRealChnl);
			cvstd = HDDVR_i2c_ReadByte(tmpRealAddr, 0x03);
			printf("video format %2x detected ch%02x Addr%2x\r\n", cvstd, Chnl, tmpRealAddr);

			cvstd &= 0x0f;
			if(TP2802_SD == (cvstd&0x07)) {
				if((HDTVI_CHIP_TP2823 == HDTVI_ChipVersion)
				|| (HDTVI_CHIP_TP2824 == HDTVI_ChipVersion)) {
					if(HDTVI_Counts[Chnl] & 0x01) {
						HDTVI_Format[Chnl] = TP2802_PAL;
						tp28xx_video_set_mode(tmpRealAddr, tmpRealChnl, HDTVI_Format[Chnl]);
						if(HDTVI_CHIP_TP2824 == HDTVI_ChipVersion) {
							tp2824_pal_dataset(tmpRealAddr);
						}
						if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion) {
							tp2823_pal_dataset(tmpRealAddr);
							if(0x02==tmpRealChnl) //FIXME
								HDDVR_i2c_WriteByte(tmpRealAddr, 0x0d, 0x1);
						}
						printf("set PAL format ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
					}
					else {
						HDTVI_Format[Chnl] = TP2802_NTSC;
						tp28xx_video_set_mode(tmpRealAddr, tmpRealChnl, HDTVI_Format[Chnl]);
						if(HDTVI_CHIP_TP2824 == HDTVI_ChipVersion) {
							tp2824_ntsc_dataset(tmpRealAddr);
						}
						if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion) {
							tp2823_ntsc_dataset(tmpRealAddr);
							if(0x02==tmpRealChnl) //FIXME
								HDDVR_i2c_WriteByte(tmpRealAddr, 0x0d, 0x0);
						}
						printf("set NTSC format ch%02x Addr%2x\r\n", Chnl, tmpRealAddr);
					}
					tp282x_sysclk_v2(tmpRealAddr, tmpRealChnl);
				}
			}
			else if((cvstd&0x07) < 6) {
				HDTVI_Format[Chnl] = cvstd&0x07;
				tp28xx_video_set_mode(tmpRealAddr, tmpRealChnl, HDTVI_Format[Chnl]);

				if((HDTVI_CHIP_TP2822 == HDTVI_ChipVersion)
				|| (HDTVI_CHIP_TP2823 == HDTVI_ChipVersion)
				|| (HDTVI_CHIP_TP2824 == HDTVI_ChipVersion)) {
					tp282x_sysclk_v1(tmpRealAddr, tmpRealChnl);
					if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion)
						tp2823_v1_dataset(tmpRealAddr);
					if(HDTVI_CHIP_TP2824 == HDTVI_ChipVersion)
						tp2824_v1_dataset(tmpRealAddr);
				}
			}
		}
		I2C_UnLock();

		#define TP28XX_EQ_COUNT 10
		I2C_InLock();
		tp28xx_set_reg_page(tmpRealAddr, tmpRealChnl);
		if(VIDEO_LOCKED == HDTVI_Status[Chnl]) {//check signal lock
			if(0 == HDTVI_Counts[Chnl]) {
				if(HDTVI_CHIP_TP2802C == HDTVI_ChipVersion) {
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x07, 0x0d);
				}
				else {
					if((TP2802_720P30==HDTVI_Format[Chnl])
					|| (TP2802_720P25==HDTVI_Format[Chnl])) {
						if(0x08 & HDDVR_i2c_ReadByte(tmpRealAddr, 0x03)) {
							printf("720P V2 Detected ch%02x addr%2x\r\n", Chnl, tmpRealAddr);

							HDTVI_Format[Chnl] |= 0x08;
							tp28xx_video_set_mode(tmpRealAddr, tmpRealChnl, HDTVI_Format[Chnl]); //to speed the switching

							if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
								tmp = HDDVR_i2c_ReadByte(tmpRealAddr, 0xf5);
								tmp |= SYS_MODE[tmpRealChnl];
								HDDVR_i2c_WriteByte(tmpRealAddr, 0xf5, tmp);

								tmp = HDDVR_i2c_ReadByte(tmpRealAddr, CLK_ADDR[tmpRealChnl]);
								tmp &= CLK_AND[tmpRealChnl];
								tmp |= CLK_MODE[tmpRealChnl];
								HDDVR_i2c_WriteByte(tmpRealAddr, CLK_ADDR[tmpRealChnl], tmp);
							}
							else { //TP2822/23//24
								tp282x_sysclk_v2(tmpRealAddr, tmpRealChnl);
								if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion)
									tp2823_v2_dataset(tmpRealAddr);
								if(HDTVI_CHIP_TP2824 == HDTVI_ChipVersion)
									tp2824_v2_dataset(tmpRealAddr);
                            }
						}
					}
				}
			}
			else if(1 == HDTVI_Counts[Chnl]) {
				if(HDTVI_CHIP_TP2802C == HDTVI_ChipVersion) {
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x2F, 0x02);
					agc = HDDVR_i2c_ReadByte(tmpRealAddr, 0x04);
					printf("AGC=0x%04x ch%02x\r\n", agc, Chnl);
					agc += HDDVR_i2c_ReadByte(tmpRealAddr, 0x04);
					agc += HDDVR_i2c_ReadByte(tmpRealAddr, 0x04);
					agc += HDDVR_i2c_ReadByte(tmpRealAddr, 0x04);
					agc &= 0x3f0;
					agc >>=1;
					agc += 0x80;
					if(agc > 0x1c0) agc = 0x1c0;

					HDDVR_i2c_WriteByte(tmpRealAddr, 0x08, agc&0xff);
					printf("AGC=0x%04x ch%02x\r\n", agc, Chnl);
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x07, 0x0f);
				}
			}
			else if(HDTVI_Counts[Chnl] < TP28XX_EQ_COUNT-3) {
				//KEEPME
			}
			else if(HDTVI_Counts[Chnl] < TP28XX_EQ_COUNT) {
				gain = HDDVR_i2c_ReadByte(tmpRealAddr, 0x03);
				HDTVI_MlGain[Chnl][TP28XX_EQ_COUNT-HDTVI_Counts[Chnl]] = gain&0xf0;
				printf("Egain=0x%02x ch%02x\r\n", gain, Chnl);
			}
			else if(HDTVI_Counts[Chnl] < TP28XX_EQ_COUNT+TP28XX_EQ_COUNT ) {//add timeout handle
				HDTVI_MlGain[Chnl][3] = HDTVI_MlGain[Chnl][2];
				HDTVI_MlGain[Chnl][2] = HDTVI_MlGain[Chnl][1];
				HDTVI_MlGain[Chnl][1] = HDTVI_MlGain[Chnl][0];

				gain = HDDVR_i2c_ReadByte(tmpRealAddr, 0x03);
				HDTVI_MlGain[Chnl][0] = gain&0xf0;
				if((abs(HDTVI_MlGain[Chnl][3] - HDTVI_MlGain[Chnl][0])< 0x20)
				&& (abs(HDTVI_MlGain[Chnl][2] - HDTVI_MlGain[Chnl][0])< 0x20)
				&& (abs(HDTVI_MlGain[Chnl][1] - HDTVI_MlGain[Chnl][0])< 0x20)) {
					HDTVI_Counts[Chnl] = TP28XX_EQ_COUNT+TP28XX_EQ_COUNT-1;
				}
			}
			else if(HDTVI_Counts[Chnl] == TP28XX_EQ_COUNT+TP28XX_EQ_COUNT) {
				if(HDTVI_CHIP_TP2822 == HDTVI_ChipVersion) {
					tmp = HDDVR_i2c_ReadByte(tmpRealAddr, 0x03);
					printf("result Egain=0x%02x ch%02x\r\n",tmp, Chnl);
					tmp &=0xf0;
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x07, tmp>>2);  // manual mode
					tmp >>=4;
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x2b, TP2823_CGAINMAX[tmp]);
				}
				else if(HDTVI_CHIP_TP2802C == HDTVI_ChipVersion) {
					gain  = HDTVI_MlGain[Chnl][0];
					gain += HDTVI_MlGain[Chnl][1];
					gain += HDTVI_MlGain[Chnl][2];
					gain += HDTVI_MlGain[Chnl][3];
					gain >>=2;
					gain &=0xf0;
					if(gain > 0x70) gain = 0x70;
					printf("result Egain=0x%02x ch%02x\r\n", gain, Chnl);

					if(gain < 0x20) HDDVR_i2c_WriteByte(tmpRealAddr, 0x3a, 0x0a);
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x07, gain|0x0b);
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x13, UV_offset[gain>>4]); //color offset adjust
				}
				else if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
					tmp = HDDVR_i2c_ReadByte(tmpRealAddr, 0x03);
					printf("result Egain=0x%02x ch%02x\r\n",tmp, Chnl);

					tmp &=0xf0;
					tmp >>=4;
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x2e, TP2802D_CGAIN[tmp]);
					HDDVR_i2c_WriteByte(tmpRealAddr, 0x2b, TP2802D_CGAINMAX[tmp]);

					if(tmp < 0x02) {
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x3A, TP2802D_EQ_SHORT);  // for short cable
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x2e, TP2802D_CGAIN_SHORT);
					}
					if((tmp > 0x03) && ((TP2802_720P30V2 == HDTVI_Format[Chnl])
									|| (TP2802_720P25V2 == HDTVI_Format[Chnl]))) {
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x3A, 0x09);  // long cable
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x07, 0x40);  // for long cable
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x09, 0x20);  // for long cable
						tmp = HDDVR_i2c_ReadByte(tmpRealAddr, 0x06);
						tmp |=0x80;
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x06,tmp);
					}
				}
				else if(HDTVI_CHIP_TP2823 == HDTVI_ChipVersion) {
					if((TP2802_PAL == HDTVI_Format[Chnl])
					|| (TP2802_NTSC == HDTVI_Format[Chnl])) {
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x2b, 0x70);
					}
					else {
						tmp = HDDVR_i2c_ReadByte(tmpRealAddr, 0x03);
						printf("result Egain=0x%02x ch%02x\r\n", tmp, Chnl);
						tmp >>=4;
						HDDVR_i2c_WriteByte(tmpRealAddr, 0x2b, TP2823_CGAINMAX[tmp]);
					}
				}
			}
			else if(HDTVI_Counts[Chnl] == TP28XX_EQ_COUNT+TP28XX_EQ_COUNT+1) {
				if((HDTVI_CHIP_TP2822 == HDTVI_ChipVersion)
				|| (HDTVI_CHIP_TP2823 == HDTVI_ChipVersion)) {
					tp2802_manual_agc(tmpRealAddr, tmpRealChnl);
				}
				else if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
					if((TP2802_720P30V2 != HDTVI_Format[Chnl])
					&& (TP2802_720P25V2 != HDTVI_Format[Chnl])) {
						tp2802_manual_agc(tmpRealAddr, tmpRealChnl);
					}
				}
			}
			else if(HDTVI_Counts[Chnl] == TP28XX_EQ_COUNT+TP28XX_EQ_COUNT+5) {
				if(HDTVI_CHIP_TP2802D == HDTVI_ChipVersion) {
					if((TP2802_720P30V2 == HDTVI_Format[Chnl])
					|| (TP2802_720P25V2 == HDTVI_Format[Chnl])) {
						tp2802_manual_agc(tmpRealAddr, tmpRealChnl);
					}
				}
			}
		}
		I2C_UnLock();
	}
	SleepWithFlagCheck(proc_lock, 20000, 50); //1000ms Sleep
	}

	pthread_exit(NULL);
}

static pthread_t g_TP28XX_Video_Proc_Tid = NULL;
static int g_TP28XX_Video_Proc_LockFlag = 0;

static int tp28xx_video_proc_init()
{
	g_TP28XX_Video_Proc_LockFlag = 1;
	pthread_create(&g_TP28XX_Video_Proc_Tid, NULL, tp28xx_video_proc_loop, &g_TP28XX_Video_Proc_LockFlag);
}

static int tp28xx_video_proc_exit()
{
	g_TP28XX_Video_Proc_LockFlag = 0;
	pthread_join(g_TP28XX_Video_Proc_Tid, NULL);
	g_TP28XX_Video_Proc_Tid = NULL;
}

int HDTVI_Probe(void)
{
	int Result = -1;

	int tmpChip = 0;  //Chip ID

	I2C_InLock();
	tmpChip = ((HDDVR_i2c_ReadByte(HDTVI_ADDR[0], 0xFE) << 16)
			| (HDDVR_i2c_ReadByte(HDTVI_ADDR[0], 0xFF) << 8)
			| (HDDVR_i2c_ReadByte(HDTVI_ADDR[0], 0xFD) & 0x07));
	I2C_UnLock();

	switch(tmpChip) {
		case HDTVI_CHIP_TP2802C:
		case HDTVI_CHIP_TP2802D:
		case HDTVI_CHIP_TP2823:
		case HDTVI_CHIP_TP2824:
			HDTVI_ChipVersion = tmpChip;
			Result = 0;
			break;
		default:
		case HDTVI_CHIP_TP2804:
		case HDTVI_CHIP_TP2806:
		case HDTVI_CHIP_TP2822:
			HDTVI_ChipVersion = 0;
			Result = -1;
			break;
	};

	if (Result == 0) {
		printf("TVI Decoder Probed chip version is 0x%08X\n", HDTVI_ChipVersion);
	}

	return Result;
}

int HDTVI_Init(int Fmt)
{
	pthread_mutex_init(&gI2C_Locker, NULL);

	int ii;
	for(ii = 0; ii < ARRAY_SIZE(HDTVI_ADDR); ii ++) {
		int addr = HDTVI_ADDR[ii];
		unsigned char tmpRead = 0;

		I2C_InLock();
		tp28xx_reg_comm_init(addr);

		tp28xx_video_set_mode(addr, TP28XX_PAGE_VALL, DEFAULT_FORMAT);

        //PLL reset
		tmpRead = HDDVR_i2c_ReadByte(addr, 0x44);
		HDDVR_i2c_WriteByte(addr, 0x44, tmpRead|0x40);

		usleep(10);
		HDDVR_i2c_WriteByte(addr, 0x44, tmpRead);

		//ADC reset
		HDDVR_i2c_WriteByte(addr, 0x40, 0x04); //write 4 channels
        HDDVR_i2c_WriteByte(addr, 0x3B, 0x33);
        HDDVR_i2c_WriteByte(addr, 0x3B, 0x03);

		//soft reset
		tmpRead = HDDVR_i2c_ReadByte(addr, 0x06);
		HDDVR_i2c_WriteByte(addr, 0x06, 0x80|tmpRead);

		I2C_UnLock();
	}

	tp28xx_video_proc_init();

	return 0;
}

int HDTVI_Exit(void)
{
	tp28xx_video_proc_exit();

	pthread_mutex_destroy(&gI2C_Locker);

	return 0;
}

int HDTVI_InputFormatCheck(int Chnl)
{
	int tmpFormatVal = HDVIDEO_UNKNOWN;

	if(HDVLOSS_INVIDEO != HDTVI_VideoVLossCheck(Chnl)) {
		return HDVIDEO_UNKNOWN;
	}

	switch(HDTVI_Format[Chnl]) { //Query Buffed Video Format Result
		case TP2802_1080P25:
			tmpFormatVal = HDVIDEO_HD1080P25FPS;
			break;
		case TP2802_1080P30:
			tmpFormatVal = HDVIDEO_HD1080P30FPS;
			break;

		case TP2802_720P25:
		case TP2802_720P25V2:
			tmpFormatVal = HDVIDEO_HD720P25FPS;
			break;
		case TP2802_720P30:
		case TP2802_720P30V2:
			tmpFormatVal = HDVIDEO_HD720P30FPS;
			break;

		case TP2802_720P50:
			tmpFormatVal = HDVIDEO_HD720P50FPS;
			break;
		case TP2802_720P60:
			tmpFormatVal = HDVIDEO_HD720P60FPS;
			break;

		case TP2802_PAL:
			tmpFormatVal = HDVIDEO_SD960H25FPS;
			break;
		case TP2802_NTSC:
			tmpFormatVal = HDVIDEO_SD960H30FPS;
			break;

		default:
		case TP2802_SD:
		case INVALID_FORMAT:
			tmpFormatVal = HDVIDEO_UNKNOWN;
			break;
	};

	return tmpFormatVal;
}

int HDTVI_InputFormatSetting(int Chnl, HD_INPUT_FORMAT Frmt)
{
	return 0;
}

int HDTVI_VideoVLossCheck(int Chnl)
{
	if(!CHECK_SIZE(Chnl, HDTVI_CHN_TBL)) {
		return HDVLOSS_NOVIDEO;
	}

    if(HDTVI_Status[Chnl] == VIDEO_LOCKED) {
		return HDVLOSS_INVIDEO;
	}

	return HDVLOSS_NOVIDEO;
}

int HDTVI_VideoVLockCheck(int Chnl)
{
	if(!CHECK_SIZE(Chnl, HDTVI_CHN_TBL)) {
		return HDVLOSS_NOVIDEO;
	}

    if(HDTVI_Status[Chnl] == VIDEO_LOCKED) {
		return HDVLOSS_INVIDEO;
	}

	return HDVLOSS_NOVIDEO;
}

int HDTVI_VideoLossFix(int Chnl, int Fix)
{

	return 0;
}

int HDTVI_VideoSetColor(int Chnl, int nHue, int nBrightness, int nContrast, int nSaturation, int nSharpness)
{
	int tmpRealChip;
	int tmpRealChnl;
	int tmpRealAddr;

	#define VIDEO_COLOR_ADJTBL0_2 \
			{ -64,-62,-60,-58,-56,-54,-52,-50,\
			-48,-46,-44,-42,-40,-38,-36,-34,\
			-32,-30,-28,-26,-24,-22,-20,-18,\
			-16,-14,-12,-10,-8,-6,-4,-2,\
			0,\
			2,4,6,8,10,12,14,16,\
			18,20,22,24,26,28,30,32,\
			34,36,38,40,42,44,46,48,\
			50,52,54,56,58,60,62,63}
	#define VIDEO_COLOR_ADJTBL1_2 \
			{0,2,4,6,8,10,12,14, \
			16,18,20,22,24,26,28,30, \
			32,34,36,38,40,42,44,46, \
			48,50,52,54,56,58,60,62, \
			64, \
			66,68,70,72,74,76,78,80, \
			82,84,86,88,90,92,94,96, \
			98,100,102,104,106,108,110,112, \
			114,116,118,120,122,124,126,127}
	enum {
		HDTVI_COLOR_HUE = 0,
		HDTVI_COLOR_BRI,
		HDTVI_COLOR_CON,
		HDTVI_COLOR_SAT,
		HDTVI_COLOR_LST
	};
	const unsigned char COLOR_MAP[HDTVI_COLOR_LST][VIDEO_MAX_COLOR_GAIN+1] = {
		VIDEO_COLOR_ADJTBL0, //HUE
		VIDEO_COLOR_ADJTBL0_2, //BRI
		VIDEO_COLOR_ADJTBL1_2, //CON
		VIDEO_COLOR_ADJTBL1_2, //SAT
	};
	const unsigned char COLOR_TBL[HDTVI_COLOR_LST] = {0x13, 0x10, 0x11, 0x12};

	if(0 != HDTVI_ChipGetMap(Chnl, &tmpRealChip, &tmpRealChnl, &tmpRealAddr)) {
		return -1;
	}

	I2C_InLock();
	HDDVR_i2c_WriteByte(tmpRealAddr, 0x40, tmpRealChnl);
	if(nHue >= 0 && nHue <= VIDEO_MAX_COLOR_GAIN){
		HDDVR_i2c_WriteByte(tmpRealAddr,
					COLOR_TBL[HDTVI_COLOR_HUE],
					COLOR_MAP[HDTVI_COLOR_HUE][nHue]);
	}
	if(nBrightness >= 0 && nBrightness <= VIDEO_MAX_COLOR_GAIN){
		HDDVR_i2c_WriteByte(tmpRealAddr,
					COLOR_TBL[HDTVI_COLOR_BRI],
					COLOR_MAP[HDTVI_COLOR_BRI][nBrightness]);
	}
	if(nContrast >= 0 && nContrast <= VIDEO_MAX_COLOR_GAIN){
		HDDVR_i2c_WriteByte(tmpRealAddr,
					COLOR_TBL[HDTVI_COLOR_CON],
					COLOR_MAP[HDTVI_COLOR_CON][nContrast]);
	}
	if(nSaturation >= 0 && nSaturation <= VIDEO_MAX_COLOR_GAIN){
		HDDVR_i2c_WriteByte(tmpRealAddr,
					COLOR_TBL[HDTVI_COLOR_SAT],
					COLOR_MAP[HDTVI_COLOR_SAT][nSaturation]);
	}
	I2C_UnLock();

	return 0;
}

char * HDTVI_GetCHNLType(int Chnl)
{
	switch(HDTVI_InputFormatCheck(Chnl)) {
	default:
	case HDVIDEO_UNKNOWN:
	return "UNKNOWN";

	case HDVIDEO_SD720H25FPS:
	case HDVIDEO_SD960H25FPS:
	return "SD-PAL";
	case HDVIDEO_SD720H30FPS:
	case HDVIDEO_SD960H30FPS:
	return "SD-NTSC";

	case HDVIDEO_HD720P25FPS:
	return "TVI-720P@25FPS";
	case HDVIDEO_HD720P30FPS:
	return "TVI-720P@30FPS";

	case HDVIDEO_HD720P50FPS:
	return "TVI-720P@50FPS";
	case HDVIDEO_HD720P60FPS:
	return "TVI-720P@60FPS";

	case HDVIDEO_HD1080P25FPS:
	return "TVI-1080P@25FPS";
	case HDVIDEO_HD1080P30FPS:
	return "TVI-1080P@30FPS";

	case HDVIDEO_SD960H:
	return "CVBS";
	case HDVIDEO_HD720P:
	return "TVI-720P";
	case HDVIDEO_HD1080P:
	return "TVI-1080P";
	}

	return "UNKNOWN";
}

int HDTVI_VSupportCheck(void)
{
	return HDDVR_VSUPPORT_HD1080P;
}

int HDTVI_AudioLiveLoop(int Chnl)
{
	if(Chnl >= 16) {
		Chnl = 0;  //set value for no audio output.
	}
	else {
		Chnl += 1;
	}

	I2C_InLock();
	tp28xx_set_reg_page(HDTVI_ADDR[0], TP28XX_PAGE_AALL);
	HDDVR_i2c_WriteByte(HDTVI_ADDR[0], 0x1A, Chnl);
	I2C_UnLock();

	return 0;
}

int HDTVI_AudioVolume(int Chnl, int Vol)
{
	if(Vol >= 16) {
		Vol = 15;
	}

	I2C_InLock();
	tp28xx_set_reg_page(HDTVI_ADDR[0], TP28XX_PAGE_AALL);
	HDDVR_i2c_WriteByte(HDTVI_ADDR[0], 0x38, (Vol<<0));
	I2C_UnLock();

	return 0;
}

int HDTVI_AudioPlayBack(int Chnl)
{
	I2C_InLock();
	tp28xx_set_reg_page(HDTVI_ADDR[0], TP28XX_PAGE_AALL);
	HDDVR_i2c_WriteByte(HDTVI_ADDR[0], 0x1A, 0x15);
	I2C_UnLock();

	return 0;
}

PacketPTZData( PTZ_packet *ptz)
{
  ptz->header = 0xb5;
  ptz->addr   = 0x00;
  ptz->sum    = ptz->header + ptz->addr + ptz->cmd + ptz->data[0] + ptz->data[1] + ptz->data[2] + ptz->data[3];
}

int ar[18]={0x06,0x07,0x09,0x08,0x0F,0x0E,0x10,0x11,0x12,0x13,0x14,0x0A,0x0B,0x0C,0x0D,0x15,0x16,0x17};
PTZ_packet HandleCommand(int cm)
{
    int cmd = ar[cm];
	printf("......... cmd = %x .......................\n", cmd);

    switch (cm) {
	case HDDVR_UTC_CMD_UP:
	case HDDVR_UTC_CMD_DOWN:
		ptz.cmd = cmd;
		ptz.data[0] = tilt_speed;
		ptz.data[1] = 0x00;
		ptz.data[2] = 0x00;
		ptz.data[3] = 0x00;
		break;

	case HDDVR_UTC_CMD_LEFT:
	case HDDVR_UTC_CMD_RIGHT:
		ptz.cmd = cmd;
		ptz.data[0] = 0x00;
		ptz.data[1] = pan_speed;
		ptz.data[2] = 0x00;
		ptz.data[3] = 0x00;
		break;

	case HDDVR_UTC_CMD_LEFT_UP:
	case HDDVR_UTC_CMD_LEFT_DOWN:
	case HDDVR_UTC_CMD_RIGHT_UP:
	case HDDVR_UTC_CMD_RIGHT_DOWN:
		ptz.cmd = cmd;
		ptz.data[0] = tilt_speed;
		ptz.data[1] = pan_speed;
		ptz.data[2] = 0x00;
		ptz.data[3] = 0x00;
		break;

	case HDDVR_UTC_CMD_PRESET_SET:
	case HDDVR_UTC_CMD_PRESET_CLR:
	case HDDVR_UTC_CMD_PRESET_CALL:
		ptz.cmd = cmd;
		ptz.data[0] = presetNUM;
		ptz.data[1] = 0x00;
		ptz.data[2] = 0x00;
		ptz.data[3] = 0x00;
		break;

	case HDDVR_UTC_CMD_IRIS_CLOSE:
	case HDDVR_UTC_CMD_IRIS_OPEN:
	case HDDVR_UTC_CMD_FOCUS_NEAR:
	case HDDVR_UTC_CMD_FOCUS_FAR:
	case HDDVR_UTC_CMD_ZOOM_WIDE:
	case HDDVR_UTC_CMD_ZOOM_TELE:
	case HDDVR_UTC_CMD_MOTOR_STOP:
		ptz.cmd = cmd;
		ptz.data[0] = 0x00;
		ptz.data[1] = 0x00;
		ptz.data[2] = 0x00;
		ptz.data[3] = 0x00;
		break;

	default:
		printf("ERROR: Invalid command detected!\n");
		break;
    }

    PacketPTZData(&ptz);
	return ptz;
}

int HDTVI_UTC_Init(int chnl)
{
	int addr, ii;
	
	for(ii = 0; ii < ARRAY_SIZE(HDTVI_ADDR); ii ++) {
		int addr = HDTVI_ADDR[ii];
		//bank 1 for TX/RX 3,4
    	HDDVR_i2c_WriteByte(addr, 0x40, 0x10);
    	tp28xx_ptz_init(addr);
    	//bank 0 for TX/RX 1,2
    	HDDVR_i2c_WriteByte(addr, 0x40, 0x04);
    	tp28xx_ptz_init(addr);
    	HDDVR_i2c_WriteByte(addr, 0x7E, 0x0F);   //TX channel enable
    	HDDVR_i2c_WriteByte(addr, 0xB9, 0x0F);   //RX channel enable
	}
	return 0;
}

int HDTVI_UTC_Send(int Chnl, int cmd, unsigned char value)
{
	PTZ_packet ptzCmd;

	int tmpRealChip;
	int tmpRealChnl;
	int tmpRealAddr;
	
	if(0 != HDTVI_ChipGetMap(Chnl, &tmpRealChip, &tmpRealChnl, &tmpRealAddr)) {
		return -1;
	}
	
	I2C_InLock();
	ptzCmd = HandleCommand(cmd);
	tp28xx_ptz_sendcmd(tmpRealAddr, tmpRealChnl, ptzCmd, value);
	I2C_UnLock();
	
	usleep(35*1000);
	
	I2C_InLock();
	ptzCmd = HandleCommand(10);
	tp28xx_ptz_sendcmd(tmpRealAddr, tmpRealChnl, ptzCmd, value);
	I2C_UnLock();

	return 0;
}

int HDTVI_UTC_Exit()
{
	return 0;
}

