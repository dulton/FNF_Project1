/******************************************************************************
Copyright (c)2013-2023, ZheJiang Dahua Technology Stock CO.LTD.
All Rights Reserved.
******************************************************************************
File Name   : dahua_dh9901.h
Version       : Initial Draft
Author        : 
Created       : 2013/11/29
Last Modified :
Description   : The common defination & API interface
Function List :
History       :
1.Date        : 2013/11/29
2.Author      : 21810
3.Modification: Created file  
******************************************************************************/

#ifndef  __DH9901_API_H__ 
#define __DH9901_API_H__

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


#ifdef MODULE_NAME
#undef MODULE_NAME
#endif
#define MODULE_NAME "dh9901.a"

/*----------------------------------------------*
 * The common data type, will be used in the whole project.*
 *----------------------------------------------*/
typedef unsigned char           DH_U8;
typedef unsigned short          DH_U16;
typedef unsigned int            DH_U32;

typedef signed char             DH_S8;
typedef short                   DH_S16;
typedef int                     DH_S32;

#ifndef _M_IX86
    typedef unsigned long long  DH_U64;
    typedef long long           DH_S64;
#else
    typedef __int64             DH_U64;
    typedef __int64             DH_S64;
#endif

typedef char                    DH_CHAR;
#define DH_VOID                 void

/*----------------------------------------------*
 * const defination                             *
 *----------------------------------------------*/

typedef enum {
    DH_FALSE = 0,
    DH_TRUE  = 1,
} DH_BOOL;

#ifndef NULL
    #define NULL    0L
#endif

#define DH_NULL     0L
#define DH_SUCCESS  0
#define DH_FAILURE  (-1)

/*----------------------------------------------*
 *                           ������                 *
 *----------------------------------------------*/
#define DH_DH9901_NOT_INIT       (0xFFFFFFFE)
#define DH_DH9901_INVALID_PARAM  (0xFFFFFFFD)
#define DH_DH9901_NULL_POINTER   (0xFFFFFFFC)


/*----------------------------------------------*
 *             DH9901��ؽṹ����               *
 *----------------------------------------------*/
 
#define DH_CHNS_PER_DH9901     (4)
#define MAX_DH9901_NUM_PER_CPU (4)

typedef struct dh_video_position_s
{
	DH_S32 ucHorizonOffset; // ˮƽƫ��
	DH_S32 ucVerticalOffset;//��ֱƫ��
	DH_S32 res[2];
}DH_VIDEO_POSITION_S;

typedef enum dh_set_mode_e
{
	DH_SET_MODE_DEFAULT = 0, // Ĭ������
	DH_SET_MODE_USER    = 1, // �û�����
	DH_SET_MODE_NONE,
}DH_SET_MODE_E;

typedef struct dh_video_color_s
{
	DH_U8 ucBrightness;	    //< ���ȣ�ȡֵ0-100��
	DH_U8 ucContrast;		//< �Աȶȣ�ȡֵ0-100��
	DH_U8 ucSaturation;	    //< ���Ͷȣ�ȡֵ0-100��
	DH_U8 ucHue;			//< ɫ����ȡֵ0-100��
	DH_U8 ucGain;			//< ���棬������
	DH_U8 ucWhiteBalance;	//< �׵�ƽ,������
	DH_U8 ucSharpness;	    //< ��ȣ�ȡֵ0-15��
	DH_U8 reserved[1];
}DH_VIDEO_COLOR_S;

typedef enum dh_hd_video_format_e
{
	DH_HD_720P_25HZ = 0, // 720P 25֡
	DH_HD_720P_30HZ,
	DH_HD_720P_50HZ,
	DH_HD_720P_60HZ,
	DH_HD_1080P_25HZ, 
	DH_HD_1080P_30HZ,
	DH_HD_DEFAULT,
}DH_HD_VIDEO_FORMAT_E;

/*AD ��Ƶ��ʧ����Ƶ��ʽ״̬*/
typedef struct dh_video_status_s
{
	DH_U8 ucVideoLost; // 1��Ƶ��ʧ��0 ��Ƶ�ָ�
	DH_U8 ucVideoFormat; //������Ƶ��ʽ��ȡֵDH_HD_VIDEO_FORMAT_E
	DH_U8 reserved[2];
}DH_VIDEO_STATUS_S;

typedef struct dh_dh9901_audio_connect_mode_s
{
	DH_BOOL ucCascade;	 /*��Ƶ�Ƿ��� ture / false*/
	DH_U8 ucCascadeNum;  /*������Ŀ1 - 4*/
	DH_U8 ucCascadeStage;/*��������0 - 3*/
}DH_AUDIO_CONNECT_MODE_S;

/*��Ƶ�����ʣ�9901֧��8K(Ĭ��)��16K*/
typedef enum dh_audio_samplerate_e
{
	DH_AUDIO_SAMPLERATE_8k  = 0, //Ĭ��
	DH_AUDIO_SAMPLERATE_16K = 1,
}DH_AUDIO_SAMPLERATE_E;

/*��Ƶʱ������ģʽ*/
typedef enum dh_audio_encclk_mode_e
{
	DH_AUDIO_ENCCLK_MODE_SLAVE  = 0, //Ĭ��
	DH_AUDIO_ENCCLK_MODE_MASTER = 1,
}DH_AUDIO_ENCCLK_MODE_E;

/*��Ƶ�������ģʽ*/
typedef enum dh_audio_sync_mode_e
{
	DH_AUDIO_SYNC_MODE_I2S = 0, //Ĭ��
	DH_AUDIO_SYNC_MODE_DSP = 1,
}DH_AUDIO_SYNC_MODE_E;

/*��Ƶ����ʱ��ģʽ*/
typedef enum dh_audio_encclk_sel_e
{
	DH_AUDIO_ENCCLK_MODE_108M = 0,//Ĭ��
	DH_AUDIO_ENCCLK_MODE_54M  = 1,
	DH_AUDIO_ENCCLK_MODE_27M  = 2, 
}DH_AUDIO_ENCCLK_SEL_E;

/*��Ƶ�������� : (1)���ӷ�ʽ:�����Ǽ���
** (2) ������ (3) ����ģʽ
*/
typedef struct dh_dh9901_audio_attr_s
{
	DH_BOOL bAudioEnable;
	/*����ģʽ �û�������*/
	DH_AUDIO_CONNECT_MODE_S stAudioConnectMode;
	/*DH_SET_MODE_DEFAULTĬ������8K,I2S ��ģʽ��108M*/
	DH_SET_MODE_E enSetAudioMode; //Ĭ�����ã��û�����
	DH_AUDIO_SAMPLERATE_E enAudioSampleRate; // 0:8K (default) 1:16k
	DH_AUDIO_ENCCLK_MODE_E enAudioEncClkMode; //0:slave mode(defualt) 1: master mode
	DH_AUDIO_SYNC_MODE_E enAudioSyncMode;   //0:I2S format(defualt) 1:DSP format
	DH_AUDIO_ENCCLK_SEL_E enAudioEncClkSel;  //ϵͳʱ��0:27M 1:54M 2:108M(defualt)
	DH_U8 reserved[5];
}DH_DH9901_AUDIO_ATTR_S;

/*��Ƶ���ԣ�(1) ͨ��˳��ѡ��
** (2) ������ݸ��÷�ʽ
*/
typedef struct dh_dh9901_video_attr_s
{
	DH_SET_MODE_E enSetVideoMode; //Ĭ�����ã��û�����
	DH_U8 ucChnSequence[DH_CHNS_PER_DH9901]; //ͨ�����ӳ��:0 - 3 
	DH_U8 ucVideoOutMux; // 0: 8bit 4·����(default) 1: 16bit��·���帴��
	DH_U8 reserved[7];
}DH_DH9901_VIDEO_ATTR_S;

typedef struct dh_dh9901_ptz485attr_s
{
	DH_SET_MODE_E enSetVideoMode; //Ĭ�����ã��û�����
	DH_U8 ucProtocolLen; //485 Э�鳤��,Ĭ��7�ֽ�
	DH_U8 reserved[3];
}DH_DH9901_PTZ485_ATTR_S;


/*�ڲ��Լ��߳�����*/
typedef struct dh_auto_detect_attr_s
{
	DH_SET_MODE_E enDetectMode;//Ĭ��ģʽ50ms
	//�ڲ���Ҫ����Ӧ���⣬��������ʱ�����ù���
	DH_U8 ucDetectPeriod;//������ڣ���λms
	DH_U8 reserved[3];
}DH_AUTO_DETECT_ATTR_S;


typedef struct dh_dh9901_attr_s
{
	DH_U8 ucChipAddr; //I2C��ַ

	DH_DH9901_VIDEO_ATTR_S   stVideoAttr;
	DH_DH9901_AUDIO_ATTR_S   stAuioAttr;
	DH_DH9901_PTZ485_ATTR_S  stPtz485Attr;
	DH_U32 reserved[4];
}DH_DH9901_ATTR_S;

typedef struct dh_dh9901_init_attr_s
{
	DH_U8 ucAdCount; // ������AD ����
	
	DH_DH9901_ATTR_S stDh9901Attr[MAX_DH9901_NUM_PER_CPU];// ÿһƬDH9901������Ϣ
	DH_AUTO_DETECT_ATTR_S    stDetectAttr;

	/*��д�Ĵ�����������ʼ��ʱ��Ҫע��*/
	DH_S32 (* Dh9901_WriteByte)(DH_U8 ucChipAddr, DH_U8 ucRegAddr, DH_U8 ucRegValue);
	DH_U8 (* Dh9901_ReadByte)(DH_U8 ucChipAddr, DH_U8 ucRegAddr);

	DH_U32 reserved[4];
}DH_DH9901_INIT_ATTR_S;


///\dh9901��ʼ��
/// 
///\param [in] 9901
///\retval ����ֵ ������
///\retval 0 ��ʼ���ɹ�
DH_S32 DH9901_API_Init(DH_DH9901_INIT_ATTR_S *pDh9901InitAttrs);

///\dh9901ȥ��ʼ��
/// 
///\param [in] 9901
///\retval ����ֵ ������
///\retval 0 ȥ��ʼ���ɹ�
DH_S32 DH9901_API_DeInit(void);

///\dh9901����ͼ��ɫ��
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\param [in] 9901 pVideoColor ɫ�ʲ�����enColorMode ɫ��ģʽ
///\retval ����ֵ ������
///\retval 0 ���óɹ�
DH_S32 DH9901_API_SetColor(DH_U8 ucChipIndex, DH_U8 ucChn, DH_VIDEO_COLOR_S *pVideoColor, DH_SET_MODE_E enColorMode);

///\dh9901��ȡͼ��ɫ��
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\param [in] 9901 pVideoColor ɫ�ʲ�����enColorMode ɫ��ģʽ
///\retval ����ֵ ������
///\retval 0 ��ȡ�ɹ�
DH_S32 DH9901_API_GetColor(DH_U8 ucChipIndex, DH_U8 ucChn, DH_VIDEO_COLOR_S *pVideoColor, DH_SET_MODE_E enColorMode);


///\ dh9901���ͼ�����
///\ ע�� : ͼ���������������ã�
///\��ȷ���غ�ſɽ���ɫ�ʶ�ȡ��
///\param [in]ucChipIndexоƬ������ucChnͨ����
///\retval ����ֵ ������
///\retval 0����ɹ�
DH_S32 DH9901_API_ClearEq(DH_U8 ucChipIndex, DH_U8 ucChn);

///\dh9901 485����ʹ�ܣ�����ͨ������
///\ÿ�β���485ǰʹ�ܣ�������Ϻ�رգ�
///\ע��: ��Ҫ���ͨ��ͬʱʹ��
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\param [in] bEnable DH_TRUE/ DH_FALSE
///\retval ����ֵ ������
///\retval 0 ʹ�ܳɹ�
DH_S32 DH9901_API_Contrl485Enable(DH_U8 ucChipIndex, DH_U8 ucChn, DH_BOOL bEnable);


///\д485buffer
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\param [in] ucBuf ����ָ��ucLenth ���ݳ���
///\retval ����ֵ ������
///\retval 0  д�ɹ�
DH_S32 DH9901_API_Send485Buffer(DH_U8 ucChipIndex, DH_U8 ucChn, DH_U8 *ucBuf, DH_U8 ucLenth);


///\ͼ��ƫ�Ƶ���
///\ ����ǰ�ȶ�ȡ��ǰλ����Ϣ�����ݵ�ǰλ�õ���ƫ��
///\������Χ0 - 15
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\param [in] pVideoPos ƫ�Ʋ���
///\retval ����ֵ ������
///\retval 0 �ɹ�
DH_S32 DH9901_API_SetVideoPosition(DH_U8 ucChipIndex, DH_U8 ucChn, DH_VIDEO_POSITION_S *pVideoPos);

///\��ȡͼ��ƫ��λ����Ϣ
/// 
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\param [in] pVideoPos ƫ�Ʋ���
///\retval ����ֵ ������
///\retval 0 �ɹ�
DH_S32 DH9901_API_GetVideoPosition(DH_U8 ucChipIndex, DH_U8 ucChn, DH_VIDEO_POSITION_S *pVideoPos);

///\����ͬ��������Ƶ����
/// 
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\param [in] ucVolume ����ֵ0 - 100
///\retval -1 
///\retval 0 �ɹ�
DH_S32 DH9901_API_SetAudioInVolume(DH_U8 ucChipIndex, DH_U8 ucChn, DH_U8 ucVolume);

///\ ��ȡADоƬ��ʧ����Ƶ��ʽ
/// 
///\param [in] 9901 ucChipIndexоƬ������ucChnͨ����
///\retval ����ֵ ������
///\retval 0 �ɹ�
DH_S32 DH9901_API_GetVideoStatus(DH_U8 ucChipIndex, DH_U8 ucChn, DH_VIDEO_STATUS_S *pVideoStatus);

///\9901д�Ĵ���
/// 
///\param [in] 9901 ucChipIndexоƬ������ucPage ҳ����
///\param [in] 9901 ucRegAddr�Ĵ���ֵ��pucValue �Ĵ���ֵָ��
///\retval ����ֵ ������
///\retval 0 �ɹ�
DH_S32 DH9901_API_WriteReg(DH_U8 ucChipIndex, DH_U8 ucPage, DH_U8 ucRegAddr, DH_U8 *pucValue);

///\9901���Ĵ���
/// 
///\param [in] 9901 ucChipIndexоƬ������ucPage ҳ����
///\param [in] 9901 ucRegAddr�Ĵ���ֵ��pucValue �Ĵ���ֵָ��
///\retval ����ֵ ������
///\retval 0  ��ȡ�ɹ�
DH_S32 DH9901_API_ReadReg(DH_U8 ucChipIndex, DH_U8 ucPage, DH_U8 ucRegAddr, DH_U8 *pucValue);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif

