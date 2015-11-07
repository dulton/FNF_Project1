/*
 * rec_file_format.h
 *
 *  Created on: 2011-9-27
 *      Author: root
 */

#ifndef REC_FILE_FORMAT_H_
#define REC_FILE_FORMAT_H_

#include "conf.h"
#include "avenc_types.h"

#define FILE_HEADER_MAGIC (0x206f756c) //"luo "
#define FRAME_HEAD_MAGIC (0x2075696c)  //"liu "
#define FILE_TAIL_MAGIC (0x6c756f20) //" oul"
#define FRAME_TAIL_MAGIC (0x6c697520)  //" uil"

typedef enum
{
	REC_TYPE_NONE = 0,
	REC_TYPE_TIMER = 1,
	REC_TYPE_MOTION = 2,
	REC_TYPE_SENSOR = 4,
	REC_TYPE_MANUAL = 8,
}REC_TYPE;

#pragma pack(4)

typedef struct FileHeader
{
	union{
		struct{
			unsigned int nMagic;
			time_t lBeginTime;
			time_t lEndTime;
			int nFirstIDROffset[MAX_REF_CH];
			time_t nFirstIDRGenTime[MAX_REF_CH];
			int nLastIDROffset[MAX_REF_CH];
			time_t nLastIDRGenTime[MAX_REF_CH];
			int nLastSessionID[MAX_REF_CH];
			int nLastSessionRnd[MAX_REF_CH];
			time_t nLastSessionUpdateTime[MAX_REF_CH];
			int nFrameCount[MAX_REF_CH];
			long long nFrameTotalSize[MAX_REF_CH];
		};
		struct
		{
			unsigned int reserved[2048-1];
			unsigned int nMagic2;
		};
	};
}FileHeader;

typedef struct
{
	union
	{
		struct
		{
			unsigned int magic; //ͷ���
			int session_rnd;    //�������
			int frame_width; //�ֱ��� ��
			int frame_height; //�ֱ��� ��
			int frame_rate; //֡�� ��25fps
			int audio_sample_rate; //��Ƶ������
			char audio_format[8]; //��Ƶ��ʽ
			int audio_data_width; //��Ƶ���ݿ�� 8bit/16bit
			AVENC_FRAME_TYPE frame_type; //֡���ͣ�I/P/��Ƶ��
			int session_id; //��ID
			int channel; //ͨ����
			REC_TYPE rec_type; //¼�����ͣ����ֶ�����ʱ��
			unsigned long long frame_index; //֡�ڶ��е�����
			unsigned int nSize; //֡��С��������֡ͷ
			unsigned long long u64TSP;
			unsigned long nGenTime;
			int ref_count; //��֡�ο�
		};
		struct
		{
			unsigned int reserved[32-1];
			unsigned int magic2; //β���
		};
	};
}Frame_Head_t;

#pragma pack()

#define REC_FILE_PATH "/root/rec"

#endif /* REC_FILE_FORMAT_H_ */
