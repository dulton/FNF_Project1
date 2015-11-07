#ifndef __FLVLIB_HEADER_FILE__
#define __FLVLIB_HEADER_FILE__
#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************/
/*avlib_flv_write_header ����FLV�ļ�ͷ�����ú����󣬶�ȡpb->buffer,pb->actlenΪ��С*/
/*BufIO *pb  ���뻺��ṹ                                         */
/*width,height ����FLV����Ƶ���Ŀ�͸�*/
/*bit_rate ����FLV����Ƶ��������*/
/*Framerate ����FLV����Ƶ����֡��*/
/******************************************************************/
int avlib_flv_write_header(BufIO *pb,int width,int height,int bit_rate,double framerate,double duration);

/******************************************************************/
/*avlib_flv_write_packet ����FLV��Ƶ����֡�����ú����󣬶�ȡpb->buffer,pb->actlenΪ��С*/
/*BufIO *pb  ���뻺��ṹ                                         */
/*unsigned char *buf H.264��������֡*/
/*int size H.264������֡�Ĵ�С*/
/*int timestamp ʱ��������룩���ۼӵ�ֵ�������һ֡Ϊ0���ڶ�֡Ϊ40������֡Ϊ80..........*/
/*int keyframe �Ƿ�Ϊ�ؼ�֡��1-I frame,0-P-frame*/
/******************************************************************/
int avlib_flv_write_packet(BufIO *pb, unsigned char *buf,int size,int timestamp,int keyframe);

/******************************************************************/
/*avlib_flv_write_audio_packet ����FLV��Ƶ����֡�����ú����󣬶�ȡpb->buffer,pb->actlenΪ��С*/
/*BufIO *pb  ���뻺��ṹ                                         */
/*unsigned char *buf 11025 16bit mono lpcm������֡*/
/*int size ������֡�Ĵ�С*/
/*int timestamp ʱ��������룩���ۼӵ�ֵ�������һ֡Ϊ0���ڶ�֡Ϊ40������֡Ϊ80..........*/
/******************************************************************/
int avlib_flv_write_audio_packet(BufIO *pb, unsigned char *buf,int size, int timestamp, unsigned char conf);

/******************************************************************/
/*avlib_writehttpheader ����HTTPͷ��ͨ��rheader����*/
/*int code http���ش��룬����200ΪOK��400ΪBAD REQUEST*/
/*char *contenttype �����������ͣ�����"image/jpeg"*/
/*int contentsize �������ݵĴ�С����������Ƶ���Ļ���Ӧ����0xffffffff*/
/******************************************************************/
int avlib_writehttpheader(int code,char * contenttype,int contentsize,char *rheader);

/******************************************************************/
/*avlib_httpclientproc ����HTTPͷ��ͨ��rheader��������URL������ͻ�������http://192.168.0.1/1.flv��ô����/1.flv*/
/******************************************************************/
int avlib_httpclientproc(char *rheader,char *requestcmd);

#ifdef __cplusplus
}
#endif
#endif
