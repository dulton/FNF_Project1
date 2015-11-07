#ifndef _NETSTREAMIPC_H_
#define _NETSTREAMIPC_H_

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>

#include <conf.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SHM_BUF_CAP = 0, //Զ�̼࿴
//    SHM_BUF_PLB,     //Զ�̻ط�
    SHM_BUF_MOB,     //�ֻ�ͨ��
    SHM_BUF_NUM
} SHM_BUF_PTR;       //������

#if 0
#ifdef MAX_CAM_CH
#define SHM_BUF_NUM   MAX_CAM_CH
#else
#define SHM_BUF_NUM  (8)
#endif
#endif

#ifdef _NVR
#define SHM_BUF_SIZE (2*1024*1024) //��������С
#define MAX_PKG_SIZE (SHM_BUF_SIZE/8) //������ݰ���С
#else
#define SHM_BUF_SIZE (1*1024*1024) //��������С
#define MAX_PKG_SIZE (SHM_BUF_SIZE/10) //������ݰ���С
#endif
#define SHM_KEY_FILE "/tmp"

#ifdef __LITTLE_ENDIAN
#define _SHM_PKG_HEAD_MAGIC ( (unsigned long)'_' << 0 \
							| (unsigned long)'S' << 8 \
							| (unsigned long)'H' << 16 \
							| (unsigned long)'M' << 24 )
#else
#define _SHM_PKG_HEAD_MAGIC ( (unsigned long)'_' << 24 \
							| (unsigned long)'S' << 16 \
							| (unsigned long)'H' << 8 \
							| (unsigned long)'M' << 0 )
#endif

#pragma pack(4)
typedef struct {
	int                nChannel;      //���ݰ�Ŀ��ͨ����
	char               FrmType[4];  //���ݰ�����
	unsigned long long TimeStmp;     //ʱ���
} _SHM_PKG_HEAD_EXT;

typedef struct {
	unsigned long nDataLen; //���ݰ���С(��������ͷ)
	_SHM_PKG_HEAD_EXT Ext;
	unsigned long MagicNum; //��ͷ���
} _SHM_PKG_HEAD;
#pragma pack()

extern int SHM_SrvInit(void);
extern int SHM_SrvFin(void);
extern int SHM_CliInit(void);
extern int SHM_CliFin(void);

extern void Shm_BufForceEmpty(int Buf_Ptr);

extern int Shm_WritePkg(unsigned int TgtNo, _SHM_PKG_HEAD * InPkgHead);
extern int Shm_ReadPkg (unsigned int SrcNo, _SHM_PKG_HEAD * OutPkgHead);
extern unsigned char * Shm_Request(unsigned int TgtNo, _SHM_PKG_HEAD * InPkgHead);
extern int Shm_Commit(unsigned int TgtNo, _SHM_PKG_HEAD * InPkgHead);
extern unsigned char * Shm_Release(unsigned int SrcNo, _SHM_PKG_HEAD * InPkgHead);
extern int Shm_Update(unsigned int SrcNo, _SHM_PKG_HEAD * InPkgHead);

#ifdef __cplusplus
}
#endif
#endif //_NETSTREAMIPC_H_
