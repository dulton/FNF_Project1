#ifndef __PORT_MANAGE_H__
#define __PORT_MANAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

int PORT_MANAGE_init(unsigned int min,unsigned int max);
int PORT_MANAGE_destroy();
//
int PORT_MANAGE_add_port(unsigned int port);
/*apply 1 port*/
int PORT_MANAGE_apply1_port(unsigned int * const port);
/* apply 2 continuous ports */
int PORT_MANAGE_apply2_port(unsigned int * const port1);
/*
 ���������ӿ�����˿�ʱ��������ʽ���룬ͬʱ���ĳ���˿��ͷź󲢲�
 �����ʹ�� (�������������ӿڵĲ��ԣ����Ǵ���Сֵ����ʼ����)��
*/
int PORT_MANAGE_apply1_port2(unsigned int * const port);
int PORT_MANAGE_apply2_port2(unsigned int * const port1);
//
int PORT_MANAGE_apply1_port3(unsigned int * const port);
int PORT_MANAGE_apply2_port3(unsigned int * const port1);


int PORT_MANAGE_free_port(unsigned int port);
//

#ifdef __cplusplus
}
#endif
#endif


