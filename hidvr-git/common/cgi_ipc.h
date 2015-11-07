/*
 * cgi_ipc.h
 *
 *  Created on: 2011-11-14
 *      Author: root
 */

#ifndef CGI_IPC_H_
#define CGI_IPC_H_

#define CGI_SEM_PATH "/root/dvr_web/www/cgi-bin/"
#define CGI_SEM_COUNT (2) //�źŵ�����

#define POSTER_CGI_INDEX (0) //poster.cgi�źŵ�����
#define POSTER_CGI_NUM (1) //poster.cgi��Դ����

#define GW_CGI_INDEX (1) //gw.cgi�źŵ�����
#define GW_CGI_NUM (1) //gw.cgi��Դ����


void CGI_IPC_init();
void CGI_IPC_destory();
void CGI_IPC_attach();
void CGI_IPC_P(int _sem_index);
void CGI_IPC_V(int _sem_index);
#endif /* CGI_IPC_H_ */
