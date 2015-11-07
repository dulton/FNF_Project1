#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/string.h>
#include <linux/list.h>
#include <asm/delay.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>

//#include "gpio_i2c.h"
#include "video.h"
#include "coax_protocol.h"
#include "motion.h"
#include "common.h"
#include "audio.h"

#define DRIVER_VER "0.6.0(10.12)"

#ifdef HI_I2C   // 2102/10/11 by lkf72840 ��֮ǰ�� && ����� �޸�Ϊ &  , ��� I2C �� 9022 �����������
unsigned char __I2CReadByte8(unsigned char devaddress, unsigned char address)
{
    return (HI_I2C_Read(devaddress, address, 1, 1) & 0xff);
}

void __I2CWriteByte8(unsigned char devaddress, unsigned char address, unsigned char data)
{
    HI_I2C_Write(devaddress, address, 1, data, 1);
	udelay(200);
}

#endif


#define HI_CHIPID_BASE 0x20050000
#define HI_CHIPID0 IO_ADDRESS(HI_CHIPID_BASE + 0xEEC)
#define HI_CHIPID1 IO_ADDRESS(HI_CHIPID_BASE + 0xEE8)
#define HW_REG(reg)         *((volatile unsigned int *)(reg))

int g_soc_chiptype=0x3520;
int chip_id[4];
int rev_id[4];
unsigned int nvp6124_mode = PAL;  //0:ntsc, 1: pal
unsigned int nvp6124_cnt = 0;
unsigned int nvp6124_slave_addr[4] = {0x60, 0x62, 0x64, 0x66};
module_param(nvp6124_mode, uint, S_IRUGO);
struct semaphore nvp6124_lock;
extern unsigned char ch_mode_status[16];
extern unsigned char ch_vfmt_status[16];
extern unsigned char acp_isp_wr_en[16];

extern void nvp6124_datareverse(void);

int check_rev(unsigned int dec)
{
	int ret;
	gpio_i2c_write(dec, 0xFF, 0x00);
	ret = gpio_i2c_read(dec, 0xf5);
	return ret;
}

int check_id(unsigned int dec)
{
	int ret;
	gpio_i2c_write(dec, 0xFF, 0x00);
	ret = gpio_i2c_read(dec, 0xf4);
	return ret;
}


int nvp6124_open(struct inode * inode, struct file * file)
{
	printk("open! succeed\n");
	return 0;
} 

int nvp6124_close(struct inode * inode, struct file * file)
{
	return 0;
}

int get_hichipid(void)
{
	g_soc_chiptype = (HW_REG(HI_CHIPID0)&0xFF)<<8 | (HW_REG(HI_CHIPID1)&0xFF);
	printk("g_soc_chiptype ==> %x\n", g_soc_chiptype);
	return g_soc_chiptype;
}
unsigned char g_coax_ch;
unsigned int vloss=0xFFFF;
long nvp6124_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int __user *argp = (unsigned int __user *)arg;	
	unsigned int on;
	unsigned char i;
	unsigned long ptz_ch;
	nvp6124_opt_mode optmode;
	unsigned int sens[16];
	nvp6124_video_mode vmode;
	nvp6124_chn_mode schnmode;
	nvp6124_video_adjust v_adj;
	nvp6124_input_videofmt vfmt;
	//static unsigned char vmodechangeflag=0;

	switch (cmd)
	{
		case IOC_VDEC_GET_VIDEO_LOSS:
			down(&nvp6124_lock);	
			vloss = nvp6124_getvideoloss();
			up(&nvp6124_lock);
			if(copy_to_user(argp, &vloss, sizeof(unsigned int)))
				printk("IOC_VDEC_GET_VIDEO_LOSS error\n");
			break;
		case IOC_VDEC_SET_EQUALIZER:
			down(&nvp6124_lock);
			for(i=0;i<nvp6124_cnt*4;i++)
			{
				if(chip_id[i/4] == NVP6124B_R0_ID)
					nvp6124b_set_equalizer(i);
				else
					nvp6124_set_equalizer(i);
			}
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_GET_DRIVERVER:			
			if(copy_to_user(argp, &DRIVER_VER, sizeof(DRIVER_VER)))
				printk("IOC_VDEC_GET_DRIVERVER error\n");
			break;
		case IOC_VDEC_ACP_WRITE:
			down(&nvp6124_lock);			
			for(i=0;i<nvp6124_cnt*4;i++)
			{
				if(acp_isp_wr_en[i] == 0x01)
				{
					acp_isp_wr_en[i] = 0;
					acp_sw_ver_transfer(i);
				}
			}
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_PTZ_ACP_READ:
			if (copy_from_user(&vfmt, argp, sizeof(nvp6124_input_videofmt)))
				return -1;
			down(&nvp6124_lock);
			for(i=0;i<(4*nvp6124_cnt);i++)
			{
				if(   	((ch_mode_status[i] == NVP6124_VI_720P_2530) 	|| 
						(ch_mode_status[i] == NVP6124_VI_HDEX)			||
						(ch_mode_status[i] == NVP6124_VI_720P_5060 		||
						(ch_mode_status[i] == NVP6124_VI_1080P_2530)	)) )
				{
					acp_each_setting(i);
					init_acp_reg_wr(i);
					init_acp_camera_status(i);									
					if(read_acp_status(i)!=0x55)
					{
						init_acp_camera_status(i);
					}
					acp_read(&vfmt, i);
				}
			}							
			up(&nvp6124_lock);
			if(copy_to_user(argp, &vfmt, sizeof(nvp6124_input_videofmt)))
				printk("IOC_VDEC_PTZ_ACP_READ error\n");			
			break;	
		case IOC_VDEC_GET_INPUT_VIDEO_FMT:
			if (copy_from_user(&vfmt, argp, sizeof(nvp6124_input_videofmt)))
				return -1;
			down(&nvp6124_lock);
			video_fmt_det(&vfmt);
			up(&nvp6124_lock);
			if(copy_to_user(argp, &vfmt, sizeof(nvp6124_input_videofmt)))
				printk("IOC_VDEC_GET_INPUT_VIDEO_FMT error\n");
			break;	
        case IOC_VDEC_SET_VIDEO_MODE:
            if (copy_from_user(&vmode, argp, sizeof(nvp6124_video_mode)))
				return -1;
			down(&nvp6124_lock);
			
			nvp6124_mode = vmode.mode;
			
			#if 0  //for nextchip demoboard test only
			if(nvp6124_mode==34 || nvp6124_mode==35)
			{
				for(i=0;i<nvp6124_cnt*4;i++)
				{
					if(0 == nvp6124_set_chnmode(i, nvp6124_mode%2, NVP6124_VI_720P_2530))
						printk("nvp6124_set_chnmode[%d] OK\n", i);
				}
				for(i=0;i<nvp6124_cnt;i++)
				{
					if(chip_id[i] == NVP6124_R0_ID)
					{
						nvp6124_set_portmode(i, 2, NVP6124_OUTMODE_4MUX_HD_X, 0);
					}
					else if(chip_id[i] == NVP6114A_R0_ID)
					{
						nvp6114a_set_portmode(i, 1, NVP6124_OUTMODE_4MUX_HD_X, 0);
					}
					else if(chip_id[i] == NVP6124B_R0_ID)
					{
						nvp6124b_set_portmode(i, 1, NVP6124_OUTMODE_4MUX_HD_X, 0);
					}
				}
			}
			else if(nvp6124_mode==36 || nvp6124_mode==37)
			{
				for(i=0;i<nvp6124_cnt*4;i++)
				{
					if(0 == nvp6124_set_chnmode(i, nvp6124_mode%2, NVP6124_VI_1080P_2530))
						printk("nvp6124_set_chnmode[%d] OK\n", i);
				}
				for(i=0;i<nvp6124_cnt;i++)
				{
					if(chip_id[i] == NVP6124_R0_ID)
					{
						nvp6124_set_portmode(i, 2, NVP6124_OUTMODE_2MUX_FHD_X, 0);
						nvp6124_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_FHD_X, 1);
					}
					else if(chip_id[i] == NVP6114A_R0_ID)
					{
						nvp6114a_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_FHD_X, 0);
						nvp6114a_set_portmode(i, 0, NVP6124_OUTMODE_2MUX_FHD_X, 1);	
					}
					else if(chip_id[i] == NVP6124B_R0_ID)
					{
						nvp6124b_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_FHD_X, 0);
						nvp6124b_set_portmode(i, 0, NVP6124_OUTMODE_2MUX_FHD_X, 1);	
					}
				}
			}
			else if(nvp6124_mode==40 || nvp6124_mode==41)
			{
				for(i=0;i<nvp6124_cnt*4;i++)
				{
					if(0 == nvp6124_set_chnmode(i, nvp6124_mode%2, NVP6124_VI_SD))
						printk("nvp6124_set_chnmode[%d] OK\n", i);
				}
				for(i=0;i<nvp6124_cnt;i++)
				{
					if(chip_id[i] == NVP6124_R0_ID)
					{
						nvp6124_set_portmode(i, 2, NVP6124_OUTMODE_2MUX_SD, 0);
						nvp6124_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_SD, 1);
					}
					else if(chip_id[i] == NVP6114A_R0_ID)
					{
						nvp6114a_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_SD, 0);
						nvp6114a_set_portmode(i, 0, NVP6124_OUTMODE_2MUX_SD, 1);
					}
					else if(chip_id[i] == NVP6124B_R0_ID)
					{
						nvp6124b_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_SD, 0);
						nvp6124b_set_portmode(i, 0, NVP6124_OUTMODE_2MUX_SD, 1);
					}
				}
			}	
			else if(nvp6124_mode==42 || nvp6124_mode==43)
			{
				for(i=0;i<nvp6124_cnt*4;i++)
				{
					if(0 == nvp6124_set_chnmode(i, nvp6124_mode%2, NVP6124_VI_720P_2530))
						printk("nvp6124_set_chnmode[%d] OK\n", i);
				}
				for(i=0;i<nvp6124_cnt;i++)
				{
					if(chip_id[i] == NVP6124_R0_ID)
					{
						nvp6124_set_portmode(i, 2, NVP6124_OUTMODE_2MUX_HD, 0);
						nvp6124_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_HD, 1);
					}
					else if(chip_id[i] == NVP6114A_R0_ID)
					{
						nvp6114a_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_HD, 0);
						nvp6114a_set_portmode(i, 0, NVP6124_OUTMODE_2MUX_HD, 1);
					}
					else if(chip_id[i] == NVP6124B_R0_ID)
					{
						nvp6124b_set_portmode(i, 1, NVP6124_OUTMODE_2MUX_HD, 0);
						nvp6124b_set_portmode(i, 0, NVP6124_OUTMODE_2MUX_HD, 1);
					}
				}
			}
			else if(nvp6124_mode==44 || nvp6124_mode==45)
			{
				for(i=0;i<nvp6124_cnt*4;i++)
				{
					vmode.vformat[0] = nvp6124_mode%2;
					vmode.chmode[i] = NVP6124_VI_720P_5060; 
				}
				nvp6124_each_mode_setting(&vmode);
				for(i=0;i<nvp6124_cnt*4;i++)
				{
					if(0 == nvp6124_set_chnmode(i, nvp6124_mode%2, NVP6124_VI_720P_5060))
						printk("nvp6124_set_chnmode[%d] OK\n", i);
				}
				for(i=0;i<nvp6124_cnt;i++)
				{
					if(chip_id[i] == NVP6124_R0_ID)
					{
						nvp6124_set_portmode(i, 0, NVP6124_OUTMODE_1MUX_HD5060, 0);
						nvp6124_set_portmode(i, 1, NVP6124_OUTMODE_1MUX_HD5060, 1);
						nvp6124_set_portmode(i, 2, NVP6124_OUTMODE_1MUX_HD5060, 2);
						nvp6124_set_portmode(i, 3, NVP6124_OUTMODE_1MUX_HD5060, 3);
					}
					else if(chip_id[i] == NVP6114A_R0_ID)
					{
						nvp6114a_set_portmode(i, 0, NVP6124_OUTMODE_1MUX_HD5060, 0);
						nvp6114a_set_portmode(i, 1, NVP6124_OUTMODE_1MUX_HD5060, 1);
					}
					else if(chip_id[i] == NVP6124B_R0_ID)
					{
						nvp6124b_set_portmode(i, 0, NVP6124_OUTMODE_1MUX_HD5060, 0);
						nvp6124b_set_portmode(i, 1, NVP6124_OUTMODE_1MUX_HD5060, 1);
					}
				}
			}
			else
			#endif	
				nvp6124_each_mode_setting(&vmode);
			
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_SET_CHNMODE:
            if (copy_from_user(&schnmode, argp, sizeof(nvp6124_chn_mode))) return -1;
			down(&nvp6124_lock);
			if(0 == nvp6124_set_chnmode(schnmode.ch, schnmode.vformat, schnmode.chmode))
				printk("IOC_VDEC_SET_CHNMODE OK\n");
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_SET_OUTPORTMODE:
            if(copy_from_user(&optmode, argp, sizeof(nvp6124_opt_mode))) return -1;
			down(&nvp6124_lock);
			if(chip_id[optmode.chipsel] == NVP6124_R0_ID)
				nvp6124_set_portmode(optmode.chipsel, optmode.portsel, optmode.portmode, optmode.chid);
			else if(chip_id[optmode.chipsel] == NVP6114A_R0_ID)
				nvp6114a_set_portmode(optmode.chipsel, optmode.portsel, optmode.portmode, optmode.chid);
			else if(chip_id[optmode.chipsel] == NVP6124B_R0_ID)
				nvp6124b_set_portmode(optmode.chipsel, optmode.portsel, optmode.portmode, optmode.chid);
			up(&nvp6124_lock);
			break;	
		case IOC_VDEC_SET_BRIGHTNESS:
            if(copy_from_user(&v_adj, argp, sizeof(nvp6124_video_adjust))) return -1;
			down(&nvp6124_lock);
			nvp6124_video_set_brightness(v_adj.ch, v_adj.value, ch_vfmt_status[v_adj.ch]);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_SET_CONTRAST:
			if(copy_from_user(&v_adj, argp, sizeof(nvp6124_video_adjust))) return -1;
			down(&nvp6124_lock);
			nvp6124_video_set_contrast(v_adj.ch, v_adj.value, ch_vfmt_status[v_adj.ch]);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_SET_HUE:
			if(copy_from_user(&v_adj, argp, sizeof(nvp6124_video_adjust))) return -1;
			down(&nvp6124_lock);
			nvp6124_video_set_hue(v_adj.ch, v_adj.value, ch_vfmt_status[v_adj.ch]);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_SET_SATURATION:
			if(copy_from_user(&v_adj, argp, sizeof(nvp6124_video_adjust))) return -1;
			down(&nvp6124_lock);
			nvp6124_video_set_saturation(v_adj.ch, v_adj.value, ch_vfmt_status[v_adj.ch]);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_SET_SHARPNESS:
			if(copy_from_user(&v_adj, argp, sizeof(nvp6124_video_adjust))) return -1;
			down(&nvp6124_lock);
			nvp6124_video_set_sharpness(v_adj.ch, v_adj.value);
			up(&nvp6124_lock);
			break; 
		case IOC_VDEC_PTZ_PELCO_INIT:
			down(&nvp6124_lock);
			//nvp6124_pelco_coax_mode();
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_PTZ_PELCO_RESET:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_RESET);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_PTZ_PELCO_SET:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_SET);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_PTZ_CHANNEL_SEL:
            if (copy_from_user(&ptz_ch, argp, sizeof(ptz_ch)))
			{
				return -1;
			}
			down(&nvp6124_lock);
			g_coax_ch = ptz_ch;
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_PTZ_PELCO_UP:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_UP);
			up(&nvp6124_lock);
		 	break;
		case IOC_VDEC_PTZ_PELCO_DOWN:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_DOWN);
			up(&nvp6124_lock);
		 	break;
		case IOC_VDEC_PTZ_PELCO_LEFT:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_LEFT);
			up(&nvp6124_lock);
		 	break;
		case IOC_VDEC_PTZ_PELCO_RIGHT:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_RIGHT);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_PTZ_PELCO_FOCUS_NEAR:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_FOCUS_NEAR);
			up(&nvp6124_lock);
		 	break;
		case IOC_VDEC_PTZ_PELCO_FOCUS_FAR:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_FOCUS_FAR);
			up(&nvp6124_lock);
		 	break;
		case IOC_VDEC_PTZ_PELCO_ZOOM_WIDE:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_ZOOM_WIDE);
			up(&nvp6124_lock);
		 	break;
		case IOC_VDEC_PTZ_PELCO_ZOOM_TELE:
			down(&nvp6124_lock);
			nvp6124_pelco_command(g_coax_ch, PELCO_CMD_ZOOM_TELE);
			up(&nvp6124_lock);
			break;

		case IOC_VDEC_INIT_MOTION:
			down(&nvp6124_lock);
			nvp6124_motion_init();
			hi3520_init_blank_data(0);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_ENABLE_MOTION:
			break;
		case IOC_VDEC_DISABLE_MOTION:
			break;
		case IOC_VDEC_SET_MOTION_AREA:
			break;
		case IOC_VDEC_GET_MOTION_INFO:
			nvp6124_get_motion_info(0);
			break;
		case IOC_VDEC_SET_MOTION_DISPLAY:
            if(copy_from_user(&on, argp, sizeof(unsigned int))) return -1;
			down(&nvp6124_lock);
			nvp6124_motion_display(0,on);
			up(&nvp6124_lock);
			break;
		case IOC_VDEC_SET_MOTION_SENS:
            if(copy_from_user(&sens, argp, sizeof(unsigned int)*16)) return -1;
			down(&nvp6124_lock);
			nvp6124_motion_sensitivity(sens);
			up(&nvp6124_lock);
			break;
		default:
            //printk("drv:invalid nc decoder ioctl cmd[%x]\n", cmd);
			break;
	}
	return 0;
}

static struct file_operations nvp6124_fops = {
	.owner      = THIS_MODULE,
    .unlocked_ioctl	= nvp6124_ioctl,
	.open       = nvp6124_open,
	.release    = nvp6124_close
};

static struct miscdevice nvp6124_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "nc_vdec",
	.fops  		= &nvp6124_fops,
};

static int __init nvp6124_module_init(void)
{
	int ret = 0;
	int ch = 0;
	int chip = 0;
	
	ret = misc_register(&nvp6124_dev);

   	if (ret)
	{
		printk("ERROR: could not register AHD2.0 devices:%#x \n",ret);		
	}
	
	printk("AHD2.0 EXT Driver %s\n", DRIVER_VER);
	
	for(chip=0;chip<4;chip++)
	{
		chip_id[chip] = check_id(nvp6124_slave_addr[chip]);
		rev_id[chip]  = check_rev(nvp6124_slave_addr[chip]);
		if( (chip_id[chip] != NVP6124_R0_ID )  && 
			(chip_id[chip] != NVP6114A_R0_ID ) && 
			(chip_id[chip] != NVP6124B_R0_ID) )
		{
			printk("AHD2.0 Device ID Error... %x\n", chip_id[chip]);
		}
		else
		{
			printk("AHD2.0 Device (0x%x) ID OK... %x\n", nvp6124_slave_addr[chip], chip_id[chip]);
			printk("AHD2.0 Device (0x%x) REV ... %x\n", nvp6124_slave_addr[chip], rev_id[chip]);
			nvp6124_slave_addr[nvp6124_cnt] = nvp6124_slave_addr[chip];	
			chip_id[nvp6124_cnt] = chip_id[chip];
			rev_id[nvp6124_cnt]  = rev_id[chip];
			nvp6124_cnt++;
		}
	}
	
	printk("AHD2.0 Count = %x\n", nvp6124_cnt);
	
	sema_init(&nvp6124_lock, 1);
	nvp6124_common_init();

	for(ch=0;ch<nvp6124_cnt*4;ch++)
	{
		nvp6124_set_chnmode(ch, PAL, NVP6124_VI_1080P_2530);   //Ĭ������Ϊ1080Pģʽ.
	}
	for(chip=0;chip<nvp6124_cnt;chip++)
	{
		if(chip_id[chip] == NVP6124_R0_ID)
		{
			nvp6124_set_portmode(chip, 0, NVP6124_OUTMODE_1MUX_FHD, 2);  //����nvp6124 4��port�����ģʽ
			nvp6124_set_portmode(chip, 1, NVP6124_OUTMODE_1MUX_FHD, 3);
			nvp6124_set_portmode(chip, 2, NVP6124_OUTMODE_1MUX_FHD, 0);
			nvp6124_set_portmode(chip, 3, NVP6124_OUTMODE_1MUX_FHD, 1);
		}
		else if(chip_id[chip] == NVP6114A_R0_ID)
		{
			nvp6114a_set_portmode(chip, 0, NVP6124_OUTMODE_1MUX_FHD, 0);  //����nvp6114a 2��port�����ģʽ
			nvp6114a_set_portmode(chip, 1, NVP6124_OUTMODE_1MUX_FHD, 1);
		}
		else if(chip_id[chip] == NVP6124B_R0_ID)
		{
			nvp6124b_set_portmode(chip, 0, NVP6124_OUTMODE_2MUX_FHD, 0);  //����nvp6124b 2��port�����ģʽ
			nvp6124b_set_portmode(chip, 1, NVP6124_OUTMODE_2MUX_FHD, 1);
		}
	}
	
	audio_init(1,0,16,0,0);
	return 0;
}



static void __exit nvp6124_module_exit(void)
{
	misc_deregister(&nvp6124_dev);	
}

module_init(nvp6124_module_init);
module_exit(nvp6124_module_exit);

MODULE_LICENSE("GPL");

