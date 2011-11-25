
#include "rtl2832u_fe.h"
#include "rtl2832u_io.h"
#include "rtl2832u.h"
#include "rtl2832u_ioctl.h"

extern int demod_default_type;
extern int dvb_use_rtl2832u_card_type;
extern int dvb_use_rtl2832u_rc_mode;
extern int rtl2832u_remote_control_state;

static struct rtl2832_reg_addr rtl2832_reg_map[]= {
	/* RTD2831_RMAP_INDEX_USB_CTRL_BIT5*/			{ RTD2832U_USB, USB_CTRL, 5, 5		},
	/* RTD2831_RMAP_INDEX_USB_STAT*/				{ RTD2832U_USB, USB_STAT, 0, 7		},
	/* RTD2831_RMAP_INDEX_USB_EPA_CTL*/			{ RTD2832U_USB, USB_EPA_CTL, 0, 31	},
	/* RTD2831_RMAP_INDEX_USB_SYSCTL*/				{ RTD2832U_USB, USB_SYSCTL, 0, 31		},
	/* RTD2831_RMAP_INDEX_USB_EPA_CFG*/			{ RTD2832U_USB, USB_EPA_CFG, 0, 31	},
	/* RTD2831_RMAP_INDEX_USB_EPA_MAXPKT*/		{ RTD2832U_USB, USB_EPA_MAXPKT, 0, 31},
	/* RTD2831_RMAP_INDEX_USB_EPA_FIFO_CFG*/		{ RTD2832U_USB, USB_EPA_FIFO_CFG, 0, 31},

	/* RTD2831_RMAP_INDEX_SYS_DEMOD_CTL*/			{ RTD2832U_SYS, DEMOD_CTL, 0, 7	       },
	/* RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL*/	{ RTD2832U_SYS, GPIO_OUTPUT_VAL, 0, 7	},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_EN_BIT3*/{ RTD2832U_SYS, GPIO_OUTPUT_EN, 3, 3	},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_DIR_BIT3*/		{ RTD2832U_SYS, GPIO_DIR, 3, 3		},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_CFG0_BIT67*/	{ RTD2832U_SYS, GPIO_CFG0, 6, 7		},
	/* RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1*/		{ RTD2832U_SYS, DEMOD_CTL1, 0, 7	       },
	/* RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_EN_BIT1*/{ RTD2832U_SYS, GPIO_OUTPUT_EN, 1, 1	},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_DIR_BIT1*/		{ RTD2832U_SYS, GPIO_DIR, 1, 1		},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_EN_BIT6*/{ RTD2832U_SYS, GPIO_OUTPUT_EN, 6, 6	},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_DIR_BIT6*/		{ RTD2832U_SYS, GPIO_DIR, 6, 6		},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_EN_BIT5*/{ RTD2832U_SYS, GPIO_OUTPUT_EN, 5, 5},
	/* RTD2831_RMAP_INDEX_SYS_GPIO_DIR_BIT5*/      { RTD2832U_SYS, GPIO_DIR, 5, 5},

#if 0
	/* RTD2831_RMAP_INDEX_SYS_GPD*/			{ RTD2832U_SYS, GPD, 0, 7		},
	/* RTD2831_RMAP_INDEX_SYS_GPOE*/			{ RTD2832U_SYS, GPOE, 0, 7	},
	/* RTD2831_RMAP_INDEX_SYS_GPO*/			{ RTD2832U_SYS, GPO, 0, 7		},
	/* RTD2831_RMAP_INDEX_SYS_SYS_0*/			{ RTD2832U_SYS, SYS_0, 0, 7	},
#endif

	/* DTMB related */

   
};                                          

static int rtl2832_reg_mask[32]= {
    0x00000001,
    0x00000003,
    0x00000007,
    0x0000000f,
    0x0000001f,
    0x0000003f,
    0x0000007f,
    0x000000ff,
    0x000001ff,
    0x000003ff,
    0x000007ff,
    0x00000fff,
    0x00001fff,
    0x00003fff,
    0x00007fff,
    0x0000ffff,
    0x0001ffff,
    0x0003ffff,
    0x0007ffff,
    0x000fffff,
    0x001fffff,
    0x003fffff,
    0x007fffff,
    0x00ffffff,
    0x01ffffff,
    0x03ffffff,
    0x07ffffff,
    0x0fffffff,
    0x1fffffff,
    0x3fffffff,
    0x7fffffff,
    0xffffffff
};

typedef struct FC0012_LNA_REG_MAP
{
	unsigned char	Lna_regValue;
	long			LnaGain;
}FC0012_LNA_REG_MAP;

FC0012_LNA_REG_MAP  FC0012_LNA_GAIN_TABLE[] =
{
	{0x00 , -63},{0x01 , -58},{0x02 , -99},{0x03 , -73},{0x04 , -63},{0x05 , -65}
	,{0x06 , -54},{0x07 , -60},{0x08 , 71 },{0x09 , 70 },{0x0a , 68 },{0x0b , 67 }
	,{0x0c , 65 },{0x0d , 63 },{0x0e , 61 },{0x0f , 58 },{0x10 , 197},{0x11 , 191}
	,{0x12 , 188},{0x13 , 186},{0x14 , 184},{0x15 , 182},{0x16 , 181},{0x17 , 179}
};



static int
demod_init_setting(
	struct rtl2832_state * p_state);

static int
build_nim_module(
	struct rtl2832_state*  p_state);

static int 
fc0012_get_signal_strength(
	struct rtl2832_state	*p_state,
	unsigned long *strength);

static int 
rtl2832_sleep_mode(
	struct rtl2832_state* p_state);



static void 	
custom_wait_ms(
	BASE_INTERFACE_MODULE*	pBaseInterface,
	unsigned long				WaitTimeMs)
{
	platform_wait(WaitTimeMs);
	return;	
}



static int
custom_i2c_read(
	BASE_INTERFACE_MODULE*	pBaseInterface,
	unsigned char				DeviceAddr,
	unsigned char*				pReadingBytes,
	unsigned long				ByteNum
	)
{
	struct dvb_usb_device *d;

	pBaseInterface->GetUserDefinedDataPointer(pBaseInterface, (void **)&d);
	if ( read_rtl2832_stdi2c( d, DeviceAddr , pReadingBytes , ByteNum ) ) goto error;
	
	return 0;
error:
	return 1;
}



static int
custom_i2c_write(
	BASE_INTERFACE_MODULE*	pBaseInterface,
	unsigned char				DeviceAddr,
	const unsigned char*			pWritingBytes,
	unsigned long				ByteNum)
{
	struct dvb_usb_device *d;

	pBaseInterface->GetUserDefinedDataPointer(pBaseInterface, (void **)&d);
	if ( write_rtl2832_stdi2c( d, DeviceAddr , (unsigned char*)pWritingBytes , ByteNum ) ) goto error;
	
	return 0;
error:
	return 1;
}



static int
read_usb_sys_register(
	struct rtl2832_state*		p_state,
	rtl2832_reg_map_index		reg_map_index,
	int*						p_val)
{
	RegType			reg_type=	rtl2832_reg_map[reg_map_index].reg_type;
	unsigned short	reg_addr=	rtl2832_reg_map[reg_map_index].reg_addr;
	int				bit_low=	rtl2832_reg_map[reg_map_index].bit_low;
	int				bit_high=	rtl2832_reg_map[reg_map_index].bit_high;

	int	n_byte_read=(bit_high>> 3)+ 1;

	*p_val= 0;
	if (read_usb_sys_int_bytes(p_state->d, reg_type, reg_addr, n_byte_read, p_val)) goto error;

	*p_val= ((*p_val>> bit_low) & rtl2832_reg_mask[bit_high- bit_low]);
 
	return 0;

error:
	return 1;
}



static int
write_usb_sys_register(
	struct rtl2832_state*		p_state,
	rtl2832_reg_map_index		reg_map_index,
	int						val_write)
{
	RegType			reg_type=	rtl2832_reg_map[reg_map_index].reg_type;
	unsigned short	reg_addr=	rtl2832_reg_map[reg_map_index].reg_addr;
	int				bit_low=	rtl2832_reg_map[reg_map_index].bit_low;
	int				bit_high=	rtl2832_reg_map[reg_map_index].bit_high;
	
	int	n_byte_write=	(bit_high>> 3)+ 1;
	int	val_read= 0;
	int	new_val_write;

	if (read_usb_sys_int_bytes(p_state->d, reg_type, reg_addr, n_byte_write, &val_read)) goto error;

	new_val_write= (val_read & (~(rtl2832_reg_mask[bit_high- bit_low]<< bit_low))) | (val_write<< bit_low);

	if (write_usb_sys_int_bytes(p_state->d, reg_type, reg_addr, n_byte_write, new_val_write)) goto error;
	return 0;
	
error:
	return 1;
}



static int 
set_tuner_power(
	struct rtl2832_state*	p_state,
	unsigned char			b_gpio4, 
	unsigned char			onoff)
{

	int			data;

	deb_info(" +%s \n", __FUNCTION__);
	
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, &data) ) goto error;		

	if(b_gpio4)
	{
		if(onoff)		data &= ~(BIT4);   //set bit4 to 0
		else			data |= BIT4;		//set bit4 to 1		

	}
	else
	{
		if(onoff)		data &= ~(BIT3);   //set bit3 to 0
		else			data |= BIT3;		//set bit3 to 1		
	}
	
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL,data) ) goto error;

	deb_info(" -%s \n", __FUNCTION__);

	return 0;
error:
	return 1;
}



static int 
set_demod_power(
	struct rtl2832_state*	p_state,
	unsigned char			onoff)
{

	int			data;

	deb_info(" +%s \n", __FUNCTION__);
	
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, &data) ) goto error;		
	if(onoff)		data &= ~(BIT0);   	//set bit0 to 0  low
	else			data |= BIT0;		//set bit0 to 1	 high
	
	//data &= ~(BIT0);   //3 Demod Power always ON => hw issue.	
	
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL,data) ) goto error;

	deb_info(" -%s \n", __FUNCTION__);
	return 0;
error:
	return 1;
}



//3//////// Set GPIO3 "OUT"  => Turn ON/OFF Tuner Power
//3//////// Set GPIO3 "IN"      => Button  Wake UP (USB IF) , NO implement in rtl2832u linux driver
static int 
gpio3_out_setting(
	struct rtl2832_state*	p_state)
{
	int			data;

	deb_info(" +%s \n", __FUNCTION__);

	// GPIO3_PAD Pull-HIGH, BIT76
	data = 2;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_CFG0_BIT67,data) ) goto error;

	// GPO_GPIO3 = 1, GPIO3 output value = 1 
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, &data) ) goto error;		
	data |= BIT3;		//set bit3 to 1
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL,data) ) goto error;

	// GPD_GPIO3=0, GPIO3 output direction
	data = 0;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_DIR_BIT3,data) ) goto error;

	// GPOE_GPIO3=1, GPIO3 output enable
	data = 1;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_EN_BIT3,data) ) goto error;

	//BTN_WAKEUP_DIS = 1
	data = 1;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_CTRL_BIT5,data) ) goto error;

	deb_info(" -%s \n", __FUNCTION__);

	return 0;
error:
	return 1;
}



static int 
usb_epa_fifo_reset(
	struct rtl2832_state*	p_state)
{

	int					data;

	deb_info(" +%s \n", __FUNCTION__);
	
	//3 reset epa fifo:
	//3[9] Reset EPA FIFO
	//3 [5] FIFO Flush,Write 1 to flush the oldest TS packet (a 188 bytes block)

	data = 0x0210;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_CTL,data) ) goto error;

	data = 0xffff;
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_CTL,&data) ) goto error;

	if( (data & 0xffff) != 0x0210)
	{
		deb_info("Write error RTD2831_RMAP_INDEX_USB_EPA_CTL = 0x%x\n",data);
	 	goto error;	
	}

	data=0x0000;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_CTL,data) ) goto error;

	data = 0xffff;
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_CTL,&data) ) goto error;

	if( ( data  & 0xffff) != 0x0000)
	{
		deb_info("Write error RTD2831_RMAP_INDEX_USB_EPA_CTL = 0x%x\n",data);
	 	goto error;	
	}

	deb_info(" -%s \n", __FUNCTION__);

	return 0;

error:
	return 1;

}



static int 
usb_init_bulk_setting(
	struct rtl2832_state*	p_state)
{

	int					data;
	
	deb_info(" +%s \n", __FUNCTION__);
	
	//3 1.FULL packer mode(for bulk)
	//3 2.DMA enable.
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_SYSCTL, &data) ) goto error;

	data &=0xffffff00;
	data |= 0x09;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_SYSCTL, data) ) goto error;

	data=0;
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_SYSCTL, &data) ) goto error;
      
	if((data&0xff)!=0x09)  
	{
		deb_info("Open bulk FULL packet mode error!!\n");
	 	goto error;
	}

	//3check epa config,
	//3[9-8]:00, 1 transaction per microframe
	//3[7]:1, epa enable
	//3[6-5]:10, bulk mode
	//3[4]:1, device to host
	//3[3:0]:0001, endpoint number
	data = 0;
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_CFG, &data) ) goto error;                
	if((data&0x0300)!=0x0000 || (data&0xff)!=0xd1)
	{
		deb_info("Open bulk EPA config error! data=0x%x \n" , data);
	 	goto error;	
	}

	//3 EPA maxsize packet 
	//3 512:highspeedbulk, 64:fullspeedbulk. 
	//3 940:highspeediso,  940:fullspeediso.

	//3 get info :HIGH_SPEED or FULL_SPEED
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_STAT, &data) ) goto error;	
	if(data&0x01)  
	{
		data = 0x00000200;
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_MAXPKT, data) ) goto error;

		data=0;
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_MAXPKT, &data) ) goto error;
	                      
		if((data&0xffff)!=0x0200)
		{
			deb_info("Open bulk EPA max packet size error!\n");
		 	goto error;
		}

		deb_info("HIGH SPEED\n");
	}
	else 
    	{
		data = 0x00000040;
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_MAXPKT, data) ) goto error;

		data=0;
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_MAXPKT, &data) ) goto error;
	                      
		if((data&0xffff)!=0x0200)
		{
			deb_info("Open bulk EPA max packet size error!\n");
		 	goto error;
		}
		
		deb_info("FULL SPEED\n");
	}	

	deb_info(" -%s \n", __FUNCTION__);
	
	return 0;

error:	
	return 1;
}



static int 
usb_init_setting(
	struct rtl2832_state*	p_state)
{

	int					data;

	deb_info(" +%s \n", __FUNCTION__);

	if ( usb_init_bulk_setting(p_state) ) goto error;

	//3 change fifo length of EPA 
	data = 0x00000014;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_FIFO_CFG, data) ) goto error;
	data = 0xcccccccc;
	if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_USB_EPA_FIFO_CFG, &data) ) goto error;
	if( (data & 0xff) != 0x14)
	{
		deb_info("Write error RTD2831_RMAP_INDEX_USB_EPA_FIFO_CFG =0x%x\n",data);
	 	goto error;
	}

	if ( usb_epa_fifo_reset(p_state) ) goto error;

	deb_info(" -%s \n", __FUNCTION__);
	
	return 0;

error: 
	return 1;	
}



static int 
suspend_latch_setting(
	struct rtl2832_state*	p_state,
	unsigned char			resume)
{

	int					data;
	deb_info(" +%s \n", __FUNCTION__);

	if (resume)
	{
		//3 Suspend_latch_en = 0  => Set BIT4 = 0 
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1, &data) ) goto error;		
		data &= (~BIT4);	//set bit4 to 0
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1,data) ) goto error;
	}
	else
	{
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1, &data) ) goto error;		
		data |= BIT4;		//set bit4 to 1
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1,data) ) goto error;
	}

	deb_info(" -%s \n", __FUNCTION__);	

	return 0;
error:
	return 1;

}



//3////// DEMOD_CTL1  => IR Setting , IR wakeup from suspend mode
//3////// if resume =1, resume
//3////// if resume = 0, suspend
static int 
demod_ctl1_setting(
	struct rtl2832_state*	p_state,
	unsigned char			resume)
{

	int					data;

	deb_info(" +%s \n", __FUNCTION__);
	
	if(resume)
	{
		// IR_suspend	
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1, &data) ) goto error;		
		data &= (~BIT2);		//set bit2 to 0
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1,data) ) goto error;

		//Clk_400k
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1, &data) ) goto error;		
		data &= (~BIT3);		//set bit3 to 0
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1,data) ) goto error;
	}
	else
	{
		//Clk_400k
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1, &data) ) goto error;		
		data |= BIT3;		//set bit3 to 1
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1,data) ) goto error;

		// IR_suspend		
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1, &data) ) goto error;		
		data |= BIT2;		//set bit2 to 1
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL1,data) ) goto error;
	}

	deb_info(" -%s \n", __FUNCTION__);
	
	return 0;
error:
	return 1;

}



static int 
demod_ctl_setting(
	struct rtl2832_state*	p_state,
	unsigned char			resume,
	unsigned char               on)
{

	int					data;
	unsigned char			tmp;
	
	deb_info(" +%s \n", __FUNCTION__);
		
	if(resume)
	{
		// PLL setting
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
		data |= BIT7;		//set bit7 to 1
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;

		//2 + Begin LOCK
		// Demod  H/W Reset
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
		data &= (~BIT5);	//set bit5 to 0
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;

		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
		data |= BIT5;		//set bit5 to 1
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;
		
		//3 reset page chache to 0 		
		if ( read_demod_register(p_state->d, RTL2832_DEMOD_ADDR, 0, 1, &tmp, 1 ) ) goto error;	
		//2 -End LOCK

		// delay 5ms
		platform_wait(5);


		if(on)
		{
			// ADC_Q setting on
			if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
			data |= BIT3;		//set bit3 to 1
			if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;

			// ADC_I setting on
			if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
			data |= BIT6;		//set bit3 to 1
			if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;
		}
		else
		{
			// ADC_I setting off
			if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
			data &= (~BIT6);		//set bit3 to 0
			if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;

			// ADC_Q setting off
			if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
			data &= (~BIT3);		//set bit3 to 0
			if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;		
		}
	}
	else
	{

		// ADC_I setting
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
		data &= (~BIT6);		//set bit3 to 0
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;

		// ADC_Q setting
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
		data &= (~BIT3);		//set bit3 to 0
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;

		// PLL setting
		if ( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL, &data) ) goto error;		
		data &= (~BIT7);		//set bit7 to 0
		if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_DEMOD_CTL,data) ) goto error;

	}

	deb_info(" -%s \n", __FUNCTION__);

	return 0;
error: 
	return 1;	

}



static int
read_tuner_id_register(
	struct rtl2832_state*	p_state,
	unsigned char			tuner_addr,
	unsigned char			tuner_offset,
	unsigned char*		id_data,
	unsigned char			length)
{
	unsigned char				i2c_repeater;	
	struct dvb_usb_device*	d = p_state->d;

	//2 + Begin LOCK
	if(read_demod_register(d, RTL2832_DEMOD_ADDR, 1, 1, &i2c_repeater, 1 )) goto error;
	i2c_repeater |= BIT3;	
	if(write_demod_register(d, RTL2832_DEMOD_ADDR, 1, 1, &i2c_repeater, 1 )) goto error;
	
	if(read_rtl2832_tuner_register(d, tuner_addr, tuner_offset, id_data, length)) goto error;

	if(read_demod_register(d, RTL2832_DEMOD_ADDR, 1, 1, &i2c_repeater, 1 )) goto error;
	i2c_repeater &= (~BIT3);	
	if(write_demod_register(d, RTL2832_DEMOD_ADDR, 1, 1, &i2c_repeater, 1 )) goto error;
	//2 - End LOCK
	return 0;
	
error:
	return 1;
}



static int 
check_tuner_type(
	struct rtl2832_state	*p_state)
{

	unsigned char				tuner_id_data[2];

       deb_info(" +%s\n", __FUNCTION__);

	if ((!read_tuner_id_register(p_state, FC2580_TUNER_ADDR, FC2580_OFFSET,  tuner_id_data, LEN_1_BYTE)) &&
			((tuner_id_data[0]&(~BIT7)) == FC2580_CHECK_VAL ))
	{
		p_state->tuner_type = RTL2832_TUNER_TYPE_FC2580;

		deb_info(" -%s : FC2580 tuner on board...\n", __FUNCTION__);
	}
	else if ((!read_tuner_id_register(p_state, FC0012_BASE_ADDRESS , FC0012_CHECK_ADDRESS,  tuner_id_data, LEN_1_BYTE)) &&
			(tuner_id_data[0] == FC0012_CHECK_VALUE))
	{
		p_state->tuner_type = RTL2832_TUNER_TYPE_FC0012;
				
		deb_info(" -%s : FC0012 tuner on board...\n", __FUNCTION__);	
	}
	else	if((!read_tuner_id_register(p_state, E4000_BASE_ADDRESS, E4000_CHECK_ADDRESS, tuner_id_data, LEN_1_BYTE)) && 
			(tuner_id_data[0] == E4000_CHECK_VALUE))	
	{
		p_state->tuner_type = RTL2832_TUNER_TYPE_E4000;
		deb_info(" -%s : E4000 tuner on board...\n", __FUNCTION__);
	}
	else if ((!read_tuner_id_register(p_state, FC0013_BASE_ADDRESS , FC0013_CHECK_ADDRESS,  tuner_id_data, LEN_1_BYTE)) &&
			(tuner_id_data[0] == FC0013_CHECK_VALUE))
	{
		p_state->tuner_type = RTL2832_TUNER_TYPE_FC0013;
				
		deb_info(" -%s : FC0013 tuner on board...\n", __FUNCTION__);	
	}	
	else
	{
		p_state->tuner_type = RTL2832_TUNER_TYPE_UNKNOWN;
			
		deb_info(" -%s : Unknown tuner on board...\n", __FUNCTION__);	
		goto error;
	}

	return 0;
error:
	return -1;
}



static int 
gpio6_output_enable_direction(
	struct rtl2832_state*	p_state)
{
	int data;
	// GPD_GPIO6=0, GPIO6 output direction
	data = 0;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_DIR_BIT6,data) ) goto error;

	// GPOE_GPIO6=1, GPIO6 output enable
	data = 1;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_EN_BIT6,data) ) goto error;

	return 0;
error:
	return 1;
}



static int 
gpio5_output_enable_direction(
	struct rtl2832_state*	p_state)
{
	int data;
	// GPD_GPIO5=0, GPIO5 output direction
	data = 0;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_DIR_BIT5,data) ) goto error;

	// GPOE_GPIO5=1, GPIO5 output enable
	data = 1;
	if ( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_EN_BIT5,data) ) goto error;

	return 0;
error:
	return 1;
}



static int 
check_dvbt_reset_parameters(

	struct rtl2832_state*	p_state,
	unsigned long			frequency,
	enum fe_bandwidth	bandwidth,	
	int*					reset_needed)
{

	int							is_lock;	
	unsigned int					diff_ms;

	deb_info(" +%s \n", __FUNCTION__);

	*reset_needed = 1;	 //3initialize "reset_needed"
	
	if( (p_state->current_frequency == frequency) && (p_state->current_bandwidth == bandwidth) )
	{
		if( p_state->pNim->IsSignalLocked(p_state->pNim, &is_lock) ) goto error;
		diff_ms = 0;		
		
		while( !(is_lock == LOCKED || diff_ms > 200) )
		{
			platform_wait(40);
			diff_ms += 40;
			if( p_state->pNim->IsSignalLocked(p_state->pNim, &is_lock) ) goto error;
		}

	       if (is_lock==YES)		
	       {
		   *reset_needed = 0;		 //3 set "reset_needed" = 0
		   deb_info("%s : The same frequency = %d setting\n", __FUNCTION__, (int)frequency);
	       }
	}	   

	deb_info(" -%s \n", __FUNCTION__);

	return 0;

error:
	
	*reset_needed = 1; 	//3 set "reset_needed" = 1
	return 1;
}



#if UPDATE_FUNC_ENABLE_2832
void
rtl2832_update_functions(struct work_struct *work)
{
	struct rtl2832_state* p_state = container_of( work,  struct rtl2832_state,  update2832_procedure_work.work); 
	//unsigned  long ber_num, ber_dem;
	//long snr_num = 0;
	//long snr_dem = 0;
	//long _snr= 0;

	if(p_state->pNim == NULL)
	{
		//deb_info("%s error\n", __FUNCTION__);
		goto mutex_error;
	}


	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;

	deb_info(" +%s\n", __FUNCTION__);	

	if(!p_state->is_frequency_valid)
	{
		//deb_info("  %s no need \n", __FUNCTION__);
		goto advance_exit;
	}

	// Update tuner mode
	deb_info(" +%s run update\n", __FUNCTION__);
	if( p_state->pNim->UpdateFunction(p_state->pNim)){
		deb_info(" --->%s run update fail\n", __FUNCTION__);	
		goto advance_exit;
	}
		
	/* p_state->pNim->UpdateFunction(p_state->pNim);
	p_state->pNim->GetBer( p_state->pNim , &ber_num , &ber_dem);
	p_state->pNim->GetSnrDb(p_state->pNim, &snr_num, &snr_dem) ;

	_snr = snr_num / snr_dem;
	if( _snr < 0 ) _snr = 0;

	deb_info("%s : ber = %lu, snr = %lu\n", __FUNCTION__,ber_num,_snr);
	*/

advance_exit:
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
	schedule_delayed_work(&p_state->update2832_procedure_work, UPDATE_PROCEDURE_PERIOD_2832);

	deb_info(" -%s\n", __FUNCTION__);
	
	return;

mutex_error:
	return;
	
}
#endif



static int 
rtl2832_init(
	struct dvb_frontend*	fe)
{
	struct rtl2832_state*	p_state = fe->demodulator_priv;

	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;

	deb_info(" +%s\n", __FUNCTION__);
	
	//usb_reset_device(p_state->d->udev);

	if( usb_init_setting(p_state) ) goto error;
	
	if( gpio3_out_setting(p_state) ) goto error;				//3Set GPIO3 OUT	
	
	if( demod_ctl1_setting(p_state , 1) ) goto error;		//3	DEMOD_CTL1, resume = 1
	
	if (dvb_use_rtl2832u_card_type)
	{
		if( set_demod_power(p_state , 1) ) goto error;			//3	turn ON demod power
	}
	
	if( suspend_latch_setting(p_state , 1) ) goto error;		//3 suspend_latch_en = 0, resume = 1 					

	if( demod_ctl_setting(p_state , 1,  1) ) goto error;		//3 DEMOD_CTL, resume =1; ADC on
	
	if( set_tuner_power(p_state, 1, 1) ) goto error;		//3	turn ON tuner power

	if( p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012)
	{
		int data;
		if( gpio5_output_enable_direction(p_state))		goto error;

		if( read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, &data) ) goto error;
		data |= BIT5; // set GPIO5 high
		if( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, data) ) goto error;
		data &= ~(BIT5); // set GPIO5 low
		if( write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, data) ) goto error;
	}
		
	//3 Nim initial
	switch(p_state->demod_type)
	{	
		case RTL2832:
		{
			// Nim initialize for 2832
			if ( p_state->pNim->Initialize(p_state->pNim) ) goto error;
		}
		break;

	}

	//3  RTL2832U AGC Setting, Serial Mode Switch Setting, PIP Setting based on demod_type
	if(demod_init_setting(p_state)) goto error;
	
	p_state->is_initial = 1; 
	
	deb_info(" -%s \n", __FUNCTION__);

	mutex_unlock(&p_state->i2c_repeater_mutex);	

#if UPDATE_FUNC_ENABLE_2832
       if(p_state->demod_type == RTL2832)
	   	schedule_delayed_work(&(p_state->update2832_procedure_work), 0);//3 Initialize update function
#endif

	return 0;
error:
	mutex_unlock(&p_state->i2c_repeater_mutex);	
	
mutex_error:
	deb_info(" -%s  error end\n", __FUNCTION__);

	return -1;
}



static void
rtl2832_release(
	struct dvb_frontend*	fe)
{
	struct rtl2832_state* p_state = fe->demodulator_priv;

	if( p_state->pNim==NULL)
	{
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return;
	}

	if(p_state->is_initial)
	{

#if UPDATE_FUNC_ENABLE_2832
              if(p_state->demod_type == RTL2832)
              {
	       	cancel_delayed_work( &(p_state->update2832_procedure_work) );//cancel_rearming_delayed_work
		       flush_scheduled_work();
              }
#endif
		p_state->is_initial = 0;
	}
	p_state->is_frequency_valid = 0;
	//IrDA	
	rtl2832u_remote_control_state = RC_NO_SETTING;

	kfree(p_state);

	deb_info("  %s \n", __FUNCTION__);	

	return;
}



static int 
rtl2832_sleep_mode(struct rtl2832_state* p_state)
{
	if( p_state->is_initial )
	{

#if UPDATE_FUNC_ENABLE_2832
              if(p_state->demod_type == RTL2832)
              {
	              cancel_delayed_work( &(p_state->update2832_procedure_work));// cancel_rearming_delayed_work
	       	flush_scheduled_work();
              }
#endif		
		p_state->is_initial = 0;
	}
	p_state->is_frequency_valid = 0;

	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;

	deb_info(" +%s \n", __FUNCTION__);

	if( demod_ctl1_setting(p_state , 0) ) goto error;		//3	DEMOD_CTL1, resume = 0

	if( set_tuner_power(p_state, 1, 0) ) goto error;				//3turn OFF tuner power

	if( demod_ctl_setting(p_state , 0,  0) ) goto error;		//3 	DEMOD_CTL, resume =0	

	//2 for H/W reason
	//if( suspend_latch_setting(p_state , 0) ) goto error;		//3 suspend_latch_en = 1, resume = 0					
	if (dvb_use_rtl2832u_card_type)
	{
		deb_info(" -%s ::mini card mode gpio0 set high ,demod power off\n", __FUNCTION__);
		if( set_demod_power(p_state , 0) ) goto error;		//3	turn OFF demod power
	}
	deb_info(" -%s \n", __FUNCTION__);

	mutex_unlock(&p_state->i2c_repeater_mutex);

	return 0;
	
error:
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
mutex_error:
	deb_info(" -%s fail\n", __FUNCTION__);	
	return 1;

}



static int 
rtl2832_sleep(
	struct dvb_frontend*	fe)
{
	struct rtl2832_state* p_state = fe->demodulator_priv;

	return rtl2832_sleep_mode(p_state);
}



static int
rtl2832_set_parameters(
	struct dvb_frontend*	fe,
	struct dvb_frontend_parameters*	param)
{
	struct rtl2832_state			*p_state = fe->demodulator_priv;
	struct dvb_ofdm_parameters	*p_ofdm_param= &param->u.ofdm;

	unsigned long					frequency = param->frequency;
	int							bandwidth_mode;
	int							is_signal_present;
	int							reset_needed;
	int							int_data; 

       
       if( p_state->pNim == NULL)
       {
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return -1;
       }

	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;
	
	deb_info(" +%s frequency = %lu , bandwidth = %u\n", __FUNCTION__, frequency ,p_ofdm_param->bandwidth);

	if(p_state->demod_type == RTL2832)
	{
		if ( check_dvbt_reset_parameters( p_state , frequency , p_ofdm_param->bandwidth, &reset_needed) ) goto error;

		if( reset_needed == 0 )
		{
			mutex_unlock(&p_state->i2c_repeater_mutex);		
			return 0;
		}

		switch (p_ofdm_param->bandwidth) 
	      {
		case BANDWIDTH_6_MHZ:
		bandwidth_mode = DVBT_BANDWIDTH_6MHZ; 	
		break;
		
		case BANDWIDTH_7_MHZ:
		bandwidth_mode = DVBT_BANDWIDTH_7MHZ;
		break;
		
		case BANDWIDTH_8_MHZ:
		default:
		bandwidth_mode = DVBT_BANDWIDTH_8MHZ;	
		break;
	       }
	       
	       	//add by Dean
	        if (p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012 )
	        {
	        
	   		if( gpio6_output_enable_direction(p_state) )	goto error;	    
	   		
	   			
			if (frequency > 300000000)
			{
				
				read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, &int_data);
				int_data |= BIT6; // set GPIO6 high
				write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, int_data);	
				deb_info("  %s : Tuner :FC0012 V-band (GPIO6 high)\n", __FUNCTION__);		
			}
			else
			{
	
				read_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, &int_data);
				int_data &= (~BIT6); // set GPIO6 low	
				write_usb_sys_register(p_state, RTD2831_RMAP_INDEX_SYS_GPIO_OUTPUT_VAL, int_data);	
				deb_info("  %s : Tuner :FC0012 U-band (GPIO6 low)\n", __FUNCTION__);	
				
			}
		}
				
		if ( p_state->pNim->SetParameters( p_state->pNim,  frequency , bandwidth_mode ) ) goto error; 

		if ( p_state->pNim->IsSignalPresent( p_state->pNim, &is_signal_present) ) goto error;
		deb_info("  %s : ****** Signal Present = %d ******\n", __FUNCTION__, is_signal_present);
		
		p_state->is_frequency_valid = 1;

	}


//#if(UPDATE_FUNC_ENABLE_2832 == 0)
	//3 FC0012/E4000 update begin --
	if(p_state->demod_type == RTL2832 && (p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012 
										|| p_state->tuner_type == RTL2832_TUNER_TYPE_E4000
										|| p_state->tuner_type == RTL2832_TUNER_TYPE_FC0013))
	{
              // Enable demod DVBT_IIC_REPEAT.
		if(p_state->pNim->pDemod->SetRegBitsWithPage(p_state->pNim->pDemod, DVBT_IIC_REPEAT, 0x1) != FUNCTION_SUCCESS)
			goto error;
 
		if(p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012)//fc0012
		{
			if(rtl2832_fc0012_UpdateTunerLnaGainWithRssi(p_state->pNim) != FUNCTION_SUCCESS)
				goto error;
		}
		else if(p_state->tuner_type == RTL2832_TUNER_TYPE_E4000)//e4000
		{
			if(rtl2832_e4000_UpdateTunerMode(p_state->pNim) != FUNCTION_SUCCESS)
				goto error;
		}
		else if(p_state->tuner_type == RTL2832_TUNER_TYPE_FC0013) //fc0013
		{
			if(rtl2832_fc0013_UpdateTunerLnaGainWithRssi(p_state->pNim) != FUNCTION_SUCCESS)
				goto error;
		}
		
		// Disable demod DVBT_IIC_REPEAT.
		if(p_state->pNim->pDemod->SetRegBitsWithPage(p_state->pNim->pDemod, DVBT_IIC_REPEAT, 0x0) != FUNCTION_SUCCESS)
			goto error;
                

		deb_info("%s : fc0012/e4000 update first\n", __FUNCTION__);

		msleep(50);
 
		if(p_state->pNim->pDemod->SetRegBitsWithPage(p_state->pNim->pDemod, DVBT_IIC_REPEAT, 0x1) != FUNCTION_SUCCESS)
			goto error;

		// Update tuner LNA gain with RSSI.
		if(p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012)//fc0012
		{
			if(rtl2832_fc0012_UpdateTunerLnaGainWithRssi(p_state->pNim) != FUNCTION_SUCCESS)
				goto error;
		}
		else if (p_state->tuner_type == RTL2832_TUNER_TYPE_E4000)//e4000
		{
			if(rtl2832_e4000_UpdateTunerMode(p_state->pNim) != FUNCTION_SUCCESS)
				goto error;
		}
		else if(p_state->tuner_type == RTL2832_TUNER_TYPE_FC0013) //fc0013
		{
			if(rtl2832_fc0013_UpdateTunerLnaGainWithRssi(p_state->pNim) != FUNCTION_SUCCESS)
				goto error;
		}
			
		// Disable demod DVBT_IIC_REPEAT.
		if(p_state->pNim->pDemod->SetRegBitsWithPage(p_state->pNim->pDemod, DVBT_IIC_REPEAT, 0x0) != FUNCTION_SUCCESS)
			goto error;

		deb_info("%s : fc0012/e4000 update 2nd\n", __FUNCTION__);
	}
	//3 FC0012/E4000 update end --
      
//#endif
	
	p_state->current_frequency = frequency;	
	p_state->current_bandwidth = p_ofdm_param->bandwidth;	

	deb_info(" -%s \n", __FUNCTION__);

	mutex_unlock(&p_state->i2c_repeater_mutex);	

	return 0;

error:	
	mutex_unlock(&p_state->i2c_repeater_mutex);	
	
mutex_error:	
	p_state->current_frequency = 0;
	p_state->current_bandwidth = -1;
	p_state->is_frequency_valid = 0;
	deb_info(" -%s  error end\n", __FUNCTION__);

	return -1;
	
}



static int 
rtl2832_get_parameters(
	struct dvb_frontend*	fe,
	struct dvb_frontend_parameters*	param)
{
	return 0;
}



static int 
rtl2832_read_status(
	struct dvb_frontend*	fe,
	fe_status_t*	status)
{
	struct rtl2832_state*	p_state = fe->demodulator_priv;
	int	is_lock;			
	unsigned  long ber_num, ber_dem;
	long			snr_num, snr_dem, snr;


	if( p_state->pNim== NULL)
	{
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return -1;
	}

	*status = 0;	//3initialize "status"
	
	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;

	if(p_state->demod_type == RTL2832)
	{
	
		if( p_state->pNim->GetBer( p_state->pNim , &ber_num , &ber_dem) ) goto error;

		if (p_state->pNim->GetSnrDb(p_state->pNim, &snr_num, &snr_dem) )  goto error;
	
		if( p_state->pNim->IsSignalLocked(p_state->pNim, &is_lock) ) goto error;
	
		if( is_lock==YES ) *status|= (FE_HAS_CARRIER| FE_HAS_VITERBI| FE_HAS_LOCK| FE_HAS_SYNC| FE_HAS_SIGNAL);

		 snr = snr_num/snr_dem;

		deb_info("%s :******RTL2832 Signal Lock=%d******\n", __FUNCTION__, is_lock);
		deb_info("%s : ber_num = %d\n", __FUNCTION__, (unsigned int)ber_num);	
		deb_info("%s : snr = %d \n", __FUNCTION__, (int)snr);	
	}

#if(UPDATE_FUNC_ENABLE_2832 == 0)

	if(p_state->demod_type == RTL2832)
	{
		if( p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012 ||
			p_state->tuner_type == RTL2832_TUNER_TYPE_E4000 ||
			p_state->tuner_type == RTL2832_TUNER_TYPE_FC0013 )
		{
			// Update tuner LNA gain with RSSI.
			if( p_state->pNim->UpdateFunction(p_state->pNim))
				goto error;	
 		}//3   
	}
#endif

	mutex_unlock(&p_state->i2c_repeater_mutex);
	
	return 0;

error:
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
mutex_error:
	return -1;
}



static int 
rtl2832_read_ber(
	struct dvb_frontend*	fe,
	u32*	ber)
{
	struct rtl2832_state* p_state = fe->demodulator_priv;
	unsigned  long ber_num, ber_dem;

	if( p_state->pNim== NULL)
	{
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return -1;
	}


	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;

	if(p_state->demod_type == RTL2832)
	{
		if( p_state->pNim->GetBer( p_state->pNim , &ber_num , &ber_dem) ) 
		{
			*ber = 19616;
			goto error;
		}
		*ber =  ber_num;
		deb_info("  %s : ber = 0x%x \n", __FUNCTION__, *ber);
	}

	mutex_unlock(&p_state->i2c_repeater_mutex);
		
	return 0;
	
error:
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
mutex_error:
	return -1;
}



static int 
fc0012_get_signal_strength(
	struct rtl2832_state	*p_state,
	unsigned long *strength)
{
	int intTemp;
	int Power;
	int intTotalAGCGain;
	int intLNA;
	unsigned char ReadingByte;
	int LnaGain_reg;
	int NumberOfLnaGainTable;
	int i;
	int Index;
	TUNER_MODULE *pTuner;
	DVBT_DEMOD_MODULE *pDemod;	
	

	if(p_state->pNim== NULL)
	{
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return -1;
	}
	pDemod = p_state->pNim->pDemod;				
	pTuner = p_state->pNim->pTuner;	

	// Enable demod DVBT_IIC_REPEAT.
	if(pDemod->SetRegBitsWithPage(pDemod, DVBT_IIC_REPEAT, 0x1) != FUNCTION_SUCCESS)
		goto error;

	if(FC0012_Write(pTuner, 0x12, 0x00) != FUNCTION_SUCCESS) 
		goto error;

	if(FC0012_Read(pTuner, 0x12, &ReadingByte) != FUNCTION_SUCCESS)
		goto error;
	intTemp=(int)ReadingByte;

	if(FC0012_Read(pTuner, 0x13, &ReadingByte) != FUNCTION_SUCCESS)
		goto error;
	LnaGain_reg=(int)ReadingByte&0x7f;

	// Disable demod DVBT_IIC_REPEAT.
	if(pDemod->SetRegBitsWithPage(pDemod, DVBT_IIC_REPEAT, 0x0) != FUNCTION_SUCCESS)
		goto error;
		
	NumberOfLnaGainTable=sizeof(FC0012_LNA_GAIN_TABLE)/sizeof(FC0012_LNA_REG_MAP);
	Index=-1;
	for (i=0;i<NumberOfLnaGainTable;i++)
	{
		if (LnaGain_reg == FC0012_LNA_GAIN_TABLE[i].Lna_regValue)
		{
			Index=i;
			break;
		}
	}

	if (Index <0) 
	{
		goto error;
	}
	
	intLNA=FC0012_LNA_GAIN_TABLE[Index].LnaGain;

	intTotalAGCGain = ((abs((intTemp >> 5) - 7) -2) * 2 + (intTemp & 0x1F) * 2);

	 Power= INPUT_ADC_LEVEL - intTotalAGCGain - (intLNA/10);				

	deb_info(" %s power=%d form fc0012(%x,%x,%x)\n", __FUNCTION__,Power,intTemp,LnaGain_reg,intLNA);			

	//Signal Strength : map power to 0~100


	if(Power >=-45) 
	{
		*strength=100;

	}
	else if(Power <-95) 
			*strength=0;
	else
			*strength = ((Power+45)*100)/50+100;
	return 0;
error:
	return -1;	
}



int 
rtl2832_read_signal_strength(
	struct dvb_frontend*	fe,
	u16*	strength)
{
	struct rtl2832_state* p_state = fe->demodulator_priv;
	unsigned long		_strength;

	if( p_state->pNim== NULL)
	{
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return -1;
	}

	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;


       if(p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012)	
	{
	  if (fc0012_get_signal_strength(p_state,&_strength))	
	  {
			*strength = 0;
			goto error;	  	
	  }


	   *strength = (_strength<<8) | _strength;
          deb_info("  %s : use FC0012 strength = 0x%x(%d) \n", __FUNCTION__, *strength,*strength);	  		
	}
       else if(p_state->demod_type == RTL2832)
       {
     	
       		if( p_state->pNim->GetSignalStrength( p_state->pNim ,  &_strength) ) 
		{
			*strength = 0;
			goto error;
		}
	
		*strength = (_strength<<8) | _strength;

		deb_info("  %s : RTL2832 strength = 0x%x \n", __FUNCTION__, *strength);		
        }
	
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
	return 0;
	
error:
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
mutex_error:
	return -1;
}



int 
rtl2832_read_signal_quality(
	struct dvb_frontend*	fe,
	u32*	quality)
{
	struct rtl2832_state* p_state = fe->demodulator_priv;
	unsigned long		_quality;

	if( p_state->pNim== NULL)
	{
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return -1;
	}

	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;

	if(p_state->demod_type == RTL2832)
	{
		if ( p_state->pNim->GetSignalQuality( p_state->pNim ,  &_quality) )
		{
			*quality  = 0;		
			goto error;
		}
	}

	*quality = _quality;
	
	deb_info("  %s : quality = 0x%x \n", __FUNCTION__, *quality);	

	mutex_unlock(&p_state->i2c_repeater_mutex);
	
	return 0;
	
error:
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
mutex_error:
	return -1;	
}



static int 
rtl2832_read_snr(
	struct dvb_frontend*	fe,
	u16*	snr)
{
	struct rtl2832_state* p_state = fe->demodulator_priv;
	long snr_num = 0;
	long snr_dem = 0;
	long _snr= 0;

	if( p_state->pNim== NULL)
	{
		deb_info(" %s pNim = NULL \n", __FUNCTION__);
		return -1;
	}

	if( mutex_lock_interruptible(&p_state->i2c_repeater_mutex) )	goto mutex_error;

       if(p_state->demod_type == RTL2832)
       {
		if (p_state->pNim->GetSnrDb(p_state->pNim, &snr_num, &snr_dem) ) 
		{
			*snr = 0;
			goto error;
		}

		_snr = snr_num / snr_dem;

		if( _snr < 0 ) _snr = 0;

		*snr = _snr;
        }
	
	deb_info("  %s : snr = %d \n", __FUNCTION__, *snr);	

	mutex_unlock(&p_state->i2c_repeater_mutex);
	
	return 0;
	
error:
	mutex_unlock(&p_state->i2c_repeater_mutex);
	
mutex_error:
	return -1;
}



static int 
rtl2832_get_tune_settings(
	struct dvb_frontend*	fe,
	struct dvb_frontend_tune_settings*	fe_tune_settings)
{
	deb_info("  %s : Do Nothing\n", __FUNCTION__);	
	fe_tune_settings->min_delay_ms = 1000;
	return 0;
}



static int 
rtl2832_ts_bus_ctrl(
	struct dvb_frontend*	fe,
	int	acquire)
{
	deb_info("  %s : Do Nothing\n", __FUNCTION__);	
	return 0;
}



static struct dvb_frontend_ops rtl2832_dvbt_ops;



struct dvb_frontend* rtl2832u_fe_attach(struct dvb_usb_device *d)
{

	struct rtl2832_state	*p_state= NULL;
	//char			tmp_set_tuner_power_gpio4;
	
	deb_info("+%s : chialing 0409-1\n", __FUNCTION__);
	 
	//3 linux fe_attach  necessary setting
	/*allocate memory for the internal state */
	p_state = kzalloc(sizeof(struct rtl2832_state), GFP_KERNEL);
	if (p_state == NULL) goto error;
	memset(p_state,0,sizeof(*p_state));

	p_state->is_initial = 0;			//initialize is_initial 
	p_state->is_frequency_valid = 0;
	p_state->d = d;

	if( usb_init_setting(p_state) ) goto error;
	
	if( gpio3_out_setting(p_state) ) goto error;			//3Set GPIO3 OUT	
	
	if( demod_ctl1_setting(p_state , 1) ) goto error;		//3	DEMOD_CTL1, resume = 1
	if (dvb_use_rtl2832u_card_type)
	{
		if( set_demod_power(p_state , 1) ) goto error;		
	}
	
	if( suspend_latch_setting(p_state , 1) ) goto error;		//3 suspend_latch_en = 0, resume = 1 					

	if( demod_ctl_setting(p_state , 1,  1) ) goto error;		//3 DEMOD_CTL, resume =1; ADC on

	//3 Auto detect Tuner Power Pin (GPIO3 or GPIO4)	
	if( set_tuner_power(p_state , 1 , 1) ) goto error;		//3	turn ON tuner power, 1st try GPIO4

	if( check_tuner_type(p_state) )		goto error;

       //3 Check if support RTL2836 DTMB. 
	p_state->demod_support_type = 0;


       //3Set demod_type.
	p_state->demod_type = RTL2832;
	deb_info("demod_type is %d\n", p_state->demod_type);

	//3 Build Nim Module
	build_nim_module(p_state);
    
	/* setup the state */
	switch(p_state->demod_type)
	{
		case RTL2832:
			memcpy(&p_state->frontend.ops, &rtl2832_dvbt_ops, sizeof(struct dvb_frontend_ops));
			memset(&p_state->fep, 0, sizeof(struct dvb_frontend_parameters));			
		break;

	}
	
 	/* create dvb_frontend */
	p_state->frontend.demodulator_priv = p_state;

#if UPDATE_FUNC_ENABLE_2832
	INIT_DELAYED_WORK(&(p_state->update2832_procedure_work), rtl2832_update_functions);
#endif
	mutex_init(&p_state->i2c_repeater_mutex);
	if (dvb_use_rtl2832u_rc_mode<3){
		deb_info(">>%s go to sleep mode(low power mode)\n", __FUNCTION__);
			if (rtl2832_sleep_mode(p_state)){
			deb_info("sleep mode is fail \n");
		}
	}		
	deb_info("-%s\n", __FUNCTION__);
	return &p_state->frontend;

error:	
	return NULL;	


}



static struct dvb_frontend_ops rtl2832_dvbt_ops = {
    .info = {
        .name               = "Realtek DVB-T RTL2832",
        .type               = FE_OFDM,
        .frequency_min      = 50000000,
        .frequency_max      = 862000000,
        .frequency_stepsize = 166667,
        .caps = FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 |
            FE_CAN_FEC_3_4 | FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 |
            FE_CAN_FEC_AUTO |
            FE_CAN_QPSK | FE_CAN_QAM_16 | FE_CAN_QAM_64 | FE_CAN_QAM_AUTO |
            FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_GUARD_INTERVAL_AUTO |
            FE_CAN_HIERARCHY_AUTO | FE_CAN_RECOVER |
            FE_CAN_MUTE_TS
    },

    .init =					rtl2832_init,
    .release =				rtl2832_release,

    .sleep =				rtl2832_sleep,

    .set_frontend =			rtl2832_set_parameters,
    .get_frontend =			rtl2832_get_parameters,
    .get_tune_settings =		rtl2832_get_tune_settings,

    .read_status =			rtl2832_read_status,
    .read_ber =				rtl2832_read_ber,
    .read_signal_strength =		rtl2832_read_signal_strength,
    .read_snr =			rtl2832_read_snr,
    .read_ucblocks =		rtl2832_read_signal_quality,
    .ts_bus_ctrl   =			rtl2832_ts_bus_ctrl, 
};



static int
demod_init_setting(
		struct rtl2832_state * p_state
		)
{

	unsigned char data;
	
	deb_info(" +%s\n", __FUNCTION__);

	switch(p_state->demod_type)
	{
	case RTL2832:	
		{
			deb_info("%s for RTL2832\n", __FUNCTION__);
			//3 1. Set IF_AGC Internal    IF_AGC_MAN 0x0c
		       if(read_demod_register(p_state->d,  RTL2832_DEMOD_ADDR, PAGE_1, 0x0c, &data,  LEN_1_BYTE)) goto error;
			data &= (~BIT6);
			if(write_demod_register(p_state->d, RTL2832_DEMOD_ADDR, PAGE_1, 0x0c, &data, LEN_1_BYTE)) goto error;

		      /*if(!context->DAB_AP_running)
			{
				//3 2. Set PID filter to reject null packet(pid = 0x1fff) 
				context->pid_filter_mode = REJECT_MODE;	
				ULONG reject_pid[1] = {0x1fff}; 
				Status = PidFilterToRejectMode(context, reject_pid, 1);
				if(!NT_SUCCESS(Status))	goto error;
			}*/
			
		}
		break;	

		default:
			
		deb_info(" %s : ERROR -- ONLY for DVB-T (RTL2832U)\n", __FUNCTION__);			

		break;
	}

	deb_info(" -%s\n", __FUNCTION__);
	return 0;
	
error:
	deb_info(" -%s error \n", __FUNCTION__);
	return 1;

}



static int
build_2832_nim_module(
		struct rtl2832_state*  p_state)
{
	deb_info(" +%s\n", __FUNCTION__);

      //3 Buile 2832 nim module
	if( p_state->tuner_type == RTL2832_TUNER_TYPE_FC2580)
	{

		//3Build RTL2832 FC2580 NIM module.
		BuildRtl2832Fc2580Module(
			&p_state->pNim,
			&p_state->DvbtNimModuleMemory,

			2,					// Maximum I2C reading byte number is 9.
			2,					// Maximum I2C writing byte number is 8.
			custom_i2c_read,			// Employ CustomI2cRead() as basic I2C reading function.
			custom_i2c_write,			// Employ CustomI2cWrite() as basic I2C writing function.
			custom_wait_ms,				// Employ CustomWaitMs() as basic waiting function.

			RTL2832_DEMOD_ADDR,			// The RTL2832 I2C device address is 0x20 in 8-bit format.
			CRYSTAL_FREQ_28800000HZ,		// The RTL2832 crystal frequency is 28.8 MHz.
			TS_INTERFACE_PARALLEL,			// The RTL2832 TS interface mode is PARALLEL.
			RTL2832_APPLICATION_DONGLE,		// The RTL2832 application mode is DONGLE mode.
			200,					// The RTL2832 update function reference period is 200 millisecond
			OFF,					// The RTL2832 Function 1 enabling status is YES.

			FC2580_TUNER_ADDR,			// The FC2580 I2C device address is 0xac in 8-bit format.
			CRYSTAL_FREQ_16384000HZ,		// The FC2580 crystal frequency is 16.384 MHz.
			FC2580_AGC_INTERNAL			// The FC2580 AGC mode is external AGC mode.
			);
		deb_info(" %s BuildRtl2832Fc2580Module\n", __FUNCTION__);	
		
	}
	else if(p_state->tuner_type == RTL2832_TUNER_TYPE_FC0012)
	{
		//3Build RTL2832 FC0012 NIM module.
		BuildRtl2832Fc0012Module(
			&p_state->pNim,
			&p_state->DvbtNimModuleMemory,

			2,						// Maximum I2C reading byte number is 2.
			2,						// Maximum I2C writing byte number is 2.
			custom_i2c_read,				// Employ CustomI2cRead() as basic I2C reading function.
			custom_i2c_write,				// Employ CustomI2cWrite() as basic I2C writing function.
			custom_wait_ms,					// Employ CustomWaitMs() as basic waiting function.

			RTL2832_DEMOD_ADDR,				// The RTL2832 I2C device address is 0x20 in 8-bit format.
			CRYSTAL_FREQ_28800000HZ,			// The RTL2832 crystal frequency is 28.8 MHz.
			TS_INTERFACE_PARALLEL,				// The RTL2832 TS interface mode is PARALLEL.
			RTL2832_APPLICATION_DONGLE,			// The RTL2832 application mode is DONGLE mode.
			200,						// The RTL2832 update function reference period is 200 millisecond
			OFF,						// The RTL2832 Function 1 enabling status is YES.

			FC0012_BASE_ADDRESS,				// The FC0012 I2C device address is 0xc6 in 8-bit format.
			CRYSTAL_FREQ_28800000HZ				// The FC0012 crystal frequency is 36.0 MHz.
			);
		deb_info(" %s BuildRtl2832Fc0012Module\n", __FUNCTION__);	

	}
	else if(p_state->tuner_type == RTL2832_TUNER_TYPE_E4000)
	{
		//3 Build RTL2832 E4000 NIM module
		BuildRtl2832E4000Module(
			&p_state->pNim,
			&p_state->DvbtNimModuleMemory,

			2,						// Maximum I2C reading byte number is 2.
			2,						// Maximum I2C writing byte number is 2.
			custom_i2c_read,				// Employ CustomI2cRead() as basic I2C reading function.
			custom_i2c_write,				// Employ CustomI2cWrite() as basic I2C writing function.
			custom_wait_ms,					// Employ CustomWaitMs() as basic waiting function.

			RTL2832_DEMOD_ADDR,				// The RTL2832 I2C device address is 0x20 in 8-bit format.
			CRYSTAL_FREQ_28800000HZ,			// The RTL2832 crystal frequency is 28.8 MHz.
			TS_INTERFACE_PARALLEL,				// The RTL2832 TS interface mode is PARALLEL.
			RTL2832_APPLICATION_DONGLE,			// The RTL2832 application mode is DONGLE mode.
			200,						// The RTL2832 update function reference period is 50 millisecond
			OFF,						// The RTL2832 Function 1 enabling status is YES.

			E4000_BASE_ADDRESS,				// The E4000 I2C device address is 0xc8 in 8-bit format.
			CRYSTAL_FREQ_28800000HZ				// The E4000 crystal frequency is 28.8 MHz.
			);
		deb_info(" %s BuildRtl2832E4000Module\n", __FUNCTION__);
		
	}
	else if(p_state->tuner_type == RTL2832_TUNER_TYPE_FC0013)
	{
		//3Build RTL2832 FC0012 NIM module.
		BuildRtl2832Fc0013Module(
			&p_state->pNim,
			&p_state->DvbtNimModuleMemory,

			2,						// Maximum I2C reading byte number is 2.
			2,						// Maximum I2C writing byte number is 2.
			custom_i2c_read,				// Employ CustomI2cRead() as basic I2C reading function.
			custom_i2c_write,				// Employ CustomI2cWrite() as basic I2C writing function.
			custom_wait_ms,					// Employ CustomWaitMs() as basic waiting function.

			RTL2832_DEMOD_ADDR,				// The RTL2832 I2C device address is 0x20 in 8-bit format.
			CRYSTAL_FREQ_28800000HZ,			// The RTL2832 crystal frequency is 28.8 MHz.
			TS_INTERFACE_PARALLEL,				// The RTL2832 TS interface mode is PARALLEL.
			RTL2832_APPLICATION_DONGLE,			// The RTL2832 application mode is DONGLE mode.
			200,						// The RTL2832 update function reference period is 200 millisecond
			OFF,						// The RTL2832 Function 1 enabling status is YES.

			FC0013_BASE_ADDRESS,				// The FC0012 I2C device address is 0xc6 in 8-bit format.
			CRYSTAL_FREQ_28800000HZ				// The FC0012 crystal frequency is 36.0 MHz.
			);
		deb_info(" %s BuildRtl2832Fc0013Module\n", __FUNCTION__);	

	}
	else
	{
		deb_info(" -%s : RTL 2832 Unknown tuner on board...\n", __FUNCTION__);		
		goto error;
	}
	//Set user defined data pointer of base interface structure for custom basic functions.
       p_state->pNim->pBaseInterface->SetUserDefinedDataPointer(p_state->pNim->pBaseInterface, p_state->d );

	deb_info(" -%s\n", __FUNCTION__);

	return 0;

error:
	return 1;

}




static int
build_nim_module(
	struct rtl2832_state*  p_state
	)
{
	deb_info(" +%s\n", __FUNCTION__);

	switch(p_state->demod_type)
	{
		case RTL2832:
		// Build 2832 nim module
		build_2832_nim_module(p_state);
		break;

		default:
		deb_info(" %s : ERROR -- ONLY for DVB-T (RTL2832U)\n", __FUNCTION__);
		break;
	}

	deb_info(" -%s\n", __FUNCTION__);
	return 0;
}





