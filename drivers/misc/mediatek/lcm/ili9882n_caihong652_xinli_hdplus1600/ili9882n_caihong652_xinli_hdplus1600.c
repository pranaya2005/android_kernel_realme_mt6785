/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define LOG_TAG "LCM"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif
//#define LCM_ID   0x9882
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

/* static unsigned char lcd_id_pins_value = 0xFF; */
#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT									(1600)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH								(67930)
#define LCM_PHYSICAL_HEIGHT								(150960)
#define LCM_DENSITY										(320)

#define REGFLAG_DELAY		    0xFFFC
#define REGFLAG_UDELAY	        0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	    0xFFFE
#define REGFLAG_RESET_HIGH	    0xFFFF

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_init_setting[] = {
  {0xFF,3,{0x98,0x82,0x01}},
  {0x00,1,{0x46}},
  {0x01,1,{0x13}},
  {0x02,1,{0x00}},
  {0x03,1,{0x20}},
  {0x04,1,{0x01}},
  {0x05,1,{0x13}},
  {0x06,1,{0x00}},
  {0x07,1,{0x20}},
  {0x08,1,{0x82}},
  {0x09,1,{0x05}},
  {0x0A,1,{0x73}},
  {0x0B,1,{0x00}},
  {0x0c,1,{0x05}},
  {0x0d,1,{0x05}},
  {0x31,1,{0x0C}},
  {0x32,1,{0x06}},
  {0x33,1,{0x02}},
  {0x34,1,{0x02}},
  {0x35,1,{0x02}},
  {0x36,1,{0x02}},
  {0x37,1,{0x02}},
  {0x38,1,{0x16}},
  {0x39,1,{0x16}},
  {0x3A,1,{0x14}},
  {0x3B,1,{0x14}},
  {0x3C,1,{0x12}},
  {0x3D,1,{0x12}},
  {0x3E,1,{0x10}},
  {0x3F,1,{0x10}},
  {0x40,1,{0x08}},
  {0x41,1,{0x07}},
  {0x42,1,{0x07}},
  {0x43,1,{0x07}},
  {0x44,1,{0x07}},
  {0x45,1,{0x07}},
  {0x46,1,{0x07}},
  {0x47,1,{0x0D}},
  {0x48,1,{0x06}},
  {0x49,1,{0x02}},
  {0x4A,1,{0x02}},
  {0x4B,1,{0x02}},
  {0x4C,1,{0x02}},
  {0x4D,1,{0x02}},
  {0x4E,1,{0x17}},
  {0x4F,1,{0x17}},
  {0x50,1,{0x15}},
  {0x51,1,{0x15}},
  {0x52,1,{0x13}},
  {0x53,1,{0x13}},
  {0x54,1,{0x11}},
  {0x55,1,{0x11}},
  {0x56,1,{0x09}},
  {0x57,1,{0x07}},
  {0x58,1,{0x07}},
  {0x59,1,{0x07}},
  {0x5a,1,{0x07}},
  {0x5b,1,{0x07}},
  {0x5c,1,{0x07}},
  {0xE6,1,{0x22}},
  {0xE7,1,{0x54}},
  {0xFF,3,{0x98,0x82,0x02}},
  {0xF1,1,{0x1c}},
  {0x4B,1,{0x5A}},
  {0x50,1,{0xCA}},
  {0x51,1,{0x00}},
  {0x06,1,{0x8F}},
  {0x0B,1,{0xA0}},
  {0x0C,1,{0x00}},
  {0x0D,1,{0x14}},
  {0x0E,1,{0xE6}},
  {0x4E,1,{0x22}},
  {0x4D,1,{0xCE}},
  {0xFF,3,{0x98,0x82,0x05}},
  {0x03,1,{0x00}},
  {0x04,1,{0xD3}},
  {0x58,1,{0x61}},
  {0x63,1,{0x8D}},
  {0x64,1,{0x8D}},
  {0x68,1,{0xA1}},
  {0x69,1,{0xA7}},
  {0x6A,1,{0x79}},
  {0x6B,1,{0x6B}},
  {0x85,1,{0x37}},
  {0x46,1,{0x00}},
  {0xFF,3,{0x98,0x82,0x06}},
  {0xD9,1,{0x1F}},
  {0xC0,1,{0x40}},
  {0xC1,1,{0x16}},
  {0xFF,3,{0x98,0x82,0x08}},
  {0xE0,27,{0x40,0x24,0x9E,0xDD,0x22,0x55,0x57,0x7D,0xAA,0xCD,0xAA,0x02,0x2B,0x4F,0x72,0xEA,0x94,0xC0,0xDC,0x00,0xFF,0x1F,0x48,0x7B,0xA7,0x03,0xEC}},
  {0xE1,27,{0x40,0x24,0x9E,0xDD,0x22,0x55,0x57,0x7D,0xAA,0xCD,0xAA,0x02,0x2B,0x4F,0x72,0xEA,0x94,0xC0,0xDC,0x00,0xFF,0x1F,0x48,0x7B,0xA7,0x03,0xEC}},
  {0xFF,1,{0x98,0x82,0x0B}},
  {0x9A,1,{0x44}},
  {0x9B,1,{0x81}},
  {0x9C,1,{0x03}},
  {0x9D,1,{0x03}},
  {0x9E,1,{0x70}},
  {0x9F,1,{0x70}},
  {0xAB,1,{0xE0}},
  {0xFF,3,{0x98,0x82,0x0E}},
  {0x11,1,{0x10}},
  {0x13,1,{0x10}},     // TSHD Rise Poisition
  {0x00,1,{0xA0}},

  {0xFF,3,{0x98,0x82,0x00}},
  {0x35,1,{0x00}},  //TE enable
  {0x11,1,{0x00}},
  {REGFLAG_DELAY, 120, {}},
  {0x29,1,{0x00}},
  {REGFLAG_DELAY, 20, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
		unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;
			case REGFLAG_UDELAY:
				UDELAY(table[i].count);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->physical_width_um = LCM_PHYSICAL_WIDTH;
	params->physical_height_um = LCM_PHYSICAL_HEIGHT;
	params->density            = LCM_DENSITY;

	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active    = 4;
	params->dsi.vertical_backporch     = 16;
	params->dsi.vertical_frontporch     = 230;
	params->dsi.vertical_active_line    = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active    = 20;
	params->dsi.horizontal_backporch    = 26;
	params->dsi.horizontal_frontporch    = 26;
	params->dsi.horizontal_active_pixel    = FRAME_WIDTH;


	params->dsi.PLL_CLOCK = 265;
	params->dsi.ssc_disable  = 1;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init_power(void)
{
	display_bias_enable();
}

static void lcm_suspend_power(void)
{
	display_bias_disable();
}

static void lcm_resume_power(void)
{
	display_bias_enable();
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(NULL, lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
}

static void lcm_resume(void)
{
	//lcm_compare_id();
	lcm_init();
}

#if 1
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static int adc_read_vol(void)
{
  int adc[1];
  int data[4] ={0,0,0,0};
  int sum = 0;
  int adc_vol=0;
  int num = 0;

  for(num=0;num<10;num++)
  {
    IMM_GetOneChannelValue(2, data, adc);
    sum+=(data[0]*100+data[1]);
  }
  adc_vol = sum/10;
#if defined(BUILD_LK)
  printf("ili9882n adc_vol is %d\n",adc_vol);
#else
  printk("ili9882n adc_vol is %d\n",adc_vol);
#endif
  return (adc_vol > 100) ? 0: 1;
}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[3];
	unsigned int array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);//Must over 6 ms

	array[0]=0x00043902;
	array[1]=0x068298ff;
	dsi_set_cmdq((unsigned int *)&array, 2, 1);
	MDELAY(10);
	read_reg_v2(0xf1, buffer, 1);
	id =buffer[0] + adc_read_vol();

#if defined(BUILD_LK)
	printf("%s,ili9882n_caihong652_xinli_hdplus1600 id = 0x%08x\n", __func__, id);
#else
	printk("%s,ili9882n_caihong652_xinli_hdplus1600 id = 0x%08x\n", __func__, id);
#endif
	return (id == 0x82 )?1:0;
}

struct LCM_DRIVER ili9882n_caihong652_xinli_hdplus1600_lcm_drv = {
	.name 				= "ili9882n_caihong652_xinli_hdplus1600",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           	= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,
	.init_power 		= lcm_init_power,
	.resume_power 		= lcm_resume_power,
	.suspend_power 		= lcm_suspend_power,
	.compare_id     	= lcm_compare_id,
};
