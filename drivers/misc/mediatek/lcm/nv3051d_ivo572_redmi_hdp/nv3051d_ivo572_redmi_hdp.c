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
#define FRAME_HEIGHT									(1440)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH								(67930)
#define LCM_PHYSICAL_HEIGHT								(150960)
#define LCM_DENSITY										(320)

#define REGFLAG_DELAY		    0xFFFC
#define REGFLAG_UDELAY	        0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	    0xFFFE
#define REGFLAG_RESET_HIGH	    0xFFFF
#define GPIO_LCM_RST_1         (GPIO45 | 0x80000000)
#define GPIO_LCM_RST           (45+320)

#ifdef BUILD_LK
static void lcm_set_rst_lk(int output)
{
    mt_set_gpio_mode(GPIO_LCM_RST_1, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST_1, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_RST_1, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
#endif

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
	{0xFF, 1, {0x30}},
	{0xFF, 1, {0x52}},
	{0xFF, 1, {0x01}},
	{0xE3, 1, {0x00}},
	{0x03, 1, {0x02}},
	{0x04, 1, {0x01}},
	{0x05, 1, {0x30}},
	{0x24, 1, {0x08}},
	{0x25, 1, {0x06}},
	{0x28, 1, {0x47}},
	{0x29, 1, {0xc5}},
	{0x2A, 1, {0x9f}},
	{0x38, 1, {0x9C}},
	{0x39, 1, {0xA7}},
	{0x3A, 1, {0x2B}},
	{0x44, 1, {0x00}},
	{0x49, 1, {0x3C}},
	{0x59, 1, {0xfe}},
	{0x5c, 1, {0x00}},
	{0x6D, 1, {0x00}},
	{0x6E, 1, {0x00}},
	{0x91, 1, {0x77}},
	{0x92, 1, {0x77}},
	{0x99, 1, {0x54}},
	{0x9B, 1, {0x56}},
	{0xA0, 1, {0x55}},
	{0xA1, 1, {0x50}},
	{0xA4, 1, {0x9C}},
	{0xA7, 1, {0x02}},
	{0xA8, 1, {0x08}},
	{0xA9, 1, {0x01}},
	{0xAA, 1, {0xfc}},
	{0xAB, 1, {0x42}},
	{0xAC, 1, {0x06}},
	{0xAD, 1, {0x06}},
	{0xAE, 1, {0x06}},
	{0xAF, 1, {0x03}},
	{0xB0, 1, {0x08}},
	{0xB1, 1, {0x40}},
	{0xB2, 1, {0x42}},
	{0xB3, 1, {0x42}},
	{0xB4, 1, {0x03}},
	{0xB5, 1, {0x08}},
	{0xB6, 1, {0x40}},
	{0xB7, 1, {0x08}},
	{0xB8, 1, {0x40}},
	{0xF0, 1, {0x00}},
	{0xF6, 1, {0xC0}},
	{0xFF, 1, {0x30}},
	{0xFF, 1, {0x52}},
	{0xFF, 1, {0x02}},
	{0xB1, 1, {0x15}},
	{0xD1, 1, {0x0B}},
	{0xB4, 1, {0x2E}},
	{0xD4, 1, {0x38}},
	{0xB2, 1, {0x18}},
	{0xD2, 1, {0x0E}},
	{0xB3, 1, {0x37}},
	{0xD3, 1, {0x31}},
	{0xB6, 1, {0x28}},
	{0xD6, 1, {0x24}},
	{0xB7, 1, {0x41}},
	{0xD7, 1, {0x3F}},
	{0xC1, 1, {0x08}},
	{0xE1, 1, {0x08}},
	{0xB8, 1, {0x0E}},
	{0xD8, 1, {0x0E}},
	{0xB9, 1, {0x04}},
	{0xD9, 1, {0x06}},
	{0xBD, 1, {0x13}},
	{0xDD, 1, {0x13}},
	{0xBC, 1, {0x11}},
	{0xDC, 1, {0x11}},
	{0xBB, 1, {0x0F}},
	{0xDB, 1, {0x11}},
	{0xBA, 1, {0x11}},
	{0xDA, 1, {0x11}},
	{0xBE, 1, {0x19}},
	{0xDE, 1, {0x1B}},
	{0xBF, 1, {0x0F}},
	{0xDF, 1, {0x13}},
	{0xC0, 1, {0x16}},
	{0xE0, 1, {0x18}},
	{0xB5, 1, {0x3A}},
	{0xD5, 1, {0x37}},
	{0xB0, 1, {0x02}},
	{0xD0, 1, {0x07}},
	{0xFF, 1, {0x30}},
	{0xFF, 1, {0x52}},
	{0xFF, 1, {0x03}},  
	{0x04, 1, {0x51}},
	{0x05, 1, {0xc0}},
	{0x06, 1, {0xc0}},
	{0x08, 1, {0x87}},
	{0x09, 1, {0x86}},
	{0x0a, 1, {0x85}},
	{0x0b, 1, {0x84}},
	{0x24, 1, {0x51}},
	{0x25, 1, {0xc0}},
	{0x26, 1, {0xc0}},
	{0x2a, 1, {0xa2}},
	{0x2b, 1, {0xa2}},
	{0x2c, 1, {0xa2}},
	{0x2d, 1, {0xa2}},
	{0x34, 1, {0xa1}}, 
	{0x35, 1, {0x53}}, 
	{0x36, 1, {0x53}}, 
	{0x37, 1, {0x13}}, 
	{0x40, 1, {0x85}}, 
	{0x41, 1, {0x84}}, 
	{0x42, 1, {0x83}}, 
	{0x43, 1, {0x82}},
	{0x45, 1, {0xa0}}, 
	{0x46, 1, {0xa1}}, 
	{0x48, 1, {0xa2}}, 
	{0x49, 1, {0xa3}}, 
	{0x50, 1, {0x81}}, 
	{0x51, 1, {0x00}}, 
	{0x52, 1, {0x01}}, 
	{0x53, 1, {0x02}}, 
	{0x55, 1, {0xa5}}, 
	{0x56, 1, {0xa6}}, 
	{0x58, 1, {0xa7}}, 
	{0x59, 1, {0xa7}}, 
	{0x80, 1, {0x03}}, 
	{0x81, 1, {0x02}}, 
	{0x82, 1, {0x07}}, 
	{0x83, 1, {0x06}}, 
	{0x84, 1, {0x05}}, 
	{0x85, 1, {0x04}}, 
	{0x92, 1, {0x00}}, 
	{0x93, 1, {0x01}}, 
	{0x96, 1, {0x03}}, 
	{0x97, 1, {0x02}}, 
	{0x98, 1, {0x07}}, 
	{0x99, 1, {0x06}}, 
	{0x9A, 1, {0x05}}, 
	{0x9B, 1, {0x04}}, 
	{0xA8, 1, {0x00}}, 
	{0xA9, 1, {0x01}}, 
	{0xFF, 1, {0x30}},
	{0xFF, 1, {0x52}},
	{0xFF, 1, {0x02}}, 
	{0x01, 1, {0x01}},
	{0x02, 1, {0xDA}},
	{0x03, 1, {0xBA}},
	{0x04, 1, {0xA8}},
	{0x05, 1, {0x9A}},
	{0x06, 1, {0x70}},
	{0x07, 1, {0xFF}},
	{0x08, 1, {0x91}},
	{0x09, 1, {0x90}},
	{0x0A, 1, {0xFF}},
	{0x0B, 1, {0x8F}},
	{0x0C, 1, {0x60}},
	{0x0D, 1, {0x58}},
	{0x0E, 1, {0x48}},
	{0x0F, 1, {0x38}},
	{0x10, 1, {0x2B}},
	{0xFF, 1, {0x30}}, 
	{0xFF, 1, {0x52}},
	{0xFF, 1, {0x00}},
	{0x36, 1, {0x02}},
	{0x53, 1, {0x2c}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 1, {0x00}},
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

	params->dsi.vertical_sync_active    = 20;
	params->dsi.vertical_backporch     = 32;
	params->dsi.vertical_frontporch     = 56;
	params->dsi.vertical_active_line    = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active    = 30;
	params->dsi.horizontal_backporch    = 80;
	params->dsi.horizontal_frontporch    = 80;
	params->dsi.horizontal_active_pixel    = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 230;
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
#ifdef BUILD_LK
  lcm_set_rst_lk(1);
  MDELAY(10);
  lcm_set_rst_lk(0);
  MDELAY(20);
  lcm_set_rst_lk(1);
  MDELAY(120);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(50);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(120);
#endif

	push_table(NULL, lcm_init_setting, sizeof(lcm_init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(NULL, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
#ifdef BUILD_LK
  lcm_set_rst_lk(0);
#else
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
#endif
}

static void lcm_resume(void)
{
	lcm_init();
}

/*static unsigned int lcm_compare_id(void)
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
	array[1]=0x068198ff;
	dsi_set_cmdq((unsigned int *)&array, 2, 1);
	MDELAY(10);
	read_reg_v2(0xf0, buffer, 1);
	id =buffer[0] ;

#if defined(BUILD_LK)
	printf("%s,nv3051d_ivo572_redmi_hdp id = 0x%08x\n", __func__, id);
#else
	printk("%s,nv3051d_ivo572_redmi_hdp id = 0x%08x\n", __func__, id);
#endif
	return (LCM_ID == id)?1:0;
}*/

struct LCM_DRIVER nv3051d_ivo572_redmi_hdp_lcm_drv = {
	.name 				= "nv3051d_ivo572_redmi_hdp",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           	= lcm_init,
	.suspend        	= lcm_suspend,
	.resume         	= lcm_resume,
	.init_power 		= lcm_init_power,
	.resume_power 		= lcm_resume_power,
	.suspend_power 		= lcm_suspend_power,
	//.compare_id     	= lcm_compare_id,
};
