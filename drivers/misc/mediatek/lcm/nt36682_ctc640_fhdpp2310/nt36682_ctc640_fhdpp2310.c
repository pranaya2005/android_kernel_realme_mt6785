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

#define GPIO_LCM_RST              (45+320)

/* static unsigned char lcd_id_pins_value = 0xFF; */
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(2310)

/* physical size in um */
#define LCM_PHYSICAL_WIDTH									(68850)
#define LCM_PHYSICAL_HEIGHT									(147263)
#define LCM_DENSITY											(480)

#define REGFLAG_DELAY		    0xFFFC
#define REGFLAG_UDELAY	        0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD
#define REGFLAG_RESET_LOW	    0xFFFE
#define REGFLAG_RESET_HIGH	    0xFFFF

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[200];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_init_setting[] = {
  {0XFF,1,{0X10}},
  {0XFB,1,{0X01}},
  {0XFF,1,{0X20}},
  {0XFB,1,{0X01}},
  {0X07,1,{0X99}},
  {0X0F,1,{0XA4}},
  {0X1F,1,{0XC0}},
  {0X2F,1,{0X04}},
  {0X62,1,{0XBE}},
  {0X69,1,{0XD1}},
  {0X6D,1,{0X44}},
  {0X78,1,{0X01}},
  {0X95,1,{0XE1}},
  {0X96,1,{0XE1}},
  {0XFF,1,{0X24}},
  {0XFB,1,{0X01}},
  {0X00,1,{0X1C}},
  {0X01,1,{0X1C}},
  {0X02,1,{0X1C}},
  {0X03,1,{0X1C}},
  {0X04,1,{0X1C}},
  {0X05,1,{0X1C}},
  {0X06,1,{0X1C}},
  {0X07,1,{0X01}},
  {0X08,1,{0X24}},
  {0X09,1,{0X1C}},
  {0X0A,1,{0X1C}},
  {0X0B,1,{0X21}},
  {0X0C,1,{0X06}},
  {0X0D,1,{0X05}},
  {0X0E,1,{0X04}},
  {0X0F,1,{0X03}},
  {0X10,1,{0X0D}},
  {0X11,1,{0XA5}},
  {0X12,1,{0X1C}},
  {0X13,1,{0X1C}},
  {0X14,1,{0X1C}},
  {0X15,1,{0X1C}},
  {0X16,1,{0X1C}},
  {0X17,1,{0X1C}},
  {0X18,1,{0X1C}},
  {0X19,1,{0X1C}},
  {0X1A,1,{0X1C}},
  {0X1B,1,{0X1C}},
  {0X1C,1,{0X1C}},
  {0X1D,1,{0X1C}},
  {0X1E,1,{0X01}},
  {0X1F,1,{0X24}},
  {0X20,1,{0X1C}},
  {0X21,1,{0X1C}},
  {0X22,1,{0X21}},
  {0X23,1,{0X06}},
  {0X24,1,{0X05}},
  {0X25,1,{0X04}},
  {0X26,1,{0X03}},
  {0X27,1,{0X0D}},
  {0X28,1,{0XA5}},
  {0X29,1,{0X11}},
  {0X2A,1,{0X13}},
  {0X2B,1,{0X15}},
  {0X2D,1,{0X10}},
  {0X2F,1,{0X12}},
  {0X31,1,{0X10}},
  {0X32,1,{0X11}},
  {0X33,1,{0X00}},
  {0X35,1,{0X00}},
  {0X36,1,{0X81}},
  {0X37,1,{0X03}},
  {0X38,1,{0X6A}},
  {0X39,1,{0X6A}},
  {0X3F,1,{0X6A}},
  {0X41,1,{0X10}},
  {0X42,1,{0X11}},
  {0X4C,1,{0X1C}},
  {0X4D,1,{0X14}},
  {0X60,1,{0X90}},
  {0X61,1,{0X06}},
  {0X63,1,{0X20}},
  {0X72,1,{0X00}},
  {0X73,1,{0X00}},
  {0X74,1,{0X00}},
  {0X75,1,{0X00}},
  {0X79,1,{0X00}},
  {0X7A,1,{0X03}},
  {0X7B,1,{0X1C}},
  {0X7D,1,{0X06}},
  {0X80,1,{0X42}},
  {0X82,1,{0X13}},
  {0X83,1,{0X22}},
  {0X84,1,{0X31}},
  {0X85,1,{0X00}},
  {0X86,1,{0X00}},
  {0X87,1,{0X00}},
  {0X88,1,{0X13}},
  {0X89,1,{0X22}},
  {0X8A,1,{0X31}},
  {0X8B,1,{0X00}},
  {0X8C,1,{0X00}},
  {0X8D,1,{0X00}},
  {0X8E,1,{0XF4}},
  {0X8F,1,{0X01}},
  {0X90,1,{0X80}},
  {0X92,1,{0X7B}},
  {0X93,1,{0X0A}},
  {0X94,1,{0X40}},
  {0X9D,1,{0XB1}},
  {0XDC,1,{0X44}},
  {0XDD,1,{0X00}},
  {0XDE,1,{0X00}},
  {0XE2,1,{0X00}},
  {0XE5,1,{0X80}},
  {0XEB,1,{0X03}},
  {0XFF,1,{0X25}},
  {0XFB,1,{0X01}},
  //{0X05,1,{0X00}},
  {0X21,1,{0X1C}},
  {0X22,1,{0X1C}},
  {0X24,1,{0X7B}},
  {0X25,1,{0X7B}},
  {0X58,1,{0X01}},
  {0X59,1,{0X01}},
  {0X5A,1,{0X03}},
  {0X5B,1,{0X71}},
  {0X5C,1,{0X25}},
  {0X5E,1,{0XFF}},
  {0X5F,1,{0X10}},
  {0X68,1,{0X58}},
  {0X69,1,{0X10}},
  {0X6B,1,{0X00}},
  {0X6C,1,{0X1D}},
  {0X71,1,{0X1D}},
  {0X77,1,{0X74}},
  {0X7E,1,{0X1D}},
  {0X7F,1,{0X00}},
  {0X84,1,{0X68}},
  {0X8D,1,{0X00}},
  {0XBF,1,{0X00}},
  {0XD9,1,{0X68}},
  {0XDB,1,{0X22}},
  {0XFF,1,{0X26}},
  {0XFB,1,{0X01}},
  {0X06,1,{0XFF}},
  {0X0C,1,{0X08}},
  {0X0F,1,{0X05}},
  {0X10,1,{0X06}},
  {0X12,1,{0X00}},
  {0X15,1,{0XA4}},
  {0X19,1,{0X15}},
  {0X1A,1,{0X61}},
  {0X1C,1,{0XAF}},
  {0X1D,1,{0X14}},
  {0X1E,1,{0XE4}},
  {0X98,1,{0XF1}},
  {0XA9,1,{0X11}},
  {0XAA,1,{0X11}},
  {0XAE,1,{0X8A}},
  {0XFF,1,{0X27}},
  {0XFB,1,{0X01}},
  {0X13,1,{0X00}},
  {0X1E,1,{0X14}},
  {0XD2,1,{0X3F}},
  {0XD5,1,{0X33}},
  {0XD7,1,{0X35}},
  {0XD9,1,{0X18}},
  {0XFF,1,{0XF0}},
  {0XFB,1,{0X01}},
  {0XA2,1,{0X00}},
  {0XFF,1,{0X20}},
  {0XFB,1,{0X01}},
  {0XFF,1,{0X21}},
  {0XFB,1,{0X01}},
  {0XFF,1,{0X20}},
  {0XFB,1,{0X01}},
  {0XFF,1,{0X10}},
  {0X51,2,{0XFF,0XFF}},
  {0X53,1,{0X24}},
  {0x11,0,{}},
  {REGFLAG_DELAY,120, {}},
  {0x29,0,{}},
  {REGFLAG_DELAY,30, {}},
  {0XFF,1,{0X10}},
  {REGFLAG_END_OF_TABLE, 0x00, {} }
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

	params->dsi.mode = SYNC_PULSE_VDO_MODE;//SYNC_PULSE_VDO_MODE  SYNC_EVENT_VDO_MODE BURST_VDO_MODE
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

	params->dsi.vertical_sync_active = 10;
	params->dsi.vertical_backporch = 54;
	params->dsi.vertical_frontporch = 10;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 12;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 535;
	params->dsi.ssc_disable = 1;
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
	gpio_set_value_cansleep(GPIO_LCM_RST, 1);
	MDELAY(10);
	gpio_set_value_cansleep(GPIO_LCM_RST, 0);
	MDELAY(10);
	gpio_set_value_cansleep(GPIO_LCM_RST, 1);
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
	lcm_init();
}

static struct LCM_setting_table lcm_compare[] = {
  {0xFF, 1, {0x20} },
  {REGFLAG_DELAY, 20, {} },
  {0xFB, 1, {0x01} },
  {REGFLAG_DELAY, 120, {} }
};

static unsigned int lcm_compare_id(void)
{
  unsigned int version_id;
  unsigned char buffer[2];
  unsigned int array[16];

  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 0);
  MDELAY(10);
  gpio_set_value_cansleep(GPIO_LCM_RST, 1);
  MDELAY(120);

  array[0] = 0x00043700;  /* read id return two byte,version and id */
  dsi_set_cmdq(array, 1, 1);
  MDELAY(2); 

  push_table(NULL, lcm_compare, sizeof(lcm_compare) / sizeof(struct LCM_setting_table), 1);
  read_reg_v2(0x3B, buffer, 1);
  version_id = buffer[0];

  LCM_LOGI("nt36682 ctc %s,version_id=0x%x\n", __func__, version_id);
  return (0x2A == version_id)?1:0;
}

struct LCM_DRIVER nt36682_ctc640_fhdpp2310_lcm_drv = {
	.name = "nt36682_ctc640_fhdpp2310",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
    .compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};
