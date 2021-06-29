/* drivers/hwmon/mt6516/amit/IQS128.c - IQS128/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>

#include "upmu_common.h"
#include <linux/io.h>
#include "tusb320.h"

/******************************************************************************
 * configuration
*******************************************************************************/
#define TUSB320_DEV_NAME     "TUSB320"

static struct i2c_client *tusb320_i2c_client;
static const struct i2c_device_id tusb320_i2c_id[] = { {"TUSB320", 0}, {} };
//static struct i2c_board_info i2c_TUSB320 __initdata = { I2C_BOARD_INFO("TUSB320", 0x60) };
static bool tusb320_mode = 0;
static int tusb320_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char devicve_id[1];
    bool ret = 0;
	pr_debug("tusb320_i2c_probe\n");

	tusb320_i2c_client = client;

	ret = tusb320_read_byte(0x01, &devicve_id[0]);
    if(ret == 0)
    {
	pr_warn("tusb320 is not use i2c mode \n");
    tusb320_mode = 1;
    }
#if 0
	pr_warn("tusb320_i2c_probe ID1=%x\n", devicve_id[0]);
	ret = tusb320_read_byte(0x02, &devicve_id[0]);
	pr_warn("tusb320_i2c_probe ID2=%x\n", devicve_id[0]);
	ret = tusb320_read_byte(0x03, &devicve_id[0]);
	pr_warn("tusb320_i2c_probe ID3=%x\n", devicve_id[0]);
	ret = tusb320_read_byte(0x04, &devicve_id[0]);
	pr_warn("tusb320_i2c_probe ID4=%x\n", devicve_id[0]);
	ret = tusb320_read_byte(0x05, &devicve_id[0]);
	pr_warn("tusb320_i2c_probe ID5=%x\n", devicve_id[0]);
	ret = tusb320_read_byte(0x06, &devicve_id[0]);
	pr_warn("tusb320_i2c_probe ID6=%x\n", devicve_id[0]);
    //tusb320_clear_int();
#endif
	return 0;
}

static int tusb320_i2c_remove(struct i2c_client *client)
{
	pr_debug("TUSB320_i2c_remove\n");

	tusb320_i2c_client = NULL;

	return 0;
}
/*
static int tusb320_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	pr_debug("TUSB320_i2c_suspend\n");

	return 0;
}

static int tusb320_i2c_resume(struct i2c_client *client)
{
	pr_debug("TUSB320_i2c_resume\n");

	return 0;
}
*/
//#ifdef CONFIG_OF
static const struct of_device_id tusb320_of_match[] = {
    {.compatible = "mediatek,TUSB320"},
    {},
};
//#endif

static struct i2c_driver tusb320_i2c_driver = {
	.probe = tusb320_i2c_probe,
	.remove = tusb320_i2c_remove,
	//.suspend = tusb320_i2c_suspend,
	//.resume = tusb320_i2c_resume,
	.id_table = tusb320_i2c_id,
	.driver = {
		   .name = TUSB320_DEV_NAME,
//#ifdef CONFIG_OF
        .of_match_table = tusb320_of_match,
//#endif
		   },
};

static DEFINE_MUTEX(tusb320_i2c_access);

void tusb320_clear_int(void)
{
  unsigned char status;
  int ret = 0;
    if(tusb320_mode == 1)
      return ;
  ret = tusb320_read_byte(0x09, &status);
  //pr_debug("tusb320_clear_int before 0x09 reg val =%x\n",status);
  if(status&0x10)
    tusb320_write_byte(0x09,(0x10));
  ret = tusb320_read_byte(0x09, &status);
  //pr_debug("tusb320_clear_int after 0x09 reg val =%x\n",status);
  return;
}
int tusb320_read_byte(unsigned char cmd, unsigned char *returnData)
{
	//char cmd_buf[1] = { 0x00 };
	//char readData = 0;
	int ret = 0;

    if(tusb320_mode == 1)
      return 0;
    if(tusb320_i2c_client == NULL)
      return 0;
#if 1
    ret = i2c_master_send(tusb320_i2c_client, &cmd, 1);
    if(ret < 0)
{
       pr_warn("tusb320 write erro ret val =%x\n",ret);
		return 0;
} 
   ret = i2c_master_recv(tusb320_i2c_client, returnData, 1);
    if(ret < 0)
       pr_warn("tusb320 read erro ret val =%x\n",ret);
#endif
#if 0
	mutex_lock(&tusb320_i2c_access);

	/*new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;*/
	tusb320_i2c_client->ext_flag =
	    ((tusb320_i2c_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(tusb320_i2c_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/*new_client->addr = new_client->addr & I2C_MASK_FLAG;*/
		tusb320_i2c_client->ext_flag = 0;

		mutex_unlock(&tusb320_i2c_access);
       pr_warn("tusb320 read erro ret val =%x\n",ret);
		return 0;
	}
	readData = cmd_buf[0];
	*returnData = readData;

	/*new_client->addr = new_client->addr & I2C_MASK_FLAG;*/
	tusb320_i2c_client->ext_flag = 0;

	mutex_unlock(&tusb320_i2c_access);
#endif
	return 1;
}

int tusb320_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

    if(tusb320_mode == 1)
      return 0;
    if(tusb320_i2c_client == NULL)
      return 0;
	mutex_lock(&tusb320_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	//tusb320_i2c_client->ext_flag = ((tusb320_i2c_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(tusb320_i2c_client, write_data, 2);
	if (ret < 0) {
		//tusb320_i2c_client->ext_flag = 0;
		mutex_unlock(&tusb320_i2c_access);
		return 0;
	}

	//tusb320_i2c_client->ext_flag = 0;
	mutex_unlock(&tusb320_i2c_access);
	return 1;
}

/******************************************************************************
 * extern functions
*******************************************************************************/
/*----------------------------------------------------------------------------*/
static int tusb320_probe(struct platform_device *pdev)
{

	if (i2c_add_driver(&tusb320_i2c_driver)) {
		pr_err("tusb320 add driver error\n");
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int tusb320_remove(struct platform_device *pdev)
{

	i2c_del_driver(&tusb320_i2c_driver);

	return 0;
}

/*----------------------------------------------------------------------------*/
static const struct of_device_id audio_switch_of_match[] = {
	{.compatible = "mediatek,audio_switch",},
	{},
};

static struct platform_driver tusb320_audio_switch_driver = {
	.probe = tusb320_probe,
	.remove = tusb320_remove,
	.driver = {
		   .name = "audio_switch",
		   .of_match_table = audio_switch_of_match,
		   }
};

/*----------------------------------------------------------------------------*/
static int __init tusb320_init(void)
{

	//i2c_register_board_info(3, &i2c_TUSB320, 1);

	if (platform_driver_register(&tusb320_audio_switch_driver)) {
		pr_err("tusb320 failed to register driver");
		return -ENODEV;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit tusb320_exit(void)
{
	pr_warn("tusb320_exit\n");
}

/*----------------------------------------------------------------------------*/
module_init(tusb320_init);
module_exit(tusb320_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("TUSB320 driver");
MODULE_LICENSE("GPL");
