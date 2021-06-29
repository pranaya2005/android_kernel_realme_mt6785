#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>

//add for pwm and gpio control
#include <mt-plat/mtk_pwm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
// the eng

#ifdef CONFIG_OF
/* device tree */
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#ifdef CONFIG_MTK_CCU
#include "ccu_imgsensor_if.h"
#endif

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_imgsensor_errcode.h"

#include "imgsensor_cfg_table.h"
#include "imgsensor_sensor_list.h"
#include "imgsensor_hw.h"
#include "imgsensor_i2c.h"
#include "imgsensor_proc.h"
#ifdef IMGSENSOR_OC_ENABLE
#include "imgsensor_oc.h"
#endif
#include "imgsensor.h"

#if defined(CONFIG_MTK_CAM_SECURE_I2C)
#include "imgsensor_ca.h"
#endif


//static int envm_gpio = 0;
//static int endrv_gpio = 0;
//static int dir_gpio = 0;
//static unsigned int step_gop;
//add for gpio control
//struct pinctrl *pinctrl_moto;
//struct pinctrl_state *envm_output0, *envm_output1, *endrv_output0, *endrv_output1, *dir_output0, *dir_output1, *step_output0, *step_output1;

#define GPIO_EN_VM          (25 + 320)
#define GPIO_EN_DRV8846         (28 + 320)
#define GPIO_DIR            (26 + 320)
#define GPIO_STEP_PWM           (5 + 320)
extern void m1120_down_enable(int enable);
extern void m1120_up_enable(int enable);
extern int m1120_down_measure(short *raw);
extern int m1120_up_measure(short *raw);
void report_home_key_press(void);
void report_home_key_up(void);

static /*volatile*/ wait_queue_head_t up_waitqueue;
static /*volatile*/ wait_queue_head_t down_waitqueue;
//static /*volatile*/ wait_queue_head_t press_wait;

static void subcam_up_wq_routine(struct work_struct *);
struct delayed_work subcam_up_wq;
static void subcam_down_wq_routine(struct work_struct *);
struct delayed_work subcam_down_wq;

static void subcam_press_wq_routine(struct work_struct *);
struct delayed_work subcam_press_wq;
static short upl=0,uph=-130,downl=-121,downh=-2;
static short vibx0=0,vibx1=95,viby0=0,viby1=0;
static DEFINE_SPINLOCK(moto_drv_lock);
static short moto_status=0;
static up_cnt_timeout = 20;
static down_cnt_timeout = 20;
//the end
static void step_control_drv8846(void)
{

    struct pwm_spec_config pwm_setting;
    pwm_setting.pwm_no = PWM3; //PWM_A
    pwm_setting.mode = PWM_MODE_FIFO; //new mode fifo and periodical mode
    pwm_setting.pmic_pad = 0;
    pwm_setting.clk_div = CLK_DIV8;
    pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK; //26MHz


    pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 41;
    pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 41;
    pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 1; //idle High
    pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
    pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 1;
    pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
    pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;

    pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xAAAAAAAA;
    pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0xAAAAAAAA;
    pwm_set_spec_config(&pwm_setting);
}
int vib_test_start(int *time)
{

  if(*time)
  {
    // PK_DBG("vib test enable");
    //*time = 0;
    do{
      //PK_DBG("vib test en = %d\n", *en);
      //up
        gpio_direction_output(GPIO_EN_VM, 1);
      mdelay(1);
        gpio_direction_output(GPIO_DIR, 1);
        gpio_direction_output(GPIO_EN_DRV8846, 0);
      udelay(1);
      step_control_drv8846();

      mdelay(1200);
        gpio_direction_output(GPIO_EN_DRV8846, 1);
        gpio_direction_output(GPIO_EN_VM, 0);
      mt_pwm_disable(PWM3,false);
      mdelay(10);
      //down
        gpio_direction_output(GPIO_EN_VM, 1);
      mdelay(1);
        gpio_direction_output(GPIO_DIR, 0);
        gpio_direction_output(GPIO_EN_DRV8846, 0);
      udelay(1);
      step_control_drv8846();
      mdelay(1200);
        gpio_direction_output(GPIO_EN_DRV8846, 1);
        gpio_direction_output(GPIO_EN_VM, 0);
      mt_pwm_disable(PWM3,false);
      mdelay(10);
      (*time)++;
      //PK_DBG("vib test time = %d\n", *time);
      if(*time == 0)
        return 0;
    }while(1);
  }else
  {
    //PK_DBG("vib test disable");
    //*en == 0;
    //up
        gpio_direction_output(GPIO_EN_VM, 1);
    mdelay(1);
        gpio_direction_output(GPIO_DIR, 1);
        gpio_direction_output(GPIO_EN_DRV8846, 0);
    udelay(1);
    step_control_drv8846();

    mdelay(1200);
        gpio_direction_output(GPIO_EN_DRV8846, 1);
        gpio_direction_output(GPIO_EN_VM, 0);
    mdelay(10);
    //down
        gpio_direction_output(GPIO_EN_VM, 1);
    mdelay(1);
        gpio_direction_output(GPIO_DIR, 0);
        gpio_direction_output(GPIO_EN_DRV8846, 0);
    udelay(1);
    step_control_drv8846();
    mdelay(1200);
        gpio_direction_output(GPIO_EN_DRV8846, 1);
        gpio_direction_output(GPIO_EN_VM, 0);
    mt_pwm_disable(PWM3,false);
    mdelay(10);
  }

  return 0;
}
void vib_up_start()
{
  wait_event_interruptible_timeout(up_waitqueue,(moto_status == 0),1200);
  gpio_direction_output(GPIO_EN_VM, 1);
  mdelay(5);
  gpio_direction_output(GPIO_DIR, 1);
  gpio_direction_output(GPIO_EN_DRV8846, 0);
  udelay(10);
  step_control_drv8846();
  spin_lock(&moto_drv_lock);
  moto_status = 1;
  spin_unlock(&moto_drv_lock);
}
void vib_down_start()
{
  wait_event_interruptible_timeout(down_waitqueue,(moto_status == 0),1200);
  gpio_direction_output(GPIO_EN_VM, 1);
  mdelay(5);
  gpio_direction_output(GPIO_DIR, 0);
  gpio_direction_output(GPIO_EN_DRV8846, 0);
  udelay(10);
  step_control_drv8846();
  spin_lock(&moto_drv_lock);
  moto_status = -1;
  spin_unlock(&moto_drv_lock);
}
void vib_stop()
{
  gpio_direction_output(GPIO_EN_DRV8846, 1);
  gpio_direction_output(GPIO_EN_VM, 0);
  mt_pwm_disable(PWM3,false);
  //mdelay(5);
  spin_lock(&moto_drv_lock);
  moto_status = 0;
  spin_unlock(&moto_drv_lock);
}
void hall_enable(int en)
{
    m1120_up_enable(en);
    m1120_down_enable(en);
}
void listen_up_moto(unsigned long delay)
{
  //short upl=0,uph=-130,downl=-121,downh=-2;
  int i = 0;
  short up[5]={0};
  short down[5]={0};
  //if(first_open == 0)
  if((viby0 == 0) || (viby1 == 0))
  {
    vib_down_start();
    mdelay(1200);
    vib_stop();
    hall_enable(1);
    mdelay(20);
    for(i=0;i<5;i++)
    {
      m1120_down_measure(&down[i]);
      m1120_up_measure(&up[i]);
      mdelay(20);
      //PK_DBG("i=%d,down=%d up%d\n",i,down[i],up[i]);
    }
    downl=(down[0]+down[1]+down[2]+down[3]+down[4])/5;
    upl=(up[0]+up[1]+up[2]+up[3]+up[4])/5;
    PK_DBG("downl=%d upl=%d\n",downl,upl);
    viby0 = downl -upl;
    vib_up_start();
    mdelay(1200);
    for(i=0;i<5;i++)
    {
      m1120_down_measure(&down[i]);
      m1120_up_measure(&up[i]);
      mdelay(20);
      //PK_DBG("i=%d,down=%d up%d\n",i,down[i],up[i]);
    }
    downh=(down[0]+down[1]+down[2]+down[3]+down[4])/5;
    uph=(up[0]+up[1]+up[2]+up[3]+up[4])/5;
    PK_DBG("downh=%d uph=%d\n",downh,uph);
    viby1 = downh -uph;
    vib_stop();
    //first_open = 1;
    schedule_delayed_work(&subcam_press_wq, 200);
  }
  else{
    cancel_delayed_work(&subcam_press_wq);
    vib_up_start();
    hall_enable(1);
    mdelay(600);
    schedule_delayed_work(&subcam_up_wq, 50);
  }
}
static void subcam_up_wq_routine(struct work_struct *data)
{
  short hal_up=0,hal_down=0;
  short dis = 0,up_d =0;
  spin_lock(&moto_drv_lock);
  spin_unlock(&moto_drv_lock);
  m1120_down_measure(&hal_down);
  m1120_up_measure(&hal_up);
  dis = hal_down - hal_up;
  PK_DBG("---up dis is %d\n",dis);
  if(viby1>0)
  up_d = dis - viby1 + 5;
  else
  up_d= viby1-dis + 5;

  if((up_d >=0)&&(up_cnt_timeout<=17))
  {
    vib_stop();
    wake_up_interruptible(&down_waitqueue);
    PK_DBG("-up_cnt_timeout= %d\n",up_cnt_timeout);
    spin_lock(&moto_drv_lock);
    up_cnt_timeout=20;
    spin_unlock(&moto_drv_lock);
    schedule_delayed_work(&subcam_press_wq, 200);
  }
  else if(up_cnt_timeout<=17)
  {
    vib_stop();
    wake_up_interruptible(&down_waitqueue);
    PK_DBG("-up_cnt_timeout= %d\n",up_cnt_timeout);
    spin_lock(&moto_drv_lock);
    up_cnt_timeout=20;
    spin_unlock(&moto_drv_lock);
    schedule_delayed_work(&subcam_press_wq, 200);
  }
  else
  {
    cancel_delayed_work(&subcam_press_wq);
    //cancel_delayed_work(&subcam_down_wq);
    //vib_up_start();
    schedule_delayed_work(&subcam_up_wq, 50);
  }
  up_cnt_timeout--;
}
void listen_down_moto(unsigned long delay)
{
  short hal_up=0,hal_down=0;
  short dis = 0,down_d = 0;
  cancel_delayed_work(&subcam_press_wq);
  hall_enable(1);
  mdelay(20);
  m1120_down_measure(&hal_down);
  m1120_up_measure(&hal_up);
  dis = hal_down - hal_up;
  PK_DBG("---down dis0 is %d\n",dis);
  if(viby0>0)
  down_d = dis - viby0;
  else
  down_d= viby0 -dis;
  if(down_d >=0)
  {
    //cancel_delayed_work(&subcam_down_wq);
    vib_stop();
    hall_enable(0);
  }
  else
  {
    vib_down_start();
    schedule_delayed_work(&subcam_down_wq, 50);
  }
}
static void subcam_down_wq_routine(struct work_struct *data)
{
  short hal_up=0,hal_down=0;
  short dis = 0,down_d = 0;
  cancel_delayed_work(&subcam_press_wq);
  spin_lock(&moto_drv_lock);
  spin_unlock(&moto_drv_lock);
  m1120_down_measure(&hal_down);
  m1120_up_measure(&hal_up);
  dis = hal_down - hal_up;
  PK_DBG("---down dis0 is,viby0= %d %d\n",dis,viby0);
  if(viby0>0)
  down_d = dis - viby0 + 2;
  else
  down_d = viby0 -dis + 2;
  if(((down_d >=0))&&(down_cnt_timeout <=14))
  {
    mdelay(20);
    vib_stop();
    wake_up_interruptible(&up_waitqueue);
    PK_DBG("-down_cnt_timeout= %d\n",down_cnt_timeout);
    //cancel_delayed_work(&subcam_down_wq);
    spin_lock(&moto_drv_lock);
    down_cnt_timeout=20;
    spin_unlock(&moto_drv_lock);
    hall_enable(0);
  }
  else if(down_cnt_timeout <=14)
  {
    mdelay(100);
    vib_stop();
    wake_up_interruptible(&up_waitqueue);
    PK_DBG("-down_cnt_timeout= %d\n",down_cnt_timeout);
    //cancel_delayed_work(&subcam_down_wq);
    spin_lock(&moto_drv_lock);
    down_cnt_timeout=20;
    spin_unlock(&moto_drv_lock);
    hall_enable(0);
  }
  else
 {
    cancel_delayed_work(&subcam_press_wq);
    schedule_delayed_work(&subcam_down_wq, 50);
  }
  down_cnt_timeout--;
}
static void subcam_press_wq_routine(struct work_struct *data)
{
  short hal_up=0,hal_down=0;
  short dis = 0,am=0,p_data;
  PK_DBG("---viby1=%d viby0=%d\n",viby1,viby0);
  am = viby1- (25*(viby1-viby0))/(vibx1-vibx0);
  PK_DBG("---press am dis is %d\n",am);
  m1120_down_measure(&hal_down);
  m1120_up_measure(&hal_up);
  dis = hal_down - hal_up;
  if(am > 0)
  p_data = am -dis;
  else
  p_data = dis - am;
  PK_DBG("---press dis is %d\n",dis);
  if(p_data > 0)
  {
    report_home_key_press();
    report_home_key_up();
    //schedule_delayed_work(&subcam_press_wq, 100);
    cancel_delayed_work(&subcam_press_wq);
  }
  else
  {
    schedule_delayed_work(&subcam_press_wq, 100);
  }
}
int test_time = 0;
//int vib_enable = 0;
static ssize_t show_vib_test_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", test_time);
    return 0;
}
static ssize_t store_vib_test_value(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&test_time);
    PK_DBG("%s\n",__func__);
    vib_test_start(&test_time);
    return count;
}
static ssize_t show_vib_up(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", 0);
    return 0;
}
static ssize_t store_vib_up(struct device_driver *ddri, const char *buf, size_t count)
{
    int up_en = 0;
    sscanf(buf, "%d",&up_en);
    vib_up_start();
    PK_DBG("%s\n",__func__);
    return count;
}
static ssize_t show_vib_down(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", 0);
    return 0;
}
static ssize_t store_vib_down(struct device_driver *ddri, const char *buf, size_t count)
{
  int down_en = 0;
    sscanf(buf, "%d",&down_en);
    vib_down_start();
    PK_DBG("%s\n",__func__);
    return count;
}
static ssize_t show_vib_stop(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", 0);
    return 0;
}
static ssize_t store_vib_stop(struct device_driver *ddri, const char *buf, size_t count)
{
    int stop_en = 0;
    sscanf(buf, "%d",&stop_en);
    vib_stop();
    PK_DBG("%s\n",__func__);
    return count;
}
static ssize_t show_vib_cal(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d %d\n", viby0, viby1);
    return 0;
}
static ssize_t store_vib_cal(struct device_driver *ddri, const char *buf, size_t count)
{
    short y0=0,y1=0;
    sscanf(buf, "%d %d",&y0, &y1);
    PK_DBG("%s,y0=%d,y1=%d\n",__func__,y0,y1);
    viby0=y0;
    viby1=y1;
    return count;
}
static DRIVER_ATTR(vib_test,   S_IWUSR | S_IRUGO, show_vib_test_value, store_vib_test_value);
static DRIVER_ATTR(vib_up,   S_IWUSR | S_IRUGO, show_vib_up, store_vib_up);
static DRIVER_ATTR(vib_down,   S_IWUSR | S_IRUGO, show_vib_down, store_vib_down);
static DRIVER_ATTR(vib_stop,   S_IWUSR | S_IRUGO, show_vib_stop, store_vib_stop);
static DRIVER_ATTR(vib_cal,   S_IWUSR | S_IRUGO, show_vib_cal, store_vib_cal);

static struct driver_attribute *vib_val[] = {
  &driver_attr_vib_test,
  &driver_attr_vib_up,
  &driver_attr_vib_down,
  &driver_attr_vib_stop,
  &driver_attr_vib_cal,
};
int modo_get_gpio_info(struct platform_device *pdev)
{
#if 0
  int ret;
  pr_debug("[tpd %d] moto_tpd_pinctrl+++++++++++++++++\n", pdev->id);
  pinctrl_moto = devm_pinctrl_get(&pdev->dev);
  if (IS_ERR(pinctrl_moto)) {
    ret = PTR_ERR(pinctrl_moto);
    dev_info(&pdev->dev, "fwq Cannot find pinctrl1!\n");
    return ret;
  }
  envm_output0 = pinctrl_lookup_state(pinctrl_moto, "envm_output0");
  if (IS_ERR(envm_output0)) {
    ret = PTR_ERR(envm_output0);
    pr_debug("Cannot find pinctrl envm_output0!\n");
    return ret;
  }
  envm_output1 = pinctrl_lookup_state(pinctrl_moto, "envm_output1");
  if (IS_ERR(envm_output1)) {
    ret = PTR_ERR(envm_output1);
    pr_debug("Cannot find pinctrl envm_output1!\n");
    return ret;
  }
  endrv_output0 = pinctrl_lookup_state(pinctrl_moto, "endrv_output0");
  if (IS_ERR(endrv_output0)) {
    ret = PTR_ERR(endrv_output0);
    pr_debug("Cannot find pinctrl endrv_output0!\n");
    return ret;
  }
  endrv_output1 = pinctrl_lookup_state(pinctrl_moto, "endrv_output1");
  if (IS_ERR(endrv_output1)) {
    ret = PTR_ERR(endrv_output1);
    pr_debug("Cannot find pinctrl endrv_output1!\n");
    return ret;
  }
  dir_output0 = pinctrl_lookup_state(pinctrl_moto, "dir_output0");
  if (IS_ERR(dir_output0)) {
    ret = PTR_ERR(dir_output0);
    pr_debug("Cannot find pinctrl dir_output0!\n");
    return ret;
  }
  dir_output1 = pinctrl_lookup_state(pinctrl_moto, "dir_output1");
  if (IS_ERR(dir_output1)) {
    ret = PTR_ERR(dir_output1);
    pr_debug("Cannot find pinctrl dir_output1!\n");
    return ret;
  }
#endif
    return 0;
}
static int moto_probe(struct platform_device *pplatform_device)
{
  modo_get_gpio_info(pplatform_device);
  return 0;
};
static int moto_remove(struct platform_device *pplatform_device)
{
    return 0;
};
static int moto_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int moto_resume(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id moto_of_device_id[] = {
    {.compatible = "mediatek,moto",},
    {}
};
#endif

static struct platform_driver moto_platform_driver = {
    .probe   = moto_probe,
    .remove  = moto_remove,
    .suspend = moto_suspend,
    .resume  = moto_resume,
    .driver  = {
           .name  = "sensor_moto",
           .owner = THIS_MODULE,
#ifdef CONFIG_OF
           .of_match_table = moto_of_device_id,
#endif
        }
};
static int __init moto_init(void)
{
    int i = 0;
    PK_DBG("[camera_moto_probe] start\n");

    if (platform_driver_register(&moto_platform_driver)) {
        PK_PR_ERR("failed to register camera moto driver\n");
        return -ENODEV;
    }
    for(i=0;i < sizeof(vib_val) / sizeof(vib_val[0]);i++)
    {
      if(driver_create_file(&moto_platform_driver.driver, vib_val[i])){
        PK_PR_ERR("create bv_val failed!!\n");
      }
    }
    init_waitqueue_head(&up_waitqueue);
    init_waitqueue_head(&down_waitqueue);
    //init_waitqueue_head(&press_wait);

    memset((void *)&subcam_up_wq, 0, sizeof(subcam_up_wq));
    INIT_DELAYED_WORK(&subcam_up_wq, subcam_up_wq_routine);
    memset((void *)&subcam_down_wq, 0, sizeof(subcam_down_wq));
    INIT_DELAYED_WORK(&subcam_down_wq, subcam_down_wq_routine);
    memset((void *)&subcam_press_wq, 0, sizeof(subcam_press_wq));
    INIT_DELAYED_WORK(&subcam_press_wq, subcam_press_wq_routine);
    return 0;
}
static void __exit moto_exit(void)
{
    platform_driver_unregister(&moto_platform_driver);
}
module_init(moto_init);
module_exit(moto_exit);

MODULE_DESCRIPTION("sensor moto  driver");
MODULE_AUTHOR("Foeec");
MODULE_LICENSE("GPL v2");

