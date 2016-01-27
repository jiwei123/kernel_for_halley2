/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Author: Li Zidong <lizidong.marco@gmail.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <soc/gpio.h>
#include <asm/atomic.h>
#include <linux/irq.h>
#include <linux/input-polldev.h>
#include <linux/timer.h>
#include <linux/hwmon-sysfs.h>
#include <linux/input/em30719.h>
#include <linux/i2c/i2c_power_manager.h>

#define EM30719_NAME    "em30719"
#define MAG_MIN_POS     0
#define MAG_MAX_POS     256

#define LOW_THRESHOLD   0xA0
#define HIGH_THRESHOLD  0xA0

typedef struct {
    struct i2c_client *client;
    struct em30719_platform_data *pdata;
    struct class *em30719_class;
    struct device *em30719_device;
    struct input_dev *input_dev;
    struct kobject *kobj;
    struct delayed_work em30719_work;
    unsigned int major_id;
    int irq;
    atomic_t enabled;
    atomic_t period_ms;
    struct wake_lock irq_lock;
} em30719_data_t;
static em30719_data_t em30719data;

static struct i2c_power_device *device = NULL;
static struct i2c_control_operations func;
static int last_val = 0;

static int em30719_i2c_write(unsigned char reg, unsigned char * data, int len) {
    unsigned char buf[20];
    int rc;
    int ret = 0;
    int i;

    buf[0] = reg;
    if (len >= 20) {
        printk("%s (%d) : FAILED: buffer size is limitted(20) %d\n", __func__,
                __LINE__, len);
        return -1;
    }

    for (i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }

    rc = i2c_master_send(em30719data.client, buf, len + 1);

    if (rc != len + 1) {
        printk("%s (%d) : FAILED: writing to reg 0x%x\n", __func__, __LINE__,
                reg);
        ret = -1;
    }

    return ret;
}

static int em30719_i2c_read(unsigned char reg, unsigned char * data) {
    unsigned char buf[20];
    int rc;

    buf[0] = reg;

    rc = i2c_master_send(em30719data.client, buf, 1);
    if (rc != 1) {
        printk("%s (%d) : FAILED: writing to address 0x%x\n", __func__,
                __LINE__, reg);
        return -1;
    }

    rc = i2c_master_recv(em30719data.client, buf, 2);
    if (rc != 2) {
        printk("%s (%d) : FAILED: reading data\n", __func__, __LINE__);
        return -1;
    }

    *data = buf[0];
    return 0;
}

static int em30719_i2c_burst_read(unsigned char reg, unsigned char * data, int len)
{
    unsigned char buf[20];
    int rc;

    buf[0] = reg;

    rc = i2c_master_send(em30719data.client, buf, 1);   // Returns negative errno, or else the number of bytes written.
    if (rc != 1) {
        printk("%s (%d) : FAILED: writing to address 0x%x\n",
               __func__, __LINE__, reg);
        return -1;
    }

    rc = i2c_master_recv(em30719data.client, data, len);
    if (rc != len) {
        printk("%s (%d) : FAILED: reading data %d\n", __func__,
               __LINE__, rc);
        return -1;
    }

    return 0;
}

unsigned char em30719_write_reg(unsigned char addr, unsigned char data) {
    int ret = em30719_i2c_write(addr, &data, 1);

    if (ret != 0)
        return false;
    else
        return true;
}

unsigned char em30719_read_reg(unsigned char addr, unsigned char *data) {
    int ret = 0;
    unsigned char tmp_data[2] = {0};

    ret = em30719_i2c_burst_read(addr, tmp_data, 2);
    *data = tmp_data[0];

    if (ret != 0)
        return false;
    else
        return true;
}

void em30719_enable_ps(int enable) //enable operation
{
    unsigned char reg_value = 0;

    em30719_read_reg(ALP_PS_CONFIG_REG, &reg_value);
    if (enable) {
        reg_value = reg_value | 0x80;
        em30719_write_reg(ALP_PS_CONFIG_REG, reg_value);
    } else {
        reg_value = reg_value & (~0x80);
        em30719_write_reg(ALP_PS_CONFIG_REG, reg_value);
    }
}

void em30719ee_init(void) {
    unsigned char pid = 0;

    em30719_write_reg(ALP_PS_CONFIG_REG, 0);        //Disable and Power down
    em30719_read_reg(ALP_PS_PID_REG, &pid);
    printk("EM30719 PID=0x%02x\n", pid);
    if (pid != EM30719_PID)
        return;

    em30719_write_reg(ALP_PS_INTERRUPT_REG, 0);     //Clear all interrupt flag
    em30719_write_reg(ALP_PS_RESET_REG, 0);         //Initialize Reset register

    em30719_write_reg(ALP_PS_PROX_LT_REG, LOW_THRESHOLD);   //PS Low threshold value
    em30719_write_reg(ALP_PS_PROX_HT_REG, HIGH_THRESHOLD);  //PS High threshold value
    em30719_write_reg(ALP_PS_OFFSET_REG, 0x0);      //Initialize PS offset

    //Disable ALS Interrupt
    em30719_write_reg(ALP_PS_ALSIR_TH1_REG, 0x0);
    em30719_write_reg(ALP_PS_ALSIR_TH2_REG, 0xf0);
    em30719_write_reg(ALP_PS_ALSIR_TH3_REG, 0xff);

    // set LED drive current to 200mA, default LED drive current is 15mA
//    em30719_write_reg(ALP_PS_CONFIG_REG, 0x38);
    em30719_enable_ps(0);
}

static int em30719_open(struct inode *inode, struct file *filp) {
    dev_info(&em30719data.client->dev, "%s\n", __FUNCTION__);
    return 0;
}

static int em30719_release(struct inode *inode, struct file *filp) {
    dev_info(&em30719data.client->dev, "%s\n", __FUNCTION__);
    return 0;
}

static struct file_operations em30719_fops = {
        .owner = THIS_MODULE,
        .open = em30719_open,
        .release = em30719_release,
};

static irqreturn_t em30719_irq_func(int irq, void *data) {
    disable_irq_nosync(em30719data.irq);
    wake_lock(&em30719data.irq_lock);
    schedule_delayed_work(&em30719data.em30719_work, msecs_to_jiffies(atomic_read(&em30719data.period_ms)));

    return IRQ_HANDLED;
}

void em30719_work_func(struct work_struct *work) {
    int val = 0;
    unsigned char ps_high_thd = 0;
    u8 int_status = 0;
    u8 read_back = 0;
    u8 approach = 0;


    em30719_read_reg(ALP_PS_PROX_HT_REG, &ps_high_thd);
    em30719_read_reg(ALP_PS_PROX_DATA_REG, &val);

    if (val == last_val) {
        if (val < ps_high_thd) {
            val++;
        } else {
            val--;
        }
    }

    if (val == 0)
        val = 1;

    last_val = val;

    if (val < ps_high_thd) {
        if (val >= 39) {
            val = 39;
        }
        val = -val;
    }

    printk(KERN_DEBUG "====em30719 report val is %d=====\n", val);

    input_report_abs(em30719data.input_dev, ABS_GAS, val);
    input_sync(em30719data.input_dev);

    enable_irq(em30719data.irq);
    do {
        em30719_read_reg(ALP_PS_INTERRUPT_REG, &int_status);
        em30719_write_reg(ALP_PS_INTERRUPT_REG, int_status & 0x7F);
        em30719_read_reg(ALP_PS_INTERRUPT_REG, &read_back);
    } while (read_back & 0x80);

    wake_unlock(&em30719data.irq_lock);
}

static ssize_t em30719_show_enable(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", atomic_read(&em30719data.enabled));

    return ret;
}

static ssize_t em30719_store_enable(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    unsigned long enable;
    int val = 0;
    unsigned char ps_high_thd = 0;

    if (strict_strtoul(buf, 10, &enable))
        return -EINVAL;

    if (atomic_read(&em30719data.enabled) == !!enable) {
        return size;
    }

    atomic_set(&em30719data.enabled, !!enable);

    if (!!enable) {
        enable_irq(em30719data.irq);
        em30719_enable_ps(1);

        em30719_read_reg(ALP_PS_PROX_HT_REG, &ps_high_thd);
        em30719_read_reg(ALP_PS_PROX_DATA_REG, &val);

        if (val == last_val) {
            if (val < ps_high_thd) {
                val++;
            } else {
                val--;
            }
        }

        if (val == 0)
            val = 1;

        last_val = val;

        if (val < ps_high_thd) {
            val = -val;
        }

        printk(KERN_DEBUG "====em30719 report first val is %d=====\n", val);
        input_report_abs(em30719data.input_dev, ABS_GAS, val);
        input_sync(em30719data.input_dev);
    } else {
        em30719_enable_ps(0);
        disable_irq(em30719data.irq);
    }

    return size;
}

static ssize_t em30719_show_polling_rate(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", atomic_read(&em30719data.period_ms));

    return ret;
}

static ssize_t em30719_store_polling_rate(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    unsigned long interval_ms;

    if (strict_strtoul(buf, 10, &interval_ms))
        return -EINVAL;

    if (interval_ms < 10)
        interval_ms = 10;

    if (interval_ms > 500)
        interval_ms = 500;

    atomic_set(&em30719data.period_ms, interval_ms);

    return size;
}

static SENSOR_DEVICE_ATTR(enable_proximity, S_IRUGO | S_IWUGO,
        em30719_show_enable, em30719_store_enable, 0);

static SENSOR_DEVICE_ATTR(poll_period_ms_proximity, S_IRUGO | S_IWUGO,
        em30719_show_polling_rate, em30719_store_polling_rate, 0);

static struct attribute *em30719_attributes[] = {
        &sensor_dev_attr_enable_proximity.dev_attr.attr,
        &sensor_dev_attr_poll_period_ms_proximity.dev_attr.attr,
        NULL
};

static const struct attribute_group em30719_attribute_group = {
        .attrs = em30719_attributes,
};

static void em30719_power_on(struct em30719_platform_data *pdata)
{
    if (pdata->power_on)
        pdata->power_on();
}

static void em30719_power_off(struct em30719_platform_data *pdata)
{
    if (pdata->power_off)
        pdata->power_off();
}

static void em30719_init_chip(void)
{
    jzgpio_ctrl_pull(em30719data.pdata->gpio_int / 32, 1, em30719data.pdata->gpio_int % 32);
    em30719ee_init();
}

static int em30719_i2c_probe(struct i2c_client *client,
        const struct i2c_device_id *id) {
    struct em30719_platform_data *pdata = NULL;
    struct input_dev *input_dev = NULL;
    int err = 0;
    int i = 0;
    u8 pid = 0;

    pdata = (struct em30719_platform_data *) client->dev.platform_data;
    em30719data.client = client;
    em30719data.pdata = pdata;

    if (pdata->board_init) {
        err = pdata->board_init(&client->dev);
        if (err < 0) {
            pr_err("board_init failed ! errno = %d\n", err);
            return err;
        }
    }

    func.power_on = em30719_power_on;
    func.power_off = em30719_power_off;
    func.init_chip = em30719_init_chip;

    device = register_i2c_power_device(i2c_adapter_id(client->adapter), &func, pdata);
    if (!device) {
        err = -1;
        goto board_exit;
    }

    i2c_power_device_on(device);
    msleep(2);

    err = em30719_read_reg(ALP_PS_PID_REG, &pid);
    if (err == false) {
        printk("em30719 error: Can not read PID\n");
        err = -1;
        goto failed_to_read_reg;
    }

    err = gpio_request(pdata->gpio_int, "em30719_gpio_int");
    if (err < 0) {
        printk("em30719: Unable to request INT GPIO %d\n", pdata->gpio_int);
        goto failed_request_gpio_int;
    }
    gpio_direction_input(pdata->gpio_int);
    jzgpio_ctrl_pull(pdata->gpio_int / 32, 1, pdata->gpio_int % 32);

    em30719data.irq = gpio_to_irq(pdata->gpio_int);
    if (em30719data.irq > 0) {
        err = request_any_context_irq(em30719data.irq, em30719_irq_func,
                IRQF_TRIGGER_LOW, "em30719_irq",
                &em30719data);

        if (err) {
            printk(KERN_ERR "Failed to request irq: %d\n", err);
            goto failed_gpio_to_irq;
        } else {
            enable_irq_wake(em30719data.irq);
            disable_irq(em30719data.irq);
        }
    }

    err = register_chrdev(0, EM30719_NAME, &em30719_fops);
    if (err < 0) {
        printk(KERN_WARNING "em30719 : Can't get major\n");
        goto failed_gpio_to_irq;
    } else {
        em30719data.major_id = err;
        printk("em30719 : Success to register character device %d\n", em30719data.major_id);
        em30719data.em30719_class = class_create(THIS_MODULE, EM30719_NAME);

        if (IS_ERR(em30719data.em30719_class)) {
            printk(KERN_ERR "class_create() failed for ofn_class\n");
            err = -1;
            goto free_chrdev;
        }

        em30719data.em30719_device = device_create(em30719data.em30719_class,
                NULL,MKDEV(em30719data.major_id, 0), NULL,
                EM30719_NAME);
    }

    em30719data.kobj = kobject_create_and_add("em30719", kernel_kobj);
    if (!em30719data.kobj) {
        goto failed_kobj_create;
    }

    err = sysfs_create_group(em30719data.kobj, &em30719_attribute_group);
    if (err) {
        goto failed_sysfs_create_group;
    }

    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto failed_input_dev_alloc;
    }

    em30719data.input_dev = input_dev;
    set_bit(EV_ABS, em30719data.input_dev->evbit);
    input_set_abs_params(em30719data.input_dev, ABS_GAS,
                         -MAG_MIN_POS, MAG_MAX_POS, 0, 0);
    em30719data.input_dev->name = EM30719_NAME;

    err = input_register_device(em30719data.input_dev);
    if (err) {
        printk(KERN_ERR "em30719_probe: failed to register input devices\n");
        goto failed_input_register_device;
    }

    atomic_set(&em30719data.enabled, 0);
    atomic_set(&em30719data.period_ms, 80);

    INIT_DELAYED_WORK(&em30719data.em30719_work, em30719_work_func);
    wake_lock_init(&em30719data.irq_lock, WAKE_LOCK_SUSPEND, "em30719_irq");

    printk("em30719_i2c_probe success\n");

    return 0;

failed_input_register_device:
    input_free_device(em30719data.input_dev);
failed_input_dev_alloc:
    sysfs_remove_group(em30719data.kobj, &em30719_attribute_group);
failed_sysfs_create_group:
    kobject_put(em30719data.kobj);
failed_kobj_create:
    device_destroy(em30719data.em30719_class, MKDEV(em30719data.major_id, 0));
    class_destroy(em30719data.em30719_class);
free_chrdev:
    unregister_chrdev(em30719data.major_id, EM30719_NAME);
failed_gpio_to_irq:
    gpio_free(pdata->gpio_int);
failed_request_gpio_int:
failed_to_read_reg:
    i2c_power_device_off(device);
    unregister_i2c_power_device(device);
board_exit:
    pdata->board_exit(&client->dev);

    return err;
}

static int em30719_i2c_remove(struct i2c_client *client) {
    return 0;
}

static int em30719_suspend(struct device *dev)
{
    return 0;
}

static int em30719_resume(struct device *dev)
{
    return 0;
}

static const struct i2c_device_id em30719_device_id[] =
        { { "em30719", 0 }, { } };

static const struct dev_pm_ops em30719_pm_ops = {
        .suspend = em30719_suspend,
        .resume = em30719_resume,
};

static struct i2c_driver em30719_i2c_driver = {
        .driver = {
                .name = "em30719",
                .owner = THIS_MODULE,
                .pm = &em30719_pm_ops,
        },
        .probe = em30719_i2c_probe,
        .remove = em30719_i2c_remove,
        .id_table = em30719_device_id,
};

static int __init em30719_init(void)
{
    int retval;

    retval = i2c_add_driver(&em30719_i2c_driver);
    if (retval) {
        printk("*****em30719 add i2c driver failed *****\n");
    }

    return retval;
}

static void __exit em30719_exit(void)
{
    device_destroy(em30719data.em30719_class, MKDEV(em30719data.major_id, 0)); //delete device node under /dev
    class_destroy(em30719data.em30719_class);//delete class created by us
    unregister_chrdev(em30719data.major_id, EM30719_NAME);
    i2c_del_driver(&em30719_i2c_driver);
}

module_init( em30719_init);
module_exit( em30719_exit);
MODULE_DESCRIPTION("em30719 driver");
MODULE_LICENSE("GPL");
