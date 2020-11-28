//路径是内核源代码  include/linux/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/platform_device.h>

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

//用户自定义头文件
#define TRACE_MODULE    "drv_spi_icm20608.c"

#include <user_util.h>
#include <trace.h>
#include <drv_spi_icm20608.h>

#define DEV_ICM20608_CNT     1
#define DEV_ICM20608_NAME    "dev_icm20608"

typedef struct 
{
    dev_t       dev_id ;
    cdev_t      cdev   ;
    class_t     *p_dev_class  ;
    device_t    *p_dev_device ;
    nd_t        *node ;
    int         major;           /* 主设备号 */
    void        *private_data;   /* 私有数据 */
    int         cs_gpio;         /* 片选所使用的 GPIO 编号*/
    signed int gyro_x_adc;      /* 陀螺仪 X 轴原始值 */
    signed int gyro_y_adc;      /* 陀螺仪 Y 轴原始值 */
    signed int gyro_z_adc;      /* 陀螺仪 Z 轴原始值 */
    signed int accel_x_adc;     /* 加速度计 X 轴原始值 */
    signed int accel_y_adc;     /* 加速度计 Y 轴原始值 */
    signed int accel_z_adc;     /* 加速度计 Z 轴原始值 */
    signed int temp_adc;        /* 温度原始值 */
} icm20608_dev_t;

static icm20608_dev_t m_icm20608_dev;

/**
 * @brief 在icm20608寄存器里读取多个字节的数据
 */
static int icm20608_read_regs(icm20608_dev_t *p_icm20608_dev,uint8_t reg,void *buf,int len)
{
    int err_code;

    uint8_t tx_buff[len];

    spi_message_t   message_queue;
    spi_transfer_t  *p_spi_trans;
    spi_device_t    *p_spi_dev = (spi_device_t *)p_icm20608_dev->private_data;

    trace_infoln("read cs-gpio = %d",p_icm20608_dev->cs_gpio);

    gpio_set_value(p_icm20608_dev->cs_gpio,0);

    p_spi_trans = kzalloc(sizeof(spi_transfer_t),GFP_KERNEL); 

    /**
     * 第1次,发送要读取的寄存器地址
     */
    tx_buff[0] = reg | 0x80;
    p_spi_trans->tx_buf = tx_buff;
    p_spi_trans->len = 1;

    spi_message_init(&message_queue);
    spi_message_add_tail(p_spi_trans,&message_queue);

    err_code = spi_sync(p_spi_dev,&message_queue);
    if(err_code < 0)
    {
        trace_infoln("spi_sync fail");
    }


    /**
     * 第2次，读取数据
     */
    tx_buff[0] = 0xff;
    p_spi_trans->rx_buf = buf;          //这里是RX_BUFF,曾马虎踩坑了，害
    p_spi_trans->len = len;

    spi_message_init(&message_queue);
    spi_message_add_tail(p_spi_trans,&message_queue);

    err_code = spi_sync(p_spi_dev,&message_queue);
    if(err_code < 0)
    {
        trace_infoln("spi_sync fail");
    }


    kfree(p_spi_trans);

    gpio_set_value(p_icm20608_dev->cs_gpio,1);

    return err_code;
}

/**
 * @brief 向icm20608寄存器写入数据
 */
static int icm20608_write_regs(icm20608_dev_t *p_icm20608_dev,uint8_t reg,uint8_t *buf,uint8_t len)
{
    int err_code;

    uint8_t tx_buff[len];

    spi_message_t   message_queue;
    spi_transfer_t  *p_spi_trans;
    spi_device_t    *p_spi_dev = (spi_device_t *)p_icm20608_dev->private_data;

    p_spi_trans = kzalloc(sizeof(spi_transfer_t),GFP_KERNEL);
    trace_infoln("sizeof(spi_transfer_t) = %d",sizeof(spi_transfer_t));

    gpio_set_value(p_icm20608_dev->cs_gpio,0);

    /**
     * 第1次,发送要读取的寄存器地址
     */
    tx_buff[0] = reg & ~0x80;
    p_spi_trans->tx_buf = tx_buff;
    p_spi_trans->len = 1;

    spi_message_init(&message_queue);
    spi_message_add_tail(p_spi_trans,&message_queue);

    err_code = spi_sync(p_spi_dev,&message_queue);
    if(err_code < 0)
    {
        trace_infoln("spi_sync fail");
    }

    /**
     * 第2次，读取数据
     */
    //tx_buff[0] = 0xff;
    p_spi_trans->len = len;
    p_spi_trans->tx_buf = buf;

    spi_message_init(&message_queue);
    spi_message_add_tail(p_spi_trans,&message_queue);

    err_code = spi_sync(p_spi_dev,&message_queue);
    if(err_code < 0)
    {
        trace_infoln("spi_sync fail");
    }


    kfree(p_spi_trans);

    gpio_set_value(p_icm20608_dev->cs_gpio,1);

    return err_code;
}

/**
 * 读取 icm20608 指定寄存器，读取一个寄存器
 */
static uint8_t icm20608_read_onereg(icm20608_dev_t *p_icm20608_dev,uint8_t reg)
{
    uint8_t data = 0xFF;

    icm20608_read_regs(p_icm20608_dev,reg,&data,1);

    return data;
}

/**
 * 向icm20608 往指定的一个寄存器写入指定的值
 */
static void icm20608_write_onereg(icm20608_dev_t *p_icm20608_dev,uint8_t reg,uint8_t value)
{
    uint8_t buf = value;

    icm20608_write_regs(p_icm20608_dev,reg,&buf,1);
}

/**
 * 读取传感器
 */
static void icm20608_sensor_data_get(icm20608_dev_t *p_icm20608_dev)
{
    uint8_t read_data[14];

    icm20608_read_regs(p_icm20608_dev,ICM20_ACCEL_XOUT_H,read_data,sizeof(read_data));

    p_icm20608_dev->accel_x_adc = (signed short)((read_data[0] << 8) | read_data[1]);
    p_icm20608_dev->accel_y_adc = (signed short)((read_data[2] << 8) | read_data[3]);
    p_icm20608_dev->accel_z_adc = (signed short)((read_data[4] << 8) | read_data[5]);
    p_icm20608_dev->temp_adc    = (signed short)((read_data[6] << 8) | read_data[7]);
    p_icm20608_dev->gyro_x_adc =  (signed short)((read_data[8] << 8) | read_data[9]);
    p_icm20608_dev->gyro_y_adc =  (signed short)((read_data[10] << 8) | read_data[11]);
    p_icm20608_dev->gyro_z_adc =  (signed short)((read_data[12] << 8) | read_data[13]);
}

/**
 * icm20608 内部寄存器初始化函数
 */
static void icm20608_reg_init(void)
{
    uint8_t value = 0;

    icm20608_write_onereg(&m_icm20608_dev,ICM20_PWR_MGMT_1,0x80);
    mdelay(50);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_PWR_MGMT_1,0x01);
    mdelay(50);

    value = icm20608_read_onereg(&m_icm20608_dev,ICM20_WHO_AM_I);
    trace_infoln("ICM20608 ID = %#X",value);

    icm20608_write_onereg(&m_icm20608_dev,ICM20_SMPLRT_DIV,0x00);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_GYRO_CONFIG,0x18);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_ACCEL_CONFIG,0x18);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_CONFIG,0x04);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_ACCEL_CONFIG2,0x04);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_PWR_MGMT_2,0x00);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_LP_MODE_CFG,0x00);
    icm20608_write_onereg(&m_icm20608_dev,ICM20_FIFO_EN,0x00);
}

/**
 * 字符设备驱动框架
 */
static int icm20608_open(struct inode *p_inode,struct file *p_filp)
{
    p_filp->private_data = &m_icm20608_dev; //获取该层的设备结构体

    return 0;
}

/**
 * 从设备中读取数据
 */
static ssize_t icm20608_read(file_t *p_filp,char __user *buff,size_t cnt,loff_t *offt)
{
    signed int sensor_data[7];

    icm20608_dev_t *p_dev = (icm20608_dev_t *)p_filp->private_data;

    int err_code = 0;

    icm20608_sensor_data_get(p_dev);

    sensor_data[0] = p_dev->accel_x_adc;
    sensor_data[1] = p_dev->accel_y_adc;
    sensor_data[2] = p_dev->accel_z_adc;
    sensor_data[3] = p_dev->temp_adc;
    sensor_data[4] = p_dev->gyro_x_adc;
    sensor_data[5] = p_dev->gyro_y_adc;
    sensor_data[6] = p_dev->gyro_z_adc;

    err_code = copy_to_user(buff,sensor_data,sizeof(sensor_data));
    if(err_code != 0)
    {
        trace_errorln("copy to user error");
    }

    return sizeof(sensor_data);
}

/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int icm20608_release(struct inode *inode, struct file *filp)
{
	return 0;
}

//设备操作数据结构体
static file_operations_t dev_icm20608_fops = {
    .owner = THIS_MODULE ,
    .open  = icm20608_open,
    .read  = icm20608_read,
    .release = icm20608_release,
};

/**
 * @brief 注册
 * 
 */
static int icm20608_probe(struct spi_device *p_spi_dev)
{
    int ret = 0;

    alloc_chrdev_region( &m_icm20608_dev.dev_id, 0, DEV_ICM20608_CNT, DEV_ICM20608_NAME);  //申请设备号
    m_icm20608_dev.major = MAJOR(m_icm20608_dev.dev_id);

    //m_icm20608_dev.cdev.owner = THIS_MODULE ;
    cdev_init(&m_icm20608_dev.cdev,&dev_icm20608_fops);
    cdev_add(&m_icm20608_dev.cdev,m_icm20608_dev.dev_id,DEV_ICM20608_CNT);

    m_icm20608_dev.p_dev_class = class_create(THIS_MODULE, DEV_ICM20608_NAME);
    if( IS_ERR(m_icm20608_dev.p_dev_class) )
    {
        return PTR_ERR(m_icm20608_dev.p_dev_class);
    }

    m_icm20608_dev.p_dev_device = device_create(m_icm20608_dev.p_dev_class,  // in ，创建哪一类
                                        NULL,                                //一般为NULL
                                        m_icm20608_dev.dev_id ,              //设备号
                                        NULL,                                //一般为NULL
                                        DEV_ICM20608_NAME);     
    if( IS_ERR(m_icm20608_dev.p_dev_device) )
    {
        return PTR_ERR(m_icm20608_dev.p_dev_device);
    }

    //路径是imx6ull.dtsi 中可见
    m_icm20608_dev.node = of_find_node_by_path("/soc/aips-bus@02000000/spba-bus@02000000/ecspi@02010000");
    if(m_icm20608_dev.node == NULL)
    {
        trace_errorln("espspi3 node have not found");
        return -EINVAL;
    }

    m_icm20608_dev.cs_gpio = of_get_named_gpio(m_icm20608_dev.node,"cs-gpio",0);
    if(m_icm20608_dev.cs_gpio < 0)
    {
        trace_errorln("can‘t get cs-gpio");
        return -EINVAL;
    }

    /* 3、设置 GPIO1_IO20 为输出，并且输出高电平 */
    ret = gpio_direction_output(m_icm20608_dev.cs_gpio, 1);
    if(ret < 0)
    {
        trace_errorln("can't set gpio!\r\n");
    }
    trace_infoln("cs-gpio = %d",m_icm20608_dev.cs_gpio);

    /*初始化 spi_device */
    p_spi_dev->mode = SPI_MODE_0; /*MODE0， CPOL=0， CPHA=0 */
    spi_setup(p_spi_dev);
    m_icm20608_dev.private_data = p_spi_dev; /* 设置私有数据 */

    icm20608_reg_init();

    trace_infoln("Success:spi_icm20608 probe success");

    return 0;
}

/**
 * @brief 释放
 */
static int icm20608_remove(spi_device_t *p_spi_dev)
{
    cdev_del( &m_icm20608_dev.cdev );
    unregister_chrdev_region( m_icm20608_dev.dev_id , DEV_ICM20608_CNT);

    device_destroy( m_icm20608_dev.p_dev_class , m_icm20608_dev.dev_id);
    class_destroy(m_icm20608_dev.p_dev_class);

    trace_infoln("Success:spi_icm20608 remove success");

    return 0;
}


/* 传统匹配方式 ID 列表 */
static const struct spi_device_id m_icm20608_id[] = {
    {"alientek,icm20608", 0},
    {}
};

/* 设备树匹配列表 */
static const struct of_device_id m_icm20608_of_match[] = {
    { .compatible = "alientek,icm20608" },
    { /* Sentinel */ }
};

/* SPI 驱动结构体 */
static struct spi_driver m_icm20608_driver = {
                                        .probe = icm20608_probe,
                                        .remove = icm20608_remove,
                                        .driver = {
                                            .owner = THIS_MODULE,
                                            .name = "icm20608",
                                            .of_match_table = m_icm20608_of_match,
                                        },
                                        .id_table = m_icm20608_id,
                                    };

/* 驱动入口函数 */
static int __init dev_icm20608_init(void)
{
    int err_code = 0;
    
    err_code = spi_register_driver(&m_icm20608_driver);

    trace_infoln("err_code = %d",err_code);

    return err_code;
}

/* 驱动出口函数 */
static void __exit dev_icm20608_exit(void)
{
    spi_unregister_driver(&m_icm20608_driver);
}


module_init(dev_icm20608_init);
module_exit(dev_icm20608_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息
