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
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

//用户自定义头文件
#define TRACE_MODULE    "drv_i2c_ap3216c.c"

#include <user_util.h>
#include <trace.h>
#include <drv_i2c_ap3216c.h>

#define DEV_AP3216C_CNT     1
#define DEV_AP3216C_NAME    "dev_ap3216c"

#define WRITE_FLAG   0
#define READ_FLAG    1

typedef struct
{
    dev_t       dev_id ;
    cdev_t      cdev   ;
    class_t     *p_dev_class  ;
    device_t    *p_dev_device ;
    nd_t        *node ;
    int         major ;
    void        *private_data;
    uint16_t    ir;     //传感数据
    uint16_t    als;    //传感数据
    uint16_t    ps;     //传感数据
} ap3216c_dev_t;

static ap3216c_dev_t m_ap3216c_dev;

/**
 * @brief 从3216c读取多个寄存器数据
 * 
 */
static int ap3216c_read_regs(ap3216c_dev_t *p_3216c_dev,uint8_t reg,void *val,int len)
{
    int err_code;

    i2c_msg_t msg[2];
    i2c_client_t  *p_client = (i2c_client_t *)p_3216c_dev->private_data;

 /**
 * 按照AP3216C的协议来，一个msg信息体，在i2c上代表一个 start 信号
 */
    /**
     * 先写设备要读的寄存器地址
     */
    msg[0].addr = p_client->addr;
    msg[0].flags = WRITE_FLAG;
    msg[0].buf = &reg;
    msg[0].len = 1;

    /**
     * 读取数据
     */
    msg[1].addr = p_client->addr;
    msg[1].flags = READ_FLAG;
    msg[1].buf = val;
    msg[1].len = len;

    err_code = i2c_transfer(p_client->adapter,msg,2);

    if(err_code != 2)
    {
        trace_errorln("ap3216c_read_regs error!");
        return -EREMOTEIO;
    }

    return 0;
}

/**
 * @brief 往3216c寄存器写入数据
 * 
 */
static int ap3216c_write_regs(ap3216c_dev_t *p_3216c_dev,uint8_t reg,uint8_t *buf,uint8_t len)
{
    int err_code; 

    uint8_t temp_buff[255];

    i2c_msg_t msg;
    i2c_client_t  *p_client = (i2c_client_t *)p_3216c_dev->private_data;

    temp_buff[0] = reg;

    memcpy(&temp_buff[1],buf,len);

    /**
     * 读取数据
     */
    msg.addr = p_client->addr;
    msg.flags = WRITE_FLAG;
    msg.buf = temp_buff;
    msg.len = len + 1;

    err_code = i2c_transfer(p_client->adapter,&msg,1);

    return err_code;
}

/**
 *  读取寄存器的配置数据
 */
static uint8_t ap3216c_read_reg(ap3216c_dev_t *p_3216c_dev,uint8_t reg)
{
    uint8_t read_data;

    ap3216c_read_regs(p_3216c_dev,reg,&read_data,1);

    return read_data;
}

/**
 * 往寄存器写入配置数据
 */
static void ap3216c_write_reg(ap3216c_dev_t *p_3216c_dev, uint8_t reg,uint8_t data)
{
    uint8_t buf = data;

    ap3216c_write_regs(p_3216c_dev,reg,&buf,1);
}

/**
 * 读取ap3216c传感器数据
 */
static void ap3216c_sensor_data_get(ap3216c_dev_t *p_3216c_dev)
{
    uint8_t sensor_buf[6];

    uint8_t i;

    for(i = 0; i < 6; i++)
    {
        sensor_buf[i] = ap3216c_read_reg(p_3216c_dev,(AP3216C_IRDATALOW + i));
    }

    if(sensor_buf[0] & 0x80)
    {
        p_3216c_dev->ir = 0;
        trace_errorln("ir read error");
    }
    else
    {
        p_3216c_dev->ir = ((uint16_t)sensor_buf[1] << 2) | (sensor_buf[0] & 0x03);
    }
    
    p_3216c_dev->als = ((uint16_t)sensor_buf[3] << 8) | sensor_buf[2];

    if(sensor_buf[4] & 0x40)
    {
        p_3216c_dev->ps = 0;
        trace_errorln("ps read error");
    }
    else
    {
        p_3216c_dev->ps = ((uint16_t)(sensor_buf[5] & 0x3f) << 4) | (sensor_buf[4] & 0x0f);
    }
    
}

/**
 * 打开设备
 */
static int ap3216c_open( inode_t *p_inode , file_t *p_filp )
{
    p_filp->private_data = &m_ap3216c_dev;

    /**
     * 在打开设备的时候，才真正初始化设备
     */
    ap3216c_write_reg(&m_ap3216c_dev,AP3216C_SYSTEMCONG,0x04);
    mdelay(50);
    ap3216c_write_reg(&m_ap3216c_dev,AP3216C_SYSTEMCONG,0x03);

    return 0;
}

/**
 * 从设备中读取数据
 */
static ssize_t ap3216c_read( file_t *p_filp , char __user *buff  , size_t cnt , loff_t *offt  )
{
    uint16_t    data[3];

    int err_code = 0;

    ap3216c_dev_t *p_ap3216c_dev = (ap3216c_dev_t *)p_filp->private_data;

    ap3216c_sensor_data_get(p_ap3216c_dev);

    data[0] = p_ap3216c_dev->ir;
    data[1] = p_ap3216c_dev->als;
    data[2] = p_ap3216c_dev->ps;
#if 0
    trace_infoln("p_ap3216c_dev->ir =%d",p_ap3216c_dev->ir);
    trace_infoln("p_ap3216c_dev->als =%d",p_ap3216c_dev->als);
    trace_infoln("p_ap3216c_dev->ps =%d",p_ap3216c_dev->ps);
#endif
    err_code = copy_to_user(buff,data,sizeof(data));
    if(err_code != 0)
    {
        trace_errorln("copy to user error");
    }

    return sizeof(data);
}

//设备操作数据结构体
static file_operations_t dev_ap3216c_fops ={
    .owner = THIS_MODULE ,
    .open  = ap3216c_open,
    .read  = ap3216c_read,
};

/**
 * 
 */
static int ap3216c_probe(struct i2c_client *p_i2c_client, const struct i2c_device_id *p_i2c_device_id)
{
    alloc_chrdev_region( &m_ap3216c_dev.dev_id, 0, DEV_AP3216C_CNT, DEV_AP3216C_NAME);  //申请设备号
    m_ap3216c_dev.major = MAJOR(m_ap3216c_dev.dev_id);

    m_ap3216c_dev.cdev.owner = THIS_MODULE ;                   //初始化，添加一个cdev
    cdev_init(&m_ap3216c_dev.cdev , &dev_ap3216c_fops );
    cdev_add(&m_ap3216c_dev.cdev, m_ap3216c_dev.dev_id, DEV_AP3216C_CNT);

    m_ap3216c_dev.p_dev_class =  class_create(THIS_MODULE, DEV_AP3216C_NAME);
    if( IS_ERR(m_ap3216c_dev.p_dev_class) )
    {
        return PTR_ERR(m_ap3216c_dev.p_dev_class);
    }

    m_ap3216c_dev.p_dev_device = device_create( m_ap3216c_dev.p_dev_class,      // in ，创建哪一类
                                        NULL,                                   //一般为NULL
                                        m_ap3216c_dev.dev_id ,                  //设备号
                                        NULL,                                   //一般为NULL
                                        DEV_AP3216C_NAME);                      //挂载到/dev/目录下的设备节点名称                     
    if( IS_ERR(m_ap3216c_dev.p_dev_device) )
    {
        return PTR_ERR(m_ap3216c_dev.p_dev_device);
    }

    m_ap3216c_dev.private_data = p_i2c_client;

    trace_infoln("Success:i2c probe success");
    return 0;
}

/**
 * 
 */
static int ap3216c_remove(struct i2c_client *p_i2c_client)
{
    cdev_del( &m_ap3216c_dev.cdev );
    unregister_chrdev_region( m_ap3216c_dev.dev_id , DEV_AP3216C_CNT);

    device_destroy( m_ap3216c_dev.p_dev_class , m_ap3216c_dev.dev_id );
    class_destroy(m_ap3216c_dev.p_dev_class);

    trace_infoln("Success:i2c remove success");
    return 0;
}

/* 传统匹配方式 ID 列表 */
static const struct i2c_device_id m_ap3216c_id[] = {
    {"alientek,ap3216c", 0},
    {}
};

/* 设备树匹配列表 */
static const struct of_device_id m_ap3216c_of_match[] = {
    { .compatible = "alientek,ap3216c"},
    { /* Sentinel */ }
};

/* i2c 驱动结构体 */
static struct i2c_driver m_ap3216c_driver = {
    .probe = ap3216c_probe,
    .remove = ap3216c_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "ap3216c",
        .of_match_table = m_ap3216c_of_match,
    },
    .id_table = m_ap3216c_id,
};

/* 
 * 驱动入口函数 
 */
static int __init dev_ap3216c_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&m_ap3216c_driver);

    return ret;
}

/* 
 * 驱动出口函数 
 */
static void __exit dev_ap3216c_exit(void)
{
    i2c_del_driver(&m_ap3216c_driver);
}

module_init(dev_ap3216c_init);
module_exit(dev_ap3216c_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息

#undef WRITE_FLAG
#undef READ_FLAG
