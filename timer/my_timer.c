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

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>


//用户自定义头文件
#define TRACE_MODULE    "my_timer.c"

#include <user_util.h>
#include <trace.h>
#include <cmd_downstream_handler.h>

#define CDEV_CNT    1               //设备的个数
#define LED_NAME    "timer_led"

#define LED_BIT     3

#define RX_BUF_SIZE 1024

typedef struct 
{
    dev_t           dev_id ;  
    cdev_t          cdev   ;
    class_t         *p_dev_class  ;
    device_t        *p_dev_device ;
    int             major ;
    int             minor ;
    nd_t            *node ;
    int             gpio_index;
    timer_list_t    dev_timer;
} dts_chr_dev_t ;

static  dts_chr_dev_t  m_timer_dev;  //定义一个设备书书据结构体

static uint8_t  m_rx_buf[RX_BUF_SIZE] ;
static uint32_t m_timer_inter = 0 ;

/**
 * @brief 控制led开关
 */
static void led_onoff(bool sw)
{
    if( true == sw )
    {
        gpio_set_value( m_timer_dev.gpio_index , 0 ) ;
    }
    else
    {
        gpio_set_value( m_timer_dev.gpio_index , 1 ) ;
    }
}

/**
 * @brief 打开字符设备 
 */
static int chr_drv_open( inode_t *p_inode , file_t *p_filp  )
{
    p_filp->private_data = &m_timer_dev ;
    return 0 ;
}

/**
 * @brief 释放/关闭字符设备
 */
static int chr_drv_release( inode_t *p_inode , file_t *p_filp )
{
    return 0 ;
}

/**
 * @brief 读字符设备 
 */
//ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
static ssize_t chr_drv_read( file_t *p_filp , char __user *buff  , size_t cnt , loff_t *offt  )
{

    return 0 ;
}

/**
 * @brief 处理命令函数
 */
static void  cmd_handler(sub_cmd_t sub_cmd)
{   
    uint32_t   cmd_data = 0 ;
    cmd_data = uint32_decode(sub_cmd.p_data);

    m_timer_inter = cmd_data ; 
    switch ( sub_cmd.cmd )
    {
        case CMD_START:
            mod_timer( &m_timer_dev.dev_timer , jiffies + msecs_to_jiffies(cmd_data)  );
            trace_infoln("cmd: CMD_START with %d ms" , cmd_data );
        break;

        case CMD_STOP:
            del_timer_sync(&m_timer_dev.dev_timer);
            trace_infoln("cmd: CMD_STOP");
        break ;

        case CMD_MODIFY:
            mod_timer( &m_timer_dev.dev_timer , jiffies + msecs_to_jiffies(cmd_data)  );
            trace_infoln("cmd: CMD_MODIFY with %d ms" , cmd_data );

        break;

        default:
            trace_infoln("ERROR:  the sub_cmd cmd is not suit for timer");
        break;
    }
}


/**
 * @brief 写字符设备
 */
//ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t chr_drv_write( file_t *p_filp , const char __user *buff , size_t cnt , loff_t *offt )
{
    int err_code ;
    
    int cmd_total_number = 0 ;
    sub_cmd_t  sub_cmd ;
    uint32_t   cmd_data = 0 ;

    memset(&sub_cmd , 0 , sizeof(sub_cmd_t) );

    err_code = copy_from_user( m_rx_buf , buff , cnt );   //从用户空间拷贝数据到内核空间,驱动属于内核，即从应用层拷到内核
    if(err_code < 0 )
    {
        printk("app write data to kernel fail\r\n");
        return -EFAULT ;
    }

    cmd_total_number = cmd_total_num_get(m_rx_buf);
    err_code = index_sub_cmd_get( 1 , m_rx_buf , &sub_cmd);
    if( err_code != 0 )
    {
        printk("ERROR:  sub_get cmd fail\r\n");
        return -EINVAL;
    }

    cmd_data = uint32_decode(sub_cmd.p_data);

#if 0   
    uint16_t i = 0 ;

    printk("receive %d byte data , rx data:\r\n" , cnt );   //打印应用层传过来的数据
    for( i = 0 ; i < cnt ; i++ )
    {
        printk("%02x",m_rx_buf[i] );
    }
    printk("\r\n");

    printk("cmd_total_number = %d\r\n",cmd_total_number );
    printk("sub_cmd.cmd = %02x\r\n",sub_cmd.cmd );
    printk("sub_cmd.d_len = %02x\r\n",sub_cmd.d_len );
    printk("cmd_data = 0x%08x\r\n",cmd_data );
#endif

    cmd_handler( sub_cmd );
    return 0 ;
}

//字符设备操作数据结构体
static file_operations_t chr_cdev_fops ={
    .owner = THIS_MODULE ,
    .open  = chr_drv_open,
    .release = chr_drv_release,
    .read  = chr_drv_read ,
    .write = chr_drv_write,
};


/**
 * @brief 定时器回调函数
 */
static void timer_cb_handler(unsigned long arg)    // unsigned long 可以指向数据的地址
{
    static bool led_sw = false ;

    if( led_sw == false )
    {
        led_sw = true ;
    }
    else
    {
        led_sw = false ;
    }
    led_onoff(led_sw);

    mod_timer( &m_timer_dev.dev_timer , jiffies + msecs_to_jiffies(m_timer_inter)  );

    //printk("timer_cb_handler\r\n"); //屏蔽，不然终端一直有打印
}

/**
 * @brief 字符设备驱动入口函数
 */
static int __init chr_dev_init(void)
{

    int         err_code ;

    /*从设备树中获取数据*/
    /*1.获取设备节点 */
    m_timer_dev.node = of_find_node_by_path("/cxn_pinctrl"); //根节点下的设备节点
    if( NULL == m_timer_dev.node )
    {
        printk("Fail: cxn_pinctrl node can not find\n");
        return -EINVAL;
    }
    else
    {
        printk("Success: cxn_pinctrl has been found\n");
    }

    init_timer(&m_timer_dev.dev_timer);     //初始化定时器

    /*获取设备树上自定义属性的gpio编号*/
    m_timer_dev.gpio_index = of_get_named_gpio( m_timer_dev.node , "led_gpio", 0);
    if( m_timer_dev.gpio_index < 0 )
    {
        printk("Fail: cxn_pinctrl node can not find\n");
        return -EINVAL;
    }
    printk("Success: m_timer_dev.gpio_index = %d \n" , m_timer_dev.gpio_index );

    /*设置IO*/
    err_code = gpio_direction_output(m_timer_dev.gpio_index, 1);
    if( err_code < 0 )
    {
        printk("Fail:the gpio set is fail \n");
    }


    alloc_chrdev_region( &m_timer_dev.dev_id , 0 , CDEV_CNT , LED_NAME );  //申请设备号
    m_timer_dev.major = MAJOR(m_timer_dev.dev_id);    //通过设备号获取主，次设备号
    m_timer_dev.minor = MINOR(m_timer_dev.dev_id);
    printk("major = %d , minor = %d\n", m_timer_dev.major , m_timer_dev.minor );

    m_timer_dev.cdev.owner = THIS_MODULE ;                   //初始化，添加一个cdev
    cdev_init(&m_timer_dev.cdev , &chr_cdev_fops );
    cdev_add(&m_timer_dev.cdev, m_timer_dev.dev_id , CDEV_CNT );

    m_timer_dev.p_dev_class =  class_create (THIS_MODULE, LED_NAME );
    if( IS_ERR(m_timer_dev.p_dev_class) )
    {
        return PTR_ERR(m_timer_dev.p_dev_class);
    }

    m_timer_dev.p_dev_device = device_create( m_timer_dev.p_dev_class,      // in ，创建哪一类
                                        NULL,                                   //一般为NULL
                                        m_timer_dev.dev_id ,                  //设备号
                                        NULL,                                   //一般为NULL
                                        "timer_dev" );                   //挂载到/dev/目录下的设备节点名称                     
    if( IS_ERR(m_timer_dev.p_dev_device) )
    {
        return PTR_ERR(m_timer_dev.p_dev_device);
    }

//    printk("dts: dts pinctrl chr led register\n");

    init_timer(&m_timer_dev.dev_timer); //先申请空间，在赋值
    m_timer_dev.dev_timer.function = timer_cb_handler ;


    return 0 ;

}

/**
 * @brief 字符设备驱动出口函数
 */
static void __exit chr_dev_exit(void)
{
    led_onoff(false);

    gpio_free(m_timer_dev.gpio_index);
    cdev_del( &m_timer_dev.cdev );
    unregister_chrdev_region( m_timer_dev.dev_id , CDEV_CNT );

    device_destroy( m_timer_dev.p_dev_class , m_timer_dev.dev_id );
    class_destroy(m_timer_dev.p_dev_class);

    del_timer_sync(&m_timer_dev.dev_timer);

    printk("Success: dts pinctrl chr led exit\n");
}


module_init(chr_dev_init);
module_exit(chr_dev_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息

