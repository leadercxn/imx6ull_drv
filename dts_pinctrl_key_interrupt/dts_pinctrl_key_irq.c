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
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>


//用户自定义头文件
#define TRACE_MODULE    "key_irq.c"

#include <user_util.h>
#include <trace.h>
#include <event_upstream_handler.h>

#define CDEV_CNT    1               //设备的个数
#define KEY_NAME    "dev_key_irq"

#define KEY_CNT     1               //按键数量

#define KEY_DELAY   20              //消抖

enum
{
    BUTTON_RELEASE ,
    BUTTON_PUSH ,
};

/**
 *@brief 中断IO描述结构体 
 */ 
typedef struct  {
    int              gpio ;                  //gpio
    unsigned int     irq_vector ;            //中断号
    uint8_t          key_value ;             //键值
    char             name[20];               //名字
    irqreturn_t      (*irq_handler_t) (int, void *);  //中断服务回调函数
    timer_list_t     irq_timer;              //每个按键中断对应一个定时器，用来消抖
} io_irq_des_t;

/**
 *@brief 设备结构体 
 */ 
typedef struct 
{
    dev_t           dev_id ;  
    cdev_t          cdev   ;
    class_t         *p_dev_class  ;
    device_t        *p_dev_device ;
    int             major ;
    int             minor ;
    nd_t            *node ;

    io_irq_des_t    io_irq_des[KEY_CNT];        //
    uint16_t        io_irq_event ;              //irq产生的事件号
} gpio_irq_dev_t ;

static gpio_irq_dev_t m_gpio_irq_dev ;

/**
 *@brief 定时器回调函数  void (*function)(unsigned long);
 */
static void irq0_timer_cb_handler( unsigned long param)
{

    io_irq_des_t *p_io_irq_des = (io_irq_des_t *)param ;

    sub_event_t sub_event ;

    if( gpio_get_value(p_io_irq_des->gpio) == 0 )
    {
        sub_event.event = BUTTON_PUSH ;
        sub_event.d_len = sizeof(int);
        sub_event.p_data = (uint8_t *)&p_io_irq_des->gpio ;

        trace_infoln("%s push" , p_io_irq_des->name );
    }
    else
    {
        sub_event.event = BUTTON_RELEASE ;
        sub_event.d_len = sizeof(int);
        sub_event.p_data = (uint8_t *)&p_io_irq_des->gpio ;
        trace_infoln("%s release",p_io_irq_des->name);
    }

    sub_event_push(&sub_event);
}


/**
 *@brief 中断回调函数   irqreturn_t (*irq_handler_t) (int, void *)
 *@param irq_vector   [in]中断号
 *@param param        [in]传参
 */
static irqreturn_t irq_cb_handler( int irq_vector , void *param )
{

    io_irq_des_t *p_io_irq_des = (io_irq_des_t *)param ;
    mod_timer( &p_io_irq_des->irq_timer , jiffies + msecs_to_jiffies(KEY_DELAY)  );

    return IRQ_RETVAL(IRQ_HANDLED)  ;
}


/**
 * @brief 设备打开函数 
 * @brief p_inode   [in]
 * @brief p_filp    [out]       一般用来传参到设备操作结构体的其他接口
 */
static int dev_open( inode_t *p_inode , file_t *p_filp  )
{
    p_filp->private_data = &m_gpio_irq_dev ; 

    return 0 ;
}


/**
 * @brief 设备读函数 
 *
 * @param   p_filp  [in]    要打开的设备文件(文件描述符)
 * @param   buff    [out]   返回给用户空间的数据缓冲区
 * @param   cnt     [in]    用户要读取的数据字节长度
 * @param   offt    [in]    相对于文件首地址的偏移
 * 
 * @return   读取的字节数，如果为负值，表示读取失败
 */
//ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
static ssize_t chr_drv_read( file_t *p_filp , char __user *buff  , size_t cnt , loff_t *offt  )
{
    int err_code = 0 ;

    //uint8_t     *p_data = NULL ;
    uint8_t     p_data[1024] ;
    uint16_t    data_sz = 0;

    gpio_irq_dev_t *p_dev = (gpio_irq_dev_t *)p_filp->private_data ;

    memset(p_data,0,sizeof(p_data));

    event_queue_to_buff(p_data , &data_sz);

    trace_infoln("event_queue_to_buff:");
    trace_dump(p_data,data_sz);

    err_code = copy_to_user(buff, p_data, data_sz );

    event_queue_clear();    //读完之后要清掉buf
    return data_sz ;
}

//设备操作数据结构体
static file_operations_t dev_fops ={
    .owner = THIS_MODULE ,
    .open  = dev_open,
    .read  = chr_drv_read ,
};

/**
 *@brief 设备节点相关的中断，定时器初始化 
 */
static int dev_node_irq_timer_init(void)
{
    int         err_code ;
    uint8_t     i = 0 ;

    memset(&m_gpio_irq_dev , 0 , sizeof(gpio_irq_dev_t) );

    /*从设备树中获取数据*/
    /*1.获取设备节点 */
    m_gpio_irq_dev.node = of_find_node_by_path("/cxnkey"); //根节点下的设备节点
    if( NULL == m_gpio_irq_dev.node )
    {
        trace_errorln("can't find dev node -- cxnkey ");
        return -EINVAL;
    }
    else
    {
        trace_debugln("find dev node -- cxnkey");
    }

    /*2获取设备树上自定义属性的gpio编号*/
    for( i = 0 ; i < KEY_CNT ; i++ )
    {
        m_gpio_irq_dev.io_irq_des[i].gpio = of_get_named_gpio( m_gpio_irq_dev.node , "key_gpio", 0);
        if( m_gpio_irq_dev.io_irq_des[i].gpio < 0 )
        {
            trace_errorln("gpio_index can not find\n");
            return -EINVAL;
        }

        memset(m_gpio_irq_dev.io_irq_des[i].name , 0 , sizeof(m_gpio_irq_dev.io_irq_des[i].name) );
        sprintf(m_gpio_irq_dev.io_irq_des[i].name, "key%d", i);		/* 组合名字 */

//        gpio_request(m_gpio_irq_dev.io_irq_des[i].gpio, m_gpio_irq_dev.io_irq_des[i].name);
        gpio_direction_input(m_gpio_irq_dev.io_irq_des[i].gpio);

        m_gpio_irq_dev.io_irq_des[i].irq_vector = irq_of_parse_and_map( m_gpio_irq_dev.node , i );


        m_gpio_irq_dev.io_irq_des[i].irq_handler_t = irq_cb_handler ;   //全部用同一个回调，在回调里，根据传参数据来判断按键
        
        m_gpio_irq_dev.io_irq_des[i].key_value = i ;

        err_code = request_irq( m_gpio_irq_dev.io_irq_des[i].irq_vector ,   \
                                m_gpio_irq_dev.io_irq_des[i].irq_handler_t ,    \
                                IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,   \
                                m_gpio_irq_dev.io_irq_des[i].name,          \
                                (void *)&m_gpio_irq_dev.io_irq_des[i] );
        if( err_code < 0 )
        {
            trace_errorln("request_irq error\n");
            return -EINVAL;
        }
        /*初始化定时器*/      
        init_timer(&m_gpio_irq_dev.io_irq_des[i].irq_timer); //先申请空间，在赋值
        m_gpio_irq_dev.io_irq_des[i].irq_timer.function = irq0_timer_cb_handler ;    
        m_gpio_irq_dev.io_irq_des[i].irq_timer.data = (unsigned long)&m_gpio_irq_dev.io_irq_des[i] ;   //给定时器传参中断描述符       
    }

    return 0 ;
}

/**
 *@brief 设备初始化 
 */
static int dev_node_init(void)
{
    alloc_chrdev_region( &m_gpio_irq_dev.dev_id , 0 , CDEV_CNT , KEY_NAME );  //申请设备号
    m_gpio_irq_dev.major = MAJOR(m_gpio_irq_dev.dev_id);    //通过设备号获取主，次设备号
    m_gpio_irq_dev.minor = MINOR(m_gpio_irq_dev.dev_id);

    m_gpio_irq_dev.cdev.owner = THIS_MODULE ;                   //初始化，添加一个cdev
    cdev_init(&m_gpio_irq_dev.cdev , &dev_fops );
    cdev_add(&m_gpio_irq_dev.cdev, m_gpio_irq_dev.dev_id , CDEV_CNT );

    m_gpio_irq_dev.p_dev_class = class_create (THIS_MODULE, KEY_NAME );
    if( IS_ERR(m_gpio_irq_dev.p_dev_class) )
    {
        return PTR_ERR(m_gpio_irq_dev.p_dev_class);
    }

    m_gpio_irq_dev.p_dev_device = device_create( m_gpio_irq_dev.p_dev_class,      // in ，创建哪一类
                                        NULL,                                   //一般为NULL
                                        m_gpio_irq_dev.dev_id ,                  //设备号
                                        NULL,                                   //一般为NULL
                                        KEY_NAME );                   //挂载到/dev/目录下的设备节点名称                     
    if( IS_ERR(m_gpio_irq_dev.p_dev_device) )
    {
        return PTR_ERR(m_gpio_irq_dev.p_dev_device);
    }

    return 0 ;
}


/**
 * @brief 设备驱动入口函数
 */
static int __init dev_init_entry(void)
{

    int         err_code ;

    err_code = dev_node_irq_timer_init();
    if( err_code != 0 )
    {
        trace_errorln("dev_node_irq_timer_init error ");
        return -EINVAL;
    }

    err_code = dev_node_init();
    if( err_code != 0 )
    {
        trace_errorln("dev_node_init error ");
        return -EINVAL;
    }

    event_queue_init();

    trace_infoln( "dev init success");

    return 0 ;
}

/**
 * @brief 设备驱动出口函数
 */
static void __exit dev_exit(void)
{
    uint8_t i ;

    for( i = 0 ; i < KEY_CNT ; i ++)
    {
        gpio_free(m_gpio_irq_dev.io_irq_des[i].gpio);
        del_timer_sync(&m_gpio_irq_dev.io_irq_des[i].irq_timer);
        free_irq( m_gpio_irq_dev.io_irq_des[i].irq_vector, (void *)&m_gpio_irq_dev.io_irq_des[i] );
    }

    cdev_del( &m_gpio_irq_dev.cdev );
    unregister_chrdev_region( m_gpio_irq_dev.dev_id , CDEV_CNT );

    device_destroy( m_gpio_irq_dev.p_dev_class , m_gpio_irq_dev.dev_id );
    class_destroy(m_gpio_irq_dev.p_dev_class);

    trace_infoln( "dev exit success");
}


module_init(dev_init_entry);
module_exit(dev_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息





