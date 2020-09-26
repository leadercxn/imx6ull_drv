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
#define TRACE_MODULE    "drv_input.c"

#include <user_util.h>
#include <trace.h>
#include <event_upstream_handler.h>

#define DRV_NAME_IN_SYS    "drv_input"

#define INPUT_KEY_CNT     1               //按键数量

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
    int              gpio;                  //gpio
    unsigned int     irq_vector;            //中断号
    uint8_t          key_value;             //键值
    char             name[20];              //名字
    irqreturn_t      (*irq_handler_t) (int, void *);  //中断服务回调函数
    timer_list_t     irq_timer;             //每个按键中断对应一个定时器，用来消抖
    input_dev_t      *p_sub_input_dev;         //
} io_irq_des_t;

/**
 *@brief 设备结构体 
 */ 
typedef struct 
{
    nd_t            *node;

    io_irq_des_t    io_irq_des[INPUT_KEY_CNT];       //
    uint16_t        io_irq_event;              //irq产生的事件号
} drv_input_t;

static drv_input_t m_drv_input;


/**
 *@brief 定时器回调函数  void (*function)(unsigned long);
 */
static void irq0_timer_cb_handler( unsigned long param)
{
    int key_value;

    io_irq_des_t *p_io_irq_des = (io_irq_des_t *)param ;

    key_value = gpio_get_value(p_io_irq_des->gpio);

    trace_infoln("key_value = %d",key_value);
    /**
     * 向input子系统上报事件
     */
    input_event(p_io_irq_des->p_sub_input_dev,EV_KEY,KEY_0,key_value);
    input_sync(p_io_irq_des->p_sub_input_dev);
}

/**
 *@brief 中断回调函数   irqreturn_t (*irq_handler_t) (int, void *)
 *@param irq_vector   [in]中断号
 *@param param        [in]传参
 */
static irqreturn_t irq_cb_handler( int irq_vector , void *param )
{
    io_irq_des_t *p_io_irq_des = (io_irq_des_t *)param;
    mod_timer( &p_io_irq_des->irq_timer , jiffies + msecs_to_jiffies(KEY_DELAY)  );

    return IRQ_RETVAL(IRQ_HANDLED);
}

/**
 *@brief 设备节点相关的中断、定时器初始化 
 */
static int dev_node_irq_timer_init(void)
{
    int         err_code;
    uint8_t     i = 0;

    memset(&m_drv_input , 0 , sizeof(drv_input_t) );

    /*从设备树中获取数据*/
    /*1.获取设备节点 */
    m_drv_input.node = of_find_node_by_path("/cxnkey"); //根节点下的设备节点
    if( NULL == m_drv_input.node )
    {
        trace_errorln("can't find dev node -- cxnkey ");
        return -EINVAL;
    }
    else
    {
        trace_debugln("find dev node -- cxnkey");
    }

    /*2获取设备树上自定义属性的gpio编号*/
    for( i = 0 ; i < INPUT_KEY_CNT ; i++ )
    {
        m_drv_input.io_irq_des[i].gpio = of_get_named_gpio( m_drv_input.node , "key_gpio", 0);
        if( m_drv_input.io_irq_des[i].gpio < 0 )
        {
            trace_errorln("gpio_index can not find\n");
            return -EINVAL;
        }

        memset(m_drv_input.io_irq_des[i].name , 0 , sizeof(m_drv_input.io_irq_des[i].name) );
        sprintf(m_drv_input.io_irq_des[i].name, "key%d", i);		/* 组合名字 */

        gpio_direction_input(m_drv_input.io_irq_des[i].gpio);

        /**
         * 输入子系统参数的初始化
         */
        m_drv_input.io_irq_des[i].p_sub_input_dev = input_allocate_device();
        m_drv_input.io_irq_des[i].p_sub_input_dev->name =  DRV_NAME_IN_SYS;

        __set_bit(EV_KEY, m_drv_input.io_irq_des[i].p_sub_input_dev->evbit); /* 设置产生按键事件 */
/**
 * 不设置为重复事件，不然app层会一直有打印
 */
#if 0
        __set_bit(EV_REP, m_drv_input.io_irq_des[i].p_sub_input_dev->evbit); /* 重复事件 */
#endif
        __set_bit(KEY_0, m_drv_input.io_irq_des[i].p_sub_input_dev->keybit); /*设置产生哪些按键值 */

        if(input_register_device(m_drv_input.io_irq_des[i].p_sub_input_dev) < 0)
        {
            trace_errorln("input_register_device error\n");
        }

        /**
         * 申请中断，定时器中断
         */
        m_drv_input.io_irq_des[i].irq_vector = irq_of_parse_and_map( m_drv_input.node , i );

        m_drv_input.io_irq_des[i].irq_handler_t = irq_cb_handler ;   //全部用同一个回调，在回调里，根据传参数据来判断按键
        
        m_drv_input.io_irq_des[i].key_value = i ;

        err_code = request_irq( m_drv_input.io_irq_des[i].irq_vector ,   \
                                m_drv_input.io_irq_des[i].irq_handler_t ,    \
                                IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,   \
                                m_drv_input.io_irq_des[i].name,          \
                                (void *)&m_drv_input.io_irq_des[i] );
        if( err_code < 0 )
        {
            trace_errorln("request_irq error\n");
            return -EINVAL;
        }
        /*初始化定时器*/      
        init_timer(&m_drv_input.io_irq_des[i].irq_timer); //先申请空间，在赋值
        m_drv_input.io_irq_des[i].irq_timer.function = irq0_timer_cb_handler ;    
        m_drv_input.io_irq_des[i].irq_timer.data = (unsigned long)&m_drv_input.io_irq_des[i] ;   //给定时器传参中断描述符

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


    trace_infoln( "dev init success");

    return 0 ;
}

/**
 * @brief 设备驱动出口函数
 */
static void __exit dev_exit(void)
{
    uint8_t     i = 0;

    for( i = 0 ; i < INPUT_KEY_CNT ; i ++)
    {
        gpio_free(m_drv_input.io_irq_des[i].gpio);
        del_timer_sync(&m_drv_input.io_irq_des[i].irq_timer);
        free_irq( m_drv_input.io_irq_des[i].irq_vector, (void *)&m_drv_input.io_irq_des[i] );
    }

    for( i = 0 ; i < INPUT_KEY_CNT ; i++ )
    {
        input_unregister_device(m_drv_input.io_irq_des[i].p_sub_input_dev); /* 注销 input_dev */
        input_free_device(m_drv_input.io_irq_des[i].p_sub_input_dev);       /* 删除 input_dev */
    }

    trace_infoln( "dev exit success");
}

#undef INPUT_KEY_CNT

module_init(dev_init_entry);
module_exit(dev_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息

