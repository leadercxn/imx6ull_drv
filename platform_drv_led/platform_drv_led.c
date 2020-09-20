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
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

//用户自定义头文件
#define TRACE_MODULE    "drv_pl.c"

#include <user_util.h>
#include <trace.h>

typedef struct 
{
    dev_t       dev_id ;  
    cdev_t      cdev   ;
    class_t     *p_dev_class  ;
    device_t    *p_dev_device ;
    int         major ;
    int         minor ;
    nd_t        *node ;
} dts_chr_dev_t ;

static dts_chr_dev_t  m_dts_chr_dev;            //定义一个设备书书据结构体


#define CDEV_CNT            1                   //设备的个数
#define LED_NAME            "pl_drv_led"
#define DRV_NAME_IN_SYS     "drv_pl_led"        //挂载到/dev/目录下的设备节点名称

#define LED_BIT     3

/*虚拟地址*/
static void  __iomem *   virtual_ccm_ccgr1;
static void  __iomem *   virtual_sw_mux_gpio1_io03;
static void  __iomem *   virtual_sw_pad_gpio1_io03;
static void  __iomem *   virtual_gpio1_dr;
static void  __iomem *   virtual_gpio1_gdir;

static uint32_t    m_regdata[14] ;

static const struct of_device_id m_leds_table_id[] = {
    {.compatible = "cxnled"},
    { }
};

MODULE_DEVICE_TABLE(of,m_leds_table_id);  //声明 MODULE_DEVICE_TABLE 声明 leds_table_id 设备匹配表

/**
 * @brief 控制led开关
 */
static void led_onoff(bool sw)
{
    u32 val = readl(virtual_gpio1_dr) ; 
    
    if( true == sw )
    {
        val &=~ (1 << LED_BIT);
    }
    else
    {
         val |= (1 << LED_BIT);
    }

    writel(val , virtual_gpio1_dr);
}

/**
 * @brief 打开字符设备 
 */
static int chr_drv_open( inode_t *p_inode , file_t *p_filp  )
{
    p_filp->private_data = &m_dts_chr_dev ;
    return 0 ;
}

/**
 * @brief 释放/关闭字符设备
 */
static int chr_drv_release( inode_t *p_inode , file_t *p_filp )
{
    dts_chr_dev_t  *p_dts_chr_dev = (dts_chr_dev_t  *)p_filp->private_data;

    trace_infoln("\nfile major = %d , minor = %d\n", p_dts_chr_dev->major , p_dts_chr_dev->minor );
    return 0 ;
}

/**
 * @brief 写字符设备
 */
//ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t chr_drv_write( file_t *p_filp , const char __user *buff , size_t cnt , loff_t *offt )
{
    int err_code ;
    uint8_t  rx_buf[1];

    err_code = copy_from_user( rx_buf , buff , cnt );   //从用户空间拷贝数据到内核空间,驱动属于内核，即从应用层拷到内核
    if(err_code < 0 )
    {
        trace_errorln("app write data to kernel fail");
        return -EFAULT ;
    }

    if( true == rx_buf[0] )
    {
        led_onoff(true);
    }
    else
    {
        led_onoff(false);
    }

    return 0 ;
}

//字符设备操作数据结构体
static file_operations_t chr_cdev_fops ={
    .owner = THIS_MODULE ,
    .open  = chr_drv_open,
    .release = chr_drv_release,
    .write = chr_drv_write,
};

/**
 * @brief 硬件初始化
 */
static void hw_init(void)
{
    uint32_t    val = 0 ;

    //寄存器映射
    virtual_ccm_ccgr1 = of_iomap(m_dts_chr_dev.node , 0);
    virtual_sw_mux_gpio1_io03 = of_iomap(m_dts_chr_dev.node , 1);
    virtual_sw_pad_gpio1_io03 = of_iomap(m_dts_chr_dev.node , 2);
    virtual_gpio1_dr = of_iomap(m_dts_chr_dev.node , 3);
    virtual_gpio1_gdir = of_iomap(m_dts_chr_dev.node , 4);


    //用户自定义代码
        //使能GPIO时钟
        val = readl(virtual_ccm_ccgr1);
        val &= ~(3 << 26); // 清除以前的设置
        val |= (3 << 26);  // 设置新值
        writel(val, virtual_ccm_ccgr1);

        //设置gpio功能
        writel(5, virtual_sw_mux_gpio1_io03);

        //设置gpio属性
        writel(0x10B0, virtual_sw_pad_gpio1_io03);

        //设置gpio方向（输出）
        val = readl(virtual_gpio1_gdir);
        val &= ~(1 << 3); // 清除以前的设置
        val |= (1 << 3);  // 设置新值
        writel(val, virtual_gpio1_gdir);
}

/**
 * 驱动和设备数匹配之后的回调函数  匹配之后才开始初始化驱动
 */
static int pl_drv_probe(struct platform_device *p_device)
{
    int         err_code ;
    const char  *str = NULL ;
    proper_t    *proper = NULL ;

    memset( m_regdata , 0 , sizeof(uint32_t)*sizeof(m_regdata) );

    trace_infoln("pl_drv_probe");

    /*从设备树中获取数据*/
    /*1.获取设备节点 */
    m_dts_chr_dev.node = of_find_node_by_path("/cxnled"); //根节点下的设备节点
    if( NULL == m_dts_chr_dev.node )
    {
        trace_errorln("Fail: cxnled node can not find");
        return -EINVAL;
    }
    else
    {
        trace_infoln("Success: cxnled has been found");
    }

    /*2.获取设备节点 */
    proper = of_find_property(m_dts_chr_dev.node, "compatible", NULL);
    if( NULL == proper )
    {
        trace_errorln("Fail: compatible can not be found");
        return -EINVAL;
    }

    /*3.获取status属性内容 */
    err_code = of_property_read_string(m_dts_chr_dev.node, "status", &str);
    if( err_code < 0  )
    {
        trace_errorln("Fail: status can not be found");
        return -EINVAL;
    }

    /*4.获取reg属性内容 */
    err_code = of_property_read_u32_array(m_dts_chr_dev.node, "reg", m_regdata, 10);
    if(err_code < 0)
    {
        trace_errorln("Fail: reg can not be found");
        return -EINVAL;
    }

    hw_init();      //硬件初始化

    alloc_chrdev_region( &m_dts_chr_dev.dev_id , 0 , CDEV_CNT , LED_NAME );  //申请设备号
    m_dts_chr_dev.major = MAJOR(m_dts_chr_dev.dev_id);                       //通过设备号获取主，次设备号
    m_dts_chr_dev.minor = MINOR(m_dts_chr_dev.dev_id);
    trace_infoln("major = %d , minor = %d\n", m_dts_chr_dev.major , m_dts_chr_dev.minor );

    m_dts_chr_dev.cdev.owner = THIS_MODULE ;                   //初始化，添加一个cdev
    cdev_init(&m_dts_chr_dev.cdev , &chr_cdev_fops );
    cdev_add(&m_dts_chr_dev.cdev, m_dts_chr_dev.dev_id , CDEV_CNT );

    m_dts_chr_dev.p_dev_class =  class_create (THIS_MODULE, LED_NAME );
    if( IS_ERR(m_dts_chr_dev.p_dev_class) )
    {
        return PTR_ERR(m_dts_chr_dev.p_dev_class);
    }

    m_dts_chr_dev.p_dev_device = device_create( m_dts_chr_dev.p_dev_class,      // in ，创建哪一类
                                        NULL,                                   //一般为NULL
                                        m_dts_chr_dev.dev_id ,                  //设备号
                                        NULL,                                   //一般为NULL
                                        DRV_NAME_IN_SYS );                      //挂载到/dev/目录下的设备节点名称                     
    if( IS_ERR(m_dts_chr_dev.p_dev_device) )
    {
        return PTR_ERR(m_dts_chr_dev.p_dev_device);
    }

    trace_infoln("Success:init");
    return 0 ;
}

/**
 * 
 */
static int pl_drv_remove(struct platform_device *p_device)
{
    trace_infoln("pl_drv_remove");

    led_onoff(false);

    cdev_del( &m_dts_chr_dev.cdev );
    unregister_chrdev_region( m_dts_chr_dev.dev_id , CDEV_CNT );

    device_destroy( m_dts_chr_dev.p_dev_class , m_dts_chr_dev.dev_id );
    class_destroy(m_dts_chr_dev.p_dev_class);

    trace_infoln("Success:exit");

    return 0;
}

static struct platform_driver m_leds_platform_driver = {
    .driver = {
        .name = "pl_drv_led",
        .of_match_table = m_leds_table_id,
    },
    .probe = pl_drv_probe,
    .remove = pl_drv_remove,
};

/**
 * @brief 字符设备驱动入口函数
 */
static int __init chr_dev_init(void)
{
    return platform_driver_register(&m_leds_platform_driver);
}

/**
 * @brief 字符设备驱动出口函数
 */
static void __exit chr_dev_exit(void)
{
    platform_driver_unregister(&m_leds_platform_driver);
}


module_init(chr_dev_init);
module_exit(chr_dev_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息

