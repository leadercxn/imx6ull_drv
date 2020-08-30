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

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

/*重新定义*/
typedef struct inode               inode_t ;
typedef struct file                file_t ;
typedef struct file_operations     file_operations_t ;
typedef struct cdev                cdev_t ;
typedef struct device              device_t ;
typedef struct class               class_t ;
typedef struct device_node         nd_t ;
typedef struct property            proper_t ;

typedef struct 
{
    dev_t       dev_id ;  
    cdev_t      cdev   ;
    class_t     *p_dev_class  ;
    device_t    *p_dev_device ;
    int         major ;
    int         minor ;
    nd_t        *node ;
    int         gpio_index;
} dts_chr_dev_t ;

dts_chr_dev_t  m_dts_chr_dev;  //定义一个设备书书据结构体


#define CDEV_CNT    1               //设备的个数
#define LED_NAME    "dts_pinctrl_gpio_chr_led"

#define LED_BIT     3

/*物理地址*/
#define CCM_CCGR1_BASE          0x020C406C
#define SW_MUX_GPIO1_IO03_BASE  0x020E0068
#define SW_PAD_GPIO1_IO03_BASE  0x020E02F4
#define GPIO1_DR_BASE           0x0209C000
#define GPIO1_GDIR_BASE         0x0209C004

/*虚拟地址*/
static void  __iomem *   virtual_ccm_ccgr1 ;
static void  __iomem *   virtual_sw_mux_gpio1_io03 ;
static void  __iomem *   virtual_sw_pad_gpio1_io03 ;
static void  __iomem *   virtual_gpio1_dr ;
static void  __iomem *   virtual_gpio1_gdir ;

static uint32_t    m_regdata[14] ;

/**
 * @brief 控制led开关
 */
static void led_onoff(bool sw)
{
#if 0
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
#endif
    if( true == sw )
    {
        gpio_set_value( m_dts_chr_dev.gpio_index , 0 ) ;
    }
    else
    {
        gpio_set_value( m_dts_chr_dev.gpio_index , 1 ) ;
    }

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

    printk("\nfile pri data major = %d , minor = %d\n", p_dts_chr_dev->major , p_dts_chr_dev->minor );
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
        printk("app write data to kernel fail\n");
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
    .read  = chr_drv_read ,
    .write = chr_drv_write,
};

/**
 * @brief 硬件初始化
 */
static void hw_init(void)
{
    uint32_t    val = 0 ;

#if 0
    //寄存器映射
    virtual_ccm_ccgr1 = ioremap(CCM_CCGR1_BASE , 4);
    virtual_sw_mux_gpio1_io03 = ioremap(SW_MUX_GPIO1_IO03_BASE , 4);
    virtual_sw_pad_gpio1_io03 = ioremap(SW_PAD_GPIO1_IO03_BASE , 4);
    virtual_gpio1_dr = ioremap(GPIO1_DR_BASE , 4);
    virtual_gpio1_gdir = ioremap(GPIO1_GDIR_BASE , 4);
#endif 
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
 * @brief 字符设备驱动入口函数
 */
static int __init chr_dev_init(void)
{
#if 0
    hw_init();          //硬件初始化

    led_onoff(false);   //设置缺省值

    alloc_chrdev_region( &m_standard_chr_dev.dev_id , 0 , CDEV_CNT , LED_NAME );  //申请设备号
    m_standard_chr_dev.major = MAJOR(m_standard_chr_dev.dev_id);    //通过设备号获取主，次设备号
    m_standard_chr_dev.minor = MINOR(m_standard_chr_dev.dev_id);
    printk("major = %d , minor = %d\n", m_standard_chr_dev.major , m_standard_chr_dev.minor );

    m_standard_chr_dev.cdev.owner = THIS_MODULE ;                   //初始化，添加一个cdev
    cdev_init(&m_standard_chr_dev.cdev , &chr_cdev_fops );
    cdev_add(&m_standard_chr_dev.cdev, m_standard_chr_dev.dev_id , CDEV_CNT );

    m_standard_chr_dev.p_dev_class =  class_create (THIS_MODULE, LED_NAME );
    if( IS_ERR(m_standard_chr_dev.p_dev_class) )
    {
        return PTR_ERR(m_standard_chr_dev.p_dev_class);
    }

    m_standard_chr_dev.p_dev_device = device_create( m_standard_chr_dev.p_dev_class,            // in ，创建哪一类
                                        NULL,                                      //一般为NULL
                                        m_standard_chr_dev.dev_id ,                //设备号
                                        NULL,                  //一般为NULL
                                        "standard_chr_led" );
    if( IS_ERR(m_standard_chr_dev.p_dev_device) )
    {
        return PTR_ERR(m_standard_chr_dev.p_dev_device);
    }

    printk("Success: standerd chr led register\n");
    return 0 ;
#endif

    int         err_code ;
    const char  *str = NULL ;
    proper_t    *proper = NULL ;
    uint8_t     i = 0 ;

    memset( m_regdata , 0 , sizeof(uint32_t)*sizeof(m_regdata) );

#if 0
    /*从设备树中获取数据*/
    /*1.获取设备节点 */
    m_dts_chr_dev.node = of_find_node_by_path("/cxnled"); //根节点下的设备节点
    if( NULL == m_dts_chr_dev.node )
    {
        printk("Fail: cxnled node can not find\n");
        return -EINVAL;
    }
    else
    {
        printk("Success: cxnled has been found\n");
    }
    /*2.获取设备节点 */
    proper = of_find_property(m_dts_chr_dev.node, "compatible", NULL);
    if( NULL == proper )
    {
        printk("Fail: compatible can not be found\n");
        return -EINVAL;
    }
    else
    {
        printk("Success: compatible has been found\n");
    }
    /*3.获取status属性内容 */
    err_code = of_property_read_string(m_dts_chr_dev.node, "status", &str);
    if( err_code < 0  )
    {
        printk("Fail: status can not be found\n");
        return -EINVAL;
    }
    else
    {
        printk("Success: status has been found\n");
    }
    /*4.获取reg属性内容 */
    err_code = of_property_read_u32_array(m_dts_chr_dev.node, "reg", m_regdata, 10);
    if( err_code < 0  )
    {
        printk("Fail: reg can not be found\n");
        return -EINVAL;
    }
    else
    {
        printk("Success: reg has been found\n");
        printk("reg data:\n");
        
        for( i = 0 ; i < 10 ; i++ )
        {
            printk("%#X ", m_regdata[i]);
        }
        printk("\n");
    }

    hw_init();      //硬件初始化
#endif

    /*从设备树中获取数据*/
    /*1.获取设备节点 */
    m_dts_chr_dev.node = of_find_node_by_path("/cxn_pinctrl_beep"); //根节点下的设备节点
    if( NULL == m_dts_chr_dev.node )
    {
        printk("Fail: cxn_pinctrl_beep node can not find\n");
        return -EINVAL;
    }
    else
    {
        printk("Success: cxn_pinctrl_beep has been found\n");
    }

    /*获取设备树上自定义的gpio编号*/
    m_dts_chr_dev.gpio_index = of_get_named_gpio( m_dts_chr_dev.node , "led_gpio", 0);
    if( m_dts_chr_dev.gpio_index < 0 )
    {
        printk("Fail: cxn_pinctrl_beep node can not find\n");
        return -EINVAL;
    }
    printk("Success: m_dts_chr_dev.gpio_index = %d \n" , m_dts_chr_dev.gpio_index );

    /*设置IO*/
    err_code = gpio_direction_output(m_dts_chr_dev.gpio_index, 1);
    if( err_code < 0 )
    {
        printk("Fail:the gpio set is fail \n");
    }


    alloc_chrdev_region( &m_dts_chr_dev.dev_id , 0 , CDEV_CNT , LED_NAME );  //申请设备号
    m_dts_chr_dev.major = MAJOR(m_dts_chr_dev.dev_id);    //通过设备号获取主，次设备号
    m_dts_chr_dev.minor = MINOR(m_dts_chr_dev.dev_id);
    printk("major = %d , minor = %d\n", m_dts_chr_dev.major , m_dts_chr_dev.minor );

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
                                        "cxn_pinctrl_beep" );                   //挂载到/dev/目录下的设备节点名称                     
    if( IS_ERR(m_dts_chr_dev.p_dev_device) )
    {
        return PTR_ERR(m_dts_chr_dev.p_dev_device);
    }

    printk("dts: dts pinctrl chr led register\n");
    return 0 ;


}

/**
 * @brief 字符设备驱动出口函数
 */
static void __exit chr_dev_exit(void)
{
    led_onoff(false);
#if 0
    //取消映射
    iounmap(virtual_ccm_ccgr1);
    iounmap(virtual_sw_mux_gpio1_io03);
    iounmap(virtual_sw_pad_gpio1_io03);
    iounmap(virtual_gpio1_dr);
    iounmap(virtual_gpio1_gdir);
#endif

    gpio_free(m_dts_chr_dev.gpio_index);
    cdev_del( &m_dts_chr_dev.cdev );
    unregister_chrdev_region( m_dts_chr_dev.dev_id , CDEV_CNT );

    device_destroy( m_dts_chr_dev.p_dev_class , m_dts_chr_dev.dev_id );
    class_destroy(m_dts_chr_dev.p_dev_class);

    printk("Success: dts pinctrl chr led exit\n");
}


module_init(chr_dev_init);
module_exit(chr_dev_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息

