//路径是内核源代码  include/linux/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

//用户自己的头文件
//#include <user_util.h>

#define LED_MAJOR   200
#define LED_NAME    "led"

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

/*重新定义*/
typedef  struct inode               *inode_t ;
typedef  struct file                *file_t ;
typedef  struct file_operations     file_operations_t ;

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
static int chr_drv_open( inode_t p_inode , file_t p_filp  )
{

    return 0 ;
}

/**
 * @brief 释放/关闭字符设备
 */
static int chr_drv_release( inode_t p_inode , file_t p_filp )
{

    return 0 ;
}

/**
 * @brief 读字符设备 
 */
//ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
static ssize_t chr_drv_read( file_t p_filp , char __user *buff  , size_t cnt , loff_t *offt  )
{

    return 0 ;
}

/**
 * @brief 写字符设备
 */
//ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t chr_drv_write( file_t p_filp , const char __user *buff , size_t cnt , loff_t *offt )
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
static file_operations_t chr_dev_fops ={
    .owner = THIS_MODULE ,
    .open  = chr_drv_open,
    .release = chr_drv_release,
    .read  = chr_drv_read ,
    .write = chr_drv_write,
};

/**
 * @brief 字符设备驱动入口函数
 */
static int __init chr_dev_init(void)
{
    int         err_code  = 0 ;
    uint32_t    val = 0 ;

    //寄存器映射
    virtual_ccm_ccgr1 = ioremap(CCM_CCGR1_BASE , 4);
    virtual_sw_mux_gpio1_io03 = ioremap(SW_MUX_GPIO1_IO03_BASE , 4);
    virtual_sw_pad_gpio1_io03 = ioremap(SW_PAD_GPIO1_IO03_BASE , 4);
    virtual_gpio1_dr = ioremap(GPIO1_DR_BASE , 4);
    virtual_gpio1_gdir = ioremap(GPIO1_GDIR_BASE , 4);

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

        //设置缺省值
        led_onoff(true);

    //注册字符设备驱动
    err_code = register_chrdev(LED_MAJOR, LED_NAME, &chr_dev_fops); //早期字符设备
    if(err_code < 0)
    {
        printk("Fail:chr dev led register \n");
        return -EIO ;
    }
    printk("Success: chr dev led register\n");
    return 0 ;
}

/**
 * @brief 字符设备驱动出口函数
 */
static void __exit chr_dev_exit(void)
{
    led_onoff(false);

    //取消映射
    iounmap(virtual_ccm_ccgr1);
    iounmap(virtual_sw_mux_gpio1_io03);
    iounmap(virtual_sw_pad_gpio1_io03);
    iounmap(virtual_gpio1_dr);
    iounmap(virtual_gpio1_gdir);

    //注销设备号
    unregister_chrdev(LED_MAJOR , LED_NAME );

    printk("Success: chr dev led exit\n");
}


module_init(chr_dev_init);
module_exit(chr_dev_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息

