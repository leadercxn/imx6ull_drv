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
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>

//路径是内核源代码  arch/arm/include/
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>

//用户自定义头文件
#define TRACE_MODULE    "drv_block.c"

#include <user_util.h>
#include <trace.h>

#define RAMDISK_SIZE    (2 * 1024 * 1024)   //容量大小为2M
#define RAMDISK_NAME    "ramdisk"           //名字
#define RAMDISK_PARTITION_CNT   3           //分区

typedef struct
{
    int         major;              //主设备号
    uint8_t     *p_ramdisk_buff;    //ram的储存空间，模拟块设备
    spinlock_t  lock;               //自选锁
    struct gendisk  *p_gendisk;     //gendisk,磁盘
    struct request_queue *p_queue; //请求队列
} ramdisk_dev_t;

static ramdisk_dev_t m_ramdisk;     //该驱动模拟的块设备

/**
 * @块设备的open接口
 */
static int ramdisk_open(struct block_device *p_dev,fmode_t mode)
{
    trace_infoln("ramdisk open");
    return 0;
}

/**
 * @块设备的release接口
 */
static void ramdisk_release(struct gendisk *p_gendisk,fmode_t mode)
{
    trace_infoln("ramdisk release");
}

/**
 * @块设备的 getgeo 接口
 */
static int ramdisk_getgeo(struct block_device *p_block_dev, struct hd_geometry *p_geometry)
{
    //相对于机械盘
    p_geometry->heads = 2;          //柱头
    p_geometry->cylinders = 32;     //柱面
    p_geometry->sectors = RAMDISK_SIZE / (2 * 32 *512); /* 一个磁道上的扇区数量 */

    return 0;
}

/* 
 * 块设备操作函数 
 */
static struct block_device_operations ramdisk_fops =
{
	.owner	 = THIS_MODULE,
	.open	 = ramdisk_open,
	.release = ramdisk_release,
	.getgeo  = ramdisk_getgeo,
};

static void ramdisk_transfer(struct request *p_req)
{
    unsigned long start = blk_rq_pos(p_req) << 9;  	/* blk_rq_pos获取到的是扇区地址，左移9位转换为字节地址 */
    unsigned long len = blk_rq_cur_bytes(p_req);

    //指针指向request的bio数据区
    void *buff = bio_data(p_req->bio);

    /**
     * 根据request 中的方向来确认数据的流向
     * 
     * 读： 从磁盘中读取数据到 buffer
     * 写： buffer保存要写入磁盘里面的数据
     */
    if(rq_data_dir(p_req) == READ)
    {
        memcpy(buff,m_ramdisk.p_ramdisk_buff+start,len);
    }
    else if(rq_data_dir(p_req) == WRITE)
    {
        memcpy(m_ramdisk.p_ramdisk_buff+start,buff,len);
    }
}

static void ramdisk_request_handler(struct request_queue *p_req_queue)
{
    int err_code = 0;

    struct request * p_req = NULL;

    //遍历request queue
    p_req = blk_fetch_request(p_req_queue);
    while(p_req != NULL)
    {
        ramdisk_transfer(p_req);

        /* 判断是否为最后一个请求，如果不是的话就获取下一个请求
		 * 循环处理完请求队列中的所有请求。
		 */
		if (!__blk_end_request_cur(p_req, err_code))
        {
            p_req = blk_fetch_request(p_req_queue);
        }
    }
}

/* 驱动入口函数 */
static int __init ramdisk_init(void)
{
    int err_code = 0;

    trace_infoln("ramdisk_init");

    // 1.申请用于ramdisk的内存
    m_ramdisk.p_ramdisk_buff = kzalloc(RAMDISK_SIZE,GFP_KERNEL);
    if(m_ramdisk.p_ramdisk_buff == NULL)
    {
        err_code = -EINVAL;
        goto kzalloc_fail;
    }

    // 2.初始化自旋锁 
	spin_lock_init(&m_ramdisk.lock);

    // 3.注册块设备
    m_ramdisk.major = register_blkdev(0, RAMDISK_NAME);   //系统自动分配设备号
    if(m_ramdisk.major < 0)
    {
        goto register_blkdev_fail;
    }

    // 4.分配并初始化 一个磁盘 gendisk
    m_ramdisk.p_gendisk = alloc_disk(RAMDISK_PARTITION_CNT);
    if(m_ramdisk.p_gendisk == NULL)
    {
        err_code = -EINVAL;
        goto alloc_gendisk_fail;
    }

    // 5. 分配并初始化请求队列
    m_ramdisk.p_queue = (struct request_queue *)blk_init_queue(ramdisk_request_handler, &m_ramdisk.lock);
    if(m_ramdisk.p_queue == NULL)
    {
        err_code = -EINVAL;
        goto init_queue_fail;
    }

    // 6. 添加（注册）disk ，把磁盘添加到内核
    m_ramdisk.p_gendisk->major = m_ramdisk.major;		/* 主设备号 */
    m_ramdisk.p_gendisk->first_minor = 0;			/* 第一个次设备号(起始次设备号) */
    m_ramdisk.p_gendisk->fops = &ramdisk_fops; 		/* 操作函数 */
    m_ramdisk.p_gendisk->private_data = &m_ramdisk;	/* 私有数据 */
    m_ramdisk.p_gendisk->queue = (struct request_queue *)m_ramdisk.p_queue;   /* 请求队列 */
    sprintf(m_ramdisk.p_gendisk->disk_name, RAMDISK_NAME); /* 名字 */
    set_capacity(m_ramdisk.p_gendisk, RAMDISK_SIZE/512);	/* 设备容量(单位为扇区) */
	add_disk(m_ramdisk.p_gendisk);

    return 0;


init_queue_fail:
    put_disk(m_ramdisk.p_gendisk);

alloc_gendisk_fail:
    unregister_blkdev(m_ramdisk.major, RAMDISK_NAME);

register_blkdev_fail:
    kfree(m_ramdisk.p_ramdisk_buff);

kzalloc_fail:
    return err_code;

}

/* 驱动出口函数 */
static void __exit ramdisk_exit(void)
{
    trace_infoln("ramdisk_exit");

    /* 释放gendisk */
	del_gendisk(m_ramdisk.p_gendisk);
	put_disk(m_ramdisk.p_gendisk);

	/* 清除请求队列 */
	blk_cleanup_queue((struct request_queue *)m_ramdisk.p_queue);

	/* 注销块设备 */
	unregister_blkdev(m_ramdisk.major, RAMDISK_NAME);

	/* 释放内存 */
	kfree(m_ramdisk.p_ramdisk_buff); 
}

module_init(ramdisk_init);
module_exit(ramdisk_exit);

MODULE_LICENSE("GPL");       //添加模块 LICENSE 信
MODULE_AUTHOR("cxn");        //添加模块作者信息
