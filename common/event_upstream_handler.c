#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>

#include <my_dev/common/user_util.h>
#include <my_dev/common/event_upstream_handler.h>

//用户自定义头文件
#define TRACE_MODULE    "event_upstream_handler.c"

#include <my_dev/common/trace.h>

static  event_queue_t  m_event_queue ;


/**
 * @brief 获取下行缓存区的事件的总数
 * 
 * @param[in] event_total   
 */
static uint8_t event_total_get(void)
{
    return m_event_queue.event_total ;
}

/**
 * @brief 把传参的子事件数据结构放到事件列表
 * 
 * @param[in] sub_event   指向子事件数据结构变量的指针
 * @return    0    执行成功
 *            其他  执行失败
 */
int sub_event_push(sub_event_t *sub_event)
{
    if ( SUB_EVENT_TOTAL_MAX <= event_total_get() )
    {
        trace_errorln("sub_event_push fail ,the enent buff is full");
        return -ENOMEM ;
    }

    memcpy( &m_event_queue.sub_event[m_event_queue.event_total] , sub_event , sizeof(sub_event_t) );

    m_event_queue.event_total++ ;
    return 0;
}
EXPORT_SYMBOL(sub_event_push);

/**
 * @brief 获取该层的事件缓存区
 * 
 * @return    指向该层事件缓存区的指针
 */
event_queue_t * p_event_queue_get(void)
{
    return (&m_event_queue) ;
}
EXPORT_SYMBOL(p_event_queue_get);


void event_queue_clear(void)
{
    m_event_queue.event_total = 0 ;
}
EXPORT_SYMBOL(event_queue_clear);


void event_queue_init(void)
{
    m_event_queue.event_total = 0 ;
}
EXPORT_SYMBOL(event_queue_init);

void event_queue_to_buff(uint8_t *buf , uint16_t *buf_sz)
{
    uint16_t index = 0 ;
    uint8_t i = 0 ;
    buf[index] = m_event_queue.event_total ;
    index ++ ;

    for( i = 0 ; i < m_event_queue.event_total ; i++)
    {
        buf[index] = m_event_queue.sub_event[i].event ; 
        index++ ;

        buf[index] = m_event_queue.sub_event[i].d_len ;
        index++ ;

        memcpy( &buf[index] , m_event_queue.sub_event[i].p_data , m_event_queue.sub_event[i].d_len );

        index += m_event_queue.sub_event[i].d_len ;
    }

    *buf_sz = index ;
}
EXPORT_SYMBOL(event_queue_to_buff);