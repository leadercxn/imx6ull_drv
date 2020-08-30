#ifndef __EVENT_UPSTREAM_HANDLER_H__
#define __EVENT_UPSTREAM_HANDLER_H__


#define SUB_EVENT_TOTAL_MAX     20

/*子事件数据结构*/
typedef struct 
{
    uint8_t     event ;
    uint8_t     d_len ;
    uint8_t     *p_data ;
} sub_event_t ;
 
/*事件上行数据结构*/
typedef struct 
{
    uint8_t         event_total ;
    sub_event_t     sub_event[ SUB_EVENT_TOTAL_MAX ] ;
} event_queue_t  ;

int sub_event_push(sub_event_t *sub_event);

event_queue_t * p_event_queue_get(void);

void event_queue_clear(void);

void event_queue_init(void);

void event_queue_to_buff(uint8_t *buf , uint16_t *buf_sz);

#endif

















