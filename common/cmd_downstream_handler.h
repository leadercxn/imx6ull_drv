#ifndef __CMD_DOWNSTREAM_HANDLER_H__
#define __CMD_DOWNSTREAM_HANDLER_H__


/**
 * 命令类型的枚举
 */
typedef enum
{
    CMD_START ,
    CMD_STOP ,
    CMD_MODIFY,
} cmd_e ;


typedef struct 
{
    uint8_t cmd ;
    uint8_t d_len ;
    uint8_t *p_data ;
} sub_cmd_t;

int cmd_total_num_get(uint8_t *buf);
int index_sub_cmd_get(uint8_t index , uint8_t *buf , sub_cmd_t *sub_cmd);
int sub_cmd_cmd_get(sub_cmd_t *sub_cmd);
int sub_cmd_d_len_get(sub_cmd_t *sub_cmd);


#endif

