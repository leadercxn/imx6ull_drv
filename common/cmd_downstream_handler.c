#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>

#include <my_dev/common/user_util.h>
#include <my_dev/common/cmd_downstream_handler.h>

//用户自定义头文件
#define TRACE_MODULE    "cmd_downstream_handler.c"

#include <my_dev/common/trace.h>

/**
 * @brief 从buf数据缓存区，获取下行命令的总个数
 * @param[in] buf
 * 
 * @return    n          命令总个数
 *            (-EINVAL)  错误返回 
 */
int cmd_total_num_get(uint8_t *buf)
{
    if( buf == NULL )
    {
        trace_errorln("ERROR:  the buf is nul");
        return -EINVAL ;
    }

    return (int)buf[0];
}
EXPORT_SYMBOL(cmd_total_num_get);

/**
 * @brief 从buf数据缓存区，提取第index号命令，存放到 sub_cmd 数据结构
 * 
 * @param[in] index
 * @param[in] buf
 * @param[out] sub_cmd
 * 
 * @return    0          成功
 *            (-EINVAL)  错误返回 
 */
int index_sub_cmd_get(uint8_t index , uint8_t *buf , sub_cmd_t *sub_cmd)
{
    uint8_t     i = 0 ;
    uint16_t    buf_index = 1 ;     //绕过协议的第一字节（命令个数）
    uint8_t     sub_cmd_d_len = 0 ;

    sub_cmd->p_data = NULL ;

    if( index > cmd_total_num_get(buf) )
    {
        trace_errorln("ERROR:  the index sub_cmd out of total");
        return -EINVAL ;
    }

    for( i = 1 ; i < index ; i++ )          //过滤掉之前没选中的命令
    {
        buf_index ++ ;  //cmd
        sub_cmd_d_len = buf[ buf_index ];
        buf_index ++ ;  //d_len

        buf_index += sub_cmd_d_len ;
    }

    sub_cmd->cmd = buf[buf_index];
    buf_index++;

    sub_cmd->d_len = buf[buf_index];
    buf_index++;

    sub_cmd->p_data = &buf[buf_index] ;
    memcpy( sub_cmd->p_data , &buf[buf_index] , sub_cmd->d_len  );

    return 0 ;
}
EXPORT_SYMBOL(index_sub_cmd_get);

/**
 * @brief 获取子命令数据结构中的cmd
 * 
 * @param[in] sub_cmd   子命令数据结构变量
 * 
 * @return    cmd        命令
 *            (-EINVAL)  错误返回 
 */
int sub_cmd_cmd_get(sub_cmd_t *sub_cmd)
{
    if( sub_cmd == NULL )
    {
        trace_errorln("ERROR:  pass in param is invail");
        return -EINVAL ;
    }

    return (int)sub_cmd->cmd ;
}
EXPORT_SYMBOL(sub_cmd_cmd_get);

/**
 * @brief 获取子命令数据结构中的的 d_len
 * 
 * @param[in] sub_cmd   子命令数据结构变量
 * 
 * @return    d_len      数据长度
 *            (-EINVAL)  错误返回 
 */
int sub_cmd_d_len_get(sub_cmd_t *sub_cmd)
{
    if( sub_cmd == NULL )
    {
        trace_errorln("ERROR:  pass in param is invail");
        return -EINVAL ;
    }

    return (int)sub_cmd->d_len ;
}
EXPORT_SYMBOL(sub_cmd_d_len_get);
