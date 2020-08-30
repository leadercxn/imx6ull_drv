#ifndef _TRACE_H_
#define _TRACE_H_


#define LOG_PRINTF(...)  printk(__VA_ARGS__)

#define TRACE_PRINTF        LOG_PRINTF


#ifndef TRACE_MODULE
#define TRACE_MODULE        __MODULE__
#endif 

#define TRACE_LAYER         "[Kernel]"

#define TRACE_LAYER_FORMAT  "%-10s",TRACE_LAYER

#define TRACE_WARN_FORMAT   "%-10s\t%4d [W] ", TRACE_MODULE, __LINE__

#define TRACE_ERROR_FORMAT  "%-10s\t%4d [E] ", TRACE_MODULE, __LINE__

#define TRACE_INFO_FORMAT   "%-10s\t%4d [I] ", TRACE_MODULE, __LINE__

#define TRACE_DEBUG_FORMAT  "%-10s\t%4d [D] ", TRACE_MODULE, __LINE__

#define TRACE_LINE_ENDING   "\r\n"



#define trace_warn( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_WARN_FORMAT);TRACE_PRINTF(msg, ##__VA_ARGS__); }
#define trace_warnln( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_WARN_FORMAT);TRACE_PRINTF(msg TRACE_LINE_ENDING, ##__VA_ARGS__); }

#define trace_error( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_ERROR_FORMAT);TRACE_PRINTF(msg, ##__VA_ARGS__); }
#define trace_errorln( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_ERROR_FORMAT);TRACE_PRINTF(msg TRACE_LINE_ENDING, ##__VA_ARGS__); }

#define trace_info( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_INFO_FORMAT);TRACE_PRINTF(msg, ##__VA_ARGS__); }
#define trace_infoln( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_INFO_FORMAT);TRACE_PRINTF(msg TRACE_LINE_ENDING, ##__VA_ARGS__); }

#define trace_debug( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_DEBUG_FORMAT);TRACE_PRINTF(msg, ##__VA_ARGS__); }
#define trace_debugln( msg, ... )  { TRACE_PRINTF(TRACE_LAYER_FORMAT);TRACE_PRINTF(TRACE_DEBUG_FORMAT);TRACE_PRINTF(msg TRACE_LINE_ENDING, ##__VA_ARGS__); }

static inline void trace_dump(void * p_buffer, uint32_t len)
{
    uint8_t *p = (uint8_t *)p_buffer;
    uint32_t index;
    for ( index = 0; index <  len; index++)
    {
        TRACE_PRINTF("%02X", p[index]);
    }
    TRACE_PRINTF(TRACE_LINE_ENDING);
} 

#endif


