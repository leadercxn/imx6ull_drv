
#ifndef __USER_UTIL_H__
#define __USER_UTIL_H__


/*重新定义*/
typedef struct inode               inode_t ;
typedef struct file                file_t ;
typedef struct file_operations     file_operations_t ;
typedef struct cdev                cdev_t ;
typedef struct device              device_t ;
typedef struct class               class_t ;
typedef struct device_node         nd_t ;
typedef struct property            proper_t ;
typedef struct timer_list          timer_list_t ;
typedef struct fasync_struct       fasync_struct_t;
typedef struct miscdevice          miscdevice_t;


////////////////////////// 內联函数 //////////////////////////////////

/**
 * @brief (uint16_t)value ==> *p_encoded_data
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static inline uint8_t uint16_encode(uint16_t value, uint8_t *p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}

/**
 * @brief (uint32_t)value ==> *p_encoded_data
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static inline uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}

/**
 * @brief *p_encoded_data ==> (uint16_t)value
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static inline uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}

/**
 * @brief *p_encoded_data ==> (uint16_t)value (大端)
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static inline uint16_t uint16_big_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0]) << 8 ) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1])) );
}


/**
 * @brief *p_encoded_data ==> (uint32_t)value
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static inline uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}

/**
 * @brief *p_encoded_data ==> (uint32_t)value (大端)
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static inline uint32_t uint32_big_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
}

/**
 * @brief Function for checking if a pointer value is aligned to a 4 byte boundary.
 *
 * @param[in]   p   Pointer value to be checked.
 *
 * @return      TRUE if pointer is aligned to a 4 byte boundary, FALSE otherwise.
 */
static inline bool is_word_aligned(void const* p)
{
    return (((uintptr_t)p & 0x03) == 0);
}



//////////////////////////// 函数声明 //////////////////////////////////////



#endif
