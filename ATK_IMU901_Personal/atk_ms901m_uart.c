/*
    @file atk_imu901.h
    @brief IMU901 "Serial Port Interface"
    @version 1.0
    @date 2024-07-24
    @author 
    该部分提供ATK_IMU901的串口接口定义以及函数功能
*/
#include "atk_imu901.h"

uint32_t DL_UART_transmitDataArray(UART_Regs *uart, const uint8_t *data, uint32_t length)
{
    uint32_t sentCount = 0;
    for (uint32_t i = 0; i < length; i++) {
        if (DL_UART_transmitDataBlocking(uart, data[i])) {
            sentCount++;
        } else {
            break;
        }
    }
    return sentCount;
}
static  UART_Regs g_uart_handle;            /* ATK-MS901M UART */
//该结构体为HAL库中UART_HandleTypeDef结构体，用于配置UART的参数,我需将他更改为Driver中的结构体
static struct
{
    uint8_t buf[ATK_MS901M_UART_RX_FIFO_BUF_SIZE];  /* 缓冲 */
    uint16_t size;                                  /* 缓冲大小 */
    uint16_t reader;                                /* 读指针 */
    uint16_t writer;                                /* 写指针 */
} g_uart_rx_fifo;                                   /* UART接收FIFO */


/**
 * @brief       ATK-MS901M UART初始化
 * @param       baudrate: 波特率
 * @retval      无
 */
void init_uart_rx_fifo(void)
{
    g_uart_rx_fifo.size =ATK_MS901M_UART_RX_FIFO_BUF_SIZE;
    g_uart_rx_fifo.reader = 0;
    g_uart_rx_fifo.writer = 0;
}


/**
 * @brief       ATK-MS901M UART接收FIFO写入数据
 * @param       dat: 待写入数据
 *              len: 待写入数据的长度
 * @retval      0: 函数执行成功
 *              1: FIFO剩余空间不足
 */
uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len)
{
    uint16_t i;
    
    /* 将数据写入FIFO
     * 并更新FIFO写入指针
     */
    for (i=0; i<len; i++)
    {
        g_uart_rx_fifo.buf[g_uart_rx_fifo.writer] = dat[i];
        g_uart_rx_fifo.writer = (g_uart_rx_fifo.writer + 1) % g_uart_rx_fifo.size;
    }
    
    return 0;
}

/**
 * @brief       ATK-MS901M UART接收FIFO读取数据
 * @param       dat: 读取数据存放位置
 *              len: 欲读取数据的长度
 * @retval      0: FIFO中无数据
 *              其他值: 实际读取的数据长度
 */
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len)
{
    uint16_t fifo_usage;
    uint16_t i;
    
    /* 获取FIFO已使用大小 */
    if (g_uart_rx_fifo.writer >= g_uart_rx_fifo.reader)
    {
        fifo_usage = g_uart_rx_fifo.writer - g_uart_rx_fifo.reader;
    }
    else
    {
        fifo_usage = g_uart_rx_fifo.size - g_uart_rx_fifo.reader + g_uart_rx_fifo.writer;
    }
    
    /* FIFO数据量不足 */
    if (len > fifo_usage)
    {
        len = fifo_usage;
    }
    
    /* 从FIFO读取数据
     * 并更新FIFO读取指针
     */
    for (i=0; i<len; i++)
    {
        dat[i] = g_uart_rx_fifo.buf[g_uart_rx_fifo.reader];
        g_uart_rx_fifo.reader = (g_uart_rx_fifo.reader + 1) % g_uart_rx_fifo.size;
    }
    
    return len;
}

/**
 * @brief       ATK-MS901M UART接收FIFO清空
 * @param       无
 * @retval      无
 */
void atk_ms901m_rx_fifo_flush(void)
{
    g_uart_rx_fifo.writer = g_uart_rx_fifo.reader;
}

/**
 * @brief       ATK-MS901M UART发送数据
 * @param       dat: 待发送的数据
 *              len: 待发送数据的长度
 * @retval      无
 */
void atk_ms901m_uart_send(uint8_t *dat, uint8_t len)
{
     DL_UART_transmitDataArray(&g_uart_handle, dat, len);
}

/**
 * @brief       ATK-MS901M UART中断回调函数
 * @param       无
 * @retval      无
 */
void ATK_MS901M_UART_IRQHandler(void)
{
    uint8_t tmp;
    //这里我删除了原函数中的接收过载判断，因为我找不到对应的函数，我会试图启动dma和扩大缓冲区来解决这个问题。
    if(DL_UART_getPendingInterrupt(&g_uart_handle)==DL_UART_IIDX_RX)
    {
        DL_UART_receiveDataCheck(&g_uart_handle);
        atk_ms901m_uart_rx_fifo_write(&tmp, 1);                         /* 接收到的数据，写入UART接收FIFO */
    }
}

