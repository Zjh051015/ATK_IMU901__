#ifndef __ATK_IMU901_H
#define __ATK_IMU901_H
#include "stdint.h"
/*
    @file atk_imu901.h
    @brief IMU901 "Serial Port Interface"
    @version 1.0
    @date 2024-07-24
    @author 
    该部分提供ATK_IMU901的串口接口定义以及函数功能
*/
    /* UART发送引脚 */
    #define ATK_MS901M_UART_TX_GPIO_PORT            GPIOA
    #define ATK_MS901M_UART_TX_GPIO_PIN             GPIO_PIN_9

    /* UART接收引脚 */
    #define ATK_MS901M_UART_RX_GPIO_PORT            PORTA
    #define ATK_MS901M_UART_RX_GPIO_PIN             GPIO_PIN_10

    /* UART接口 */
    #define ATK_MS901M_UART_INTERFACE               USART1
    #define ATK_MS901M_UART_IRQn                    USART1_IRQn
    #define ATK_MS901M_UART_IRQHandler              USART1_IRQHandler

    /* UART接收FIFO缓冲区大小 */
    #define ATK_MS901M_UART_RX_FIFO_BUF_SIZE        128

    /*操作函数*/
    uint8_t atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);  /* ATK-MS901M UART接收FIFO写入数据 */
    uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);  /* ATK-MS901M UART接收FIFO读取数据 */
    void atk_ms901m_rx_fifo_flush(void);                                /* ATK-MS901M UART接收FIFO清空 */
    void atk_ms901m_uart_send(uint8_t *dat, uint8_t len);               /* ATK-MS901M UART发送数据 */
    void atk_ms901m_uart_init(uint32_t baudrate);                       /* ATK-MS901M UART初始化 */
#endif