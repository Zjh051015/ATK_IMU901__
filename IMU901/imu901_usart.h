#ifndef _IMU901_USART_H_
#define _IMU901_USART_H_

#include "imu901.h"
#include "ringbuffer.h"

extern ringbuffer_t uart3RxFifo;

void imu901_usart_init(u32 bound) ;

#endif
