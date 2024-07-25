#include "imu901_usart.h"
#include "imu901.h"

ringbuffer_t uart3RxFifo;
#define UART3_RX_BUFFER_SIZE	256
uint8_t uart3RxBuffer[UART3_RX_BUFFER_SIZE];


void imu901_usart_init(u32 bound)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

   
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART3, &USART_InitStructure); 
    ringbuffer_init(&uart3RxFifo, uart3RxBuffer, UART3_RX_BUFFER_SIZE);
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
#if EN_USART3_RX	

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
//	USART_ITConfig(USART3, USART_IT_IDLE, DISABLE);//开启相关中断
	
	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
#endif

USART_Cmd(USART3, ENABLE);  

}

void USART3_IRQHandler(void)
{
	  uint8_t res ;
	  
    if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) != RESET)
		  {
		   
				res = USART3-> DR ;
				ringbuffer_in_check(&uart3RxFifo,(uint8_t *)&res,1);
				
		  }
	

}

