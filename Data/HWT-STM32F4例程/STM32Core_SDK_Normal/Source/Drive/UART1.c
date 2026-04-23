#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "UART1.h"

void Usart1Init(unsigned int uiBaud)//UART1(串口1)初始化，不同单片机不同
{
 	GPIO_InitTypeDef GPIO_InitStructure;//GPIO命名
	USART_InitTypeDef USART_InitStructure;//USART串口命名
	NVIC_InitTypeDef NVIC_InitStructure;//中断NVIC命名
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);//时钟配置USART1,GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//时钟配置USART1,GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;//GPIO配置pin9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
*/
/*
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//中断配置USART1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
*/
	USART_InitStructure.USART_BaudRate = uiBaud;//USART配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE); 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_Cmd(USART1, ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

int fputc(int ch, FILE *file)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, ch);
	return ch;
}

void CopeCmdData(unsigned char ucData);
void USART1_IRQHandler(void)//USART1(串口1)中断
{
	unsigned char ucTemp;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//判断获取IT状态是否为0
	{
		ucTemp = USART_ReceiveData(USART1);//串口1接收数据并赋值给ucTemp
//		printf("ucTemp%c\n",ucTemp);
		CopeCmdData(ucTemp);//把接收到的数据代入CopeCmdData函数中
//		printf("ucTemp1%c\n",ucTemp);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);//把UART1串口1中的数据清零
//printf("ucTemp2%c\n",ucTemp);
	}
	USART_ClearITPendingBit(USART2,USART_IT_ORE);//清零
//printf("ucTemp3%c\n",ucTemp);
}

