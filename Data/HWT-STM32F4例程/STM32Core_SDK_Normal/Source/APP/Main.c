/*
@ link : http://wit-motion.cn

@ Function:
1. Power on automatic detection sensor
2. Read acceleration, angular velocity, angle and magnetic field data
3. Set switching baud rate parameters

USB-TTL                   STM32Core              		JY901s
VCC          -----           VCC        ----        	 VCC
TX           -----           RX1  (GPIOA_10)   
RX           -----           TX1  (GPIOA_9)
GND          -----           GND    ----       			 GND
                             RX2  (GPIOA_3)  ----        TX
							 TX2  (GPIOA_2)  ----        RX
------------------------------------
*/

#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "wit_c_sdk.h"
#include "UART1.h"
#include "UART2.h"
#include "delay.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

int main(void)//程序启动函数
{
	float fAcc[3], fGyro[3], fAngle[3]; //定义float数组为下面计算输出做准备
	int i;
	
	SysTick_Init();//系统时钟初始化
	Usart1Init(115200);//串口1初始化接PC端
	Usart2Init(9600);//串口2初始化接传感器端
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);//初始化标准协议，设置设备地址
	WitSerialWriteRegister(SensorUartSend);//注册写回调函数    串口1接收数据调用 SensorUartSend函数
	WitRegisterCallBack(SensorDataUpdata);//注册获取传感器数据回调函数   串口2接收数据调用SensorDataUpdata函数
	WitDelayMsRegister(Delayms);//注册延时回调函数
	printf("\r\n********************** wit-motion normal example  ************************\r\n");//打印输出
	AutoScanSensor();//自动搜索传感器，如果线没有插对或者使用的串口工具不对会搜索不到传感器
	while (1)//循环语句
	{
		CmdProcess();//如果发送数据修改失败会发送错误提示会一直检测，成功的话不会有提示，如果需要成功有提示也是可以进行修改的
		if(s_cDataUpdate)//在注册获取传感器数据回调函数会对变量进行赋值
		{
			for(i = 0; i < 3; i++)//for循环待i不符合条件时跳出for循环
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;//算法公式
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;//算法公式
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;//算法公式
			}
			if(s_cDataUpdate & ACC_UPDATE)//判断不为0就执行下面的语句
			{
				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);//打印对应的数据出来
				s_cDataUpdate &= ~ACC_UPDATE;//s_cDataUpdate和~ACC_UPDATE与运算后赋值给s_cDataUpdate
			}
			if(s_cDataUpdate & GYRO_UPDATE)//以下几行代码同上
			{
				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
				s_cDataUpdate &= ~GYRO_UPDATE;
			}
			if(s_cDataUpdate & ANGLE_UPDATE)
			{
				printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}
			if(s_cDataUpdate & MAG_UPDATE)
			{
				printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
				s_cDataUpdate &= ~MAG_UPDATE;
			}
		}
	}
}

void CopeCmdData(unsigned char ucData)//UART1(串口1)接收数据并代入该函数进行处理
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;//代入进来的数据遍历到s_ucData数组中
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)//判断接收到的数据有多少个，一般PC端上发送一个‘a’加上 '\r'和'\n'一共3个
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))//正常接收回来的数据和判断括号里面的一样
		{
			s_cCmd = s_ucData[0];//把接收到的第一个数据赋值给s_cCmd
			memset(s_ucData,0,50);//底层函数，初始化数组全部为0
			s_ucRxCnt = 0;
		}
		else //如果if()括号里面的内容不成立就会执行下面的操作
		{
			s_ucData[0] = s_ucData[1];//数组赋值前移
			s_ucData[1] = s_ucData[2];//数组赋值前移
			s_ucRxCnt = 2;
			
		}
	}

}
static void ShowHelp(void)//上电提示并且打印下面内容
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(void)//UART1接收到的数据会代入到这里进行检测
{
	switch(s_cCmd)//判断输入的是什么进而来判断进行什么操作
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) //加速度校准 判断WitStartAccCali的结果是否等于WIT_HAL_OK如果不等于就执行下面的内容，等于就跳出
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) //磁场校准
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK) //
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) //带宽缩减
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) //带宽增加 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) //波特率增加到115200
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart2Init(c_uiBaud[WIT_BAUD_115200]);											
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) //波特率减少到9600
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart2Init(c_uiBaud[WIT_BAUD_9600]);												
			break;
		case 'R':	
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) //返回率增加到10HZ
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'r':	
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) //返回率减少到1HZ
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'C':	
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) //基本返回内容：加速度、角速度、角度、磁场
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'c':
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK) //返回内容：加速度
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'h':
			ShowHelp();//帮助
			break;
	}
	s_cCmd = 0xff;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)//传感器串口发送
{
	Uart2Send(p_data, uiSize);//串口2发送
}

static void Delayms(uint16_t ucMs)//延时程序
{
	delay_ms(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)//传感器数据升级
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)//判断uiReg的数据是什么来进行选择对应的操作
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;//s_cDataUpdate变量和ACC_UPDATE变量或运算后的结果再把结果赋值给s_cDataUpdate，如s_cDataUpdate=s_cDataUpdate|和ACC_UPDATE
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)//串口波特率检测
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)//for循环待i不符合条件会跳出循环
	{
		Usart2Init(c_uiBaud[i]);//串口2波特率从小到大查询
		iRetry = 2;
		do//do-while()循环语句先执行在判断
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);//给函数进行导入形参
			delay_ms(100);//延时100ms
			if(s_cDataUpdate != 0)//判断s_cDataUpdate不等于0如果是不等于0就执行括号里面的内容
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);//打印找到传感器和打印对应的波特率
				ShowHelp();//执行函数打印内容
				return ;
			}
			iRetry--;//变量自减
		}while(iRetry);//while不为0就一直执行循环的内容
	}
	printf("can not find sensor\r\n");//如果上面没有找到传感器就会执行下面这两句，如果找到就不会执行这两句
	printf("please check your connection\r\n");//
}

