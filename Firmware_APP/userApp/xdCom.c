#include "xdCom.h"
#include "crc16modbus.h"
#include "motor_control.h"
#include "uart_mixed.h"
#include "bsp_delay.h"



uint8_t mode_sendBack = SendBackPosSpeed;
uint8_t userMode=0;

xdriveComTypddef xdriveCtl_Send;

extern int16_t trackSpeedPss;
extern int32_t trackPosEnd;
extern int16_t trackAcc;
extern int16_t trackAccEnd;
extern int16_t softSpeed;

uint16_t crcCal;
uint16_t crcRcv;


//魔方
extern int16_t minDiv;//在此范围内结束加减速运算

extern void trackTriger(void);
void xdUartRxIsr(char* rcvBuf,uint16_t len)
{	
	
	if((motorId == rcvBuf[0]) || (rcvBuf[0] == 0x00))//校验ID,0x00通用ID
	{
		//ID正确
		crcCal = crc16Cal((uint8_t*)&rcvBuf[0],len-2,1);//校验CRC
		crcRcv = (rcvBuf[len-2]<<8) + rcvBuf[len-1];

		if(crcCal == crcRcv)
		{
			//CRC正确
			static int32_t tmp[3];
			switch(rcvBuf[1])//选择模式
			{
				static uint8_t motorsync_mod=0;
				case 0x02:
					if(motorsync_mod == 1)
					{
						motor_control.goal_location = tmp[0];
						motor_control.goal_speed = tmp[1];
						Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//更新模式
					}
						
					else if (motorsync_mod == 2)
						motor_control.goal_speed = tmp[1];
						Motor_Control_SetMotorMode(Motor_Mode_Digital_Speed);//更新模式
					break;
					
					
				/**位置速度模式*/
				case Motor_Mode_Digital_Location://位置速度模式		立即更新	
					memcpy((uint8_t*)&tmp,&rcvBuf[2],8);//接受数据转入		
				/*
					if (tmp[0] > posMax)tmp[0]=posMax;
					if (tmp[0] < posMin)tmp[0]=posMin;	
					if (tmp[1] > speedMax)tmp[1]=speedMax;
					if (tmp[1] < speedMin)tmp[1]=speedMin;	*/
					motor_control.goal_location = tmp[0];
					motor_control.goal_speed = tmp[1];
					userMode=0;
					mode_sendBack = SendBackPosSpeed;
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//更新模式	
					motorsync_mod = 1;
					break;

				case Motor_Mode_Digital_Location+0x08://位置速度模式		不更新	
					memcpy((uint8_t*)&tmp,&rcvBuf[2],8);//接受数据转入		
				/*
					if (tmp[0] > posMax)tmp[0]=posMax;
					if (tmp[0] < posMin)tmp[0]=posMin;	
					if (tmp[1] > speedMax)tmp[1]=speedMax;
					if (tmp[1] < speedMin)tmp[1]=speedMin;	
*/
					userMode=0;
					mode_sendBack = SendBackPosSpeed;
					//Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//更新模式	
					motorsync_mod = 1;
					break;
				
				
				
				
				/**速度模式*/		
				case Motor_Mode_Digital_Speed://速度模式	立即更新	
					BYTE0(tmp[1]) = rcvBuf[2];
					BYTE1(tmp[1]) = rcvBuf[3];
					BYTE2(tmp[1]) = rcvBuf[4];
					BYTE3(tmp[1]) = rcvBuf[5];		
					mode_sendBack = SendBackPosSpeed;		
					motor_control.goal_speed = tmp[1];
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Speed);//更新模式	
					motorsync_mod = 2;
					break;	
				case Motor_Mode_Digital_Speed+0x08://速度模式	no更新	
					BYTE0(tmp[1]) = rcvBuf[2];
					BYTE1(tmp[1]) = rcvBuf[3];
					BYTE2(tmp[1]) = rcvBuf[4];
					BYTE3(tmp[1]) = rcvBuf[5];		
					mode_sendBack = SendBackPosSpeed;		
					//Motor_Control_SetMotorMode(Motor_Mode_Digital_Speed);//更新模式	
					motorsync_mod = 2;
					break;	
				
				
				
				case Motor_Mode_Digital_Current://电流模式
					BYTE0(motor_control.goal_current) = rcvBuf[2];
					BYTE1(motor_control.goal_current) = rcvBuf[3];
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Current);//更新模式	
					break;
		
				case Motor_Mode_SelfTrack://轨迹
					BYTE0(trackPosEnd) = rcvBuf[2];//目标位置
					BYTE1(trackPosEnd) = rcvBuf[3];
					BYTE2(trackPosEnd) = rcvBuf[4];
					BYTE3(trackPosEnd) = rcvBuf[5];
					BYTE0(trackSpeedPss) = rcvBuf[6];//过程速度
					BYTE1(trackSpeedPss) = rcvBuf[7];
					BYTE0(trackAcc) = rcvBuf[8];//过程加速度
					BYTE1(trackAcc) = rcvBuf[9];					
					BYTE0(trackAccEnd) = rcvBuf[10];
					BYTE1(trackAccEnd) = rcvBuf[11];				
					userMode=1;	
					trackTriger();			
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//更新模式	
					break;
				case Control_Mode_Stop://stop					
					Motor_Control_SetMotorMode(Control_Mode_Stop);//更新模式	
					break;
				case Control_Mode_SetId:
					//motorId = rcvBuf[2];
					break;
				

			}
			if(rcvBuf[0] == 0x00)mode_sendBack=0x00;//通用id不回传
			delay_us(20);

			switch(mode_sendBack)//返回数据
			{
				case SendBackPosSpeed:
					
					xdriveCtl_Send.mode = rcvBuf[1];
					xdriveCtl_Send.id = motorId;
					xdriveCtl_Send.pos = motor_control.est_location;
					xdriveCtl_Send.speed = motor_control.est_speed;
					xdriveCtl_Send.crc = crc16Cal((uint8_t*)&xdriveCtl_Send,10,0);
					UART_Mixed_TxTrigger(&muart1,(char*)&xdriveCtl_Send,12);		
					break;
			}
		}
	}
		
	
}

void xdUartTxIsr(void)
{
	
}
