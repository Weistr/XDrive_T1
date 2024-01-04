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


//ħ��
extern int16_t minDiv;//�ڴ˷�Χ�ڽ����Ӽ�������

extern void trackTriger(void);
void xdUartRxIsr(char* rcvBuf,uint16_t len)
{	
	
	if((motorId == rcvBuf[0]) || (rcvBuf[0] == 0x00))//У��ID,0x00ͨ��ID
	{
		//ID��ȷ
		crcCal = crc16Cal((uint8_t*)&rcvBuf[0],len-2,1);//У��CRC
		crcRcv = (rcvBuf[len-2]<<8) + rcvBuf[len-1];

		if(crcCal == crcRcv)
		{
			//CRC��ȷ
			static int32_t tmp[3];
			switch(rcvBuf[1])//ѡ��ģʽ
			{
				static uint8_t motorsync_mod=0;
				case 0x02:
					if(motorsync_mod == 1)
					{
						motor_control.goal_location = tmp[0];
						motor_control.goal_speed = tmp[1];
						Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//����ģʽ
					}
						
					else if (motorsync_mod == 2)
						motor_control.goal_speed = tmp[1];
						Motor_Control_SetMotorMode(Motor_Mode_Digital_Speed);//����ģʽ
					break;
					
					
				/**λ���ٶ�ģʽ*/
				case Motor_Mode_Digital_Location://λ���ٶ�ģʽ		��������	
					memcpy((uint8_t*)&tmp,&rcvBuf[2],8);//��������ת��		
				/*
					if (tmp[0] > posMax)tmp[0]=posMax;
					if (tmp[0] < posMin)tmp[0]=posMin;	
					if (tmp[1] > speedMax)tmp[1]=speedMax;
					if (tmp[1] < speedMin)tmp[1]=speedMin;	*/
					motor_control.goal_location = tmp[0];
					motor_control.goal_speed = tmp[1];
					userMode=0;
					mode_sendBack = SendBackPosSpeed;
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//����ģʽ	
					motorsync_mod = 1;
					break;

				case Motor_Mode_Digital_Location+0x08://λ���ٶ�ģʽ		������	
					memcpy((uint8_t*)&tmp,&rcvBuf[2],8);//��������ת��		
				/*
					if (tmp[0] > posMax)tmp[0]=posMax;
					if (tmp[0] < posMin)tmp[0]=posMin;	
					if (tmp[1] > speedMax)tmp[1]=speedMax;
					if (tmp[1] < speedMin)tmp[1]=speedMin;	
*/
					userMode=0;
					mode_sendBack = SendBackPosSpeed;
					//Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//����ģʽ	
					motorsync_mod = 1;
					break;
				
				
				
				
				/**�ٶ�ģʽ*/		
				case Motor_Mode_Digital_Speed://�ٶ�ģʽ	��������	
					BYTE0(tmp[1]) = rcvBuf[2];
					BYTE1(tmp[1]) = rcvBuf[3];
					BYTE2(tmp[1]) = rcvBuf[4];
					BYTE3(tmp[1]) = rcvBuf[5];		
					mode_sendBack = SendBackPosSpeed;		
					motor_control.goal_speed = tmp[1];
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Speed);//����ģʽ	
					motorsync_mod = 2;
					break;	
				case Motor_Mode_Digital_Speed+0x08://�ٶ�ģʽ	no����	
					BYTE0(tmp[1]) = rcvBuf[2];
					BYTE1(tmp[1]) = rcvBuf[3];
					BYTE2(tmp[1]) = rcvBuf[4];
					BYTE3(tmp[1]) = rcvBuf[5];		
					mode_sendBack = SendBackPosSpeed;		
					//Motor_Control_SetMotorMode(Motor_Mode_Digital_Speed);//����ģʽ	
					motorsync_mod = 2;
					break;	
				
				
				
				case Motor_Mode_Digital_Current://����ģʽ
					BYTE0(motor_control.goal_current) = rcvBuf[2];
					BYTE1(motor_control.goal_current) = rcvBuf[3];
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Current);//����ģʽ	
					break;
		
				case Motor_Mode_SelfTrack://�켣
					BYTE0(trackPosEnd) = rcvBuf[2];//Ŀ��λ��
					BYTE1(trackPosEnd) = rcvBuf[3];
					BYTE2(trackPosEnd) = rcvBuf[4];
					BYTE3(trackPosEnd) = rcvBuf[5];
					BYTE0(trackSpeedPss) = rcvBuf[6];//�����ٶ�
					BYTE1(trackSpeedPss) = rcvBuf[7];
					BYTE0(trackAcc) = rcvBuf[8];//���̼��ٶ�
					BYTE1(trackAcc) = rcvBuf[9];					
					BYTE0(trackAccEnd) = rcvBuf[10];
					BYTE1(trackAccEnd) = rcvBuf[11];				
					userMode=1;	
					trackTriger();			
					Motor_Control_SetMotorMode(Motor_Mode_Digital_Location);//����ģʽ	
					break;
				case Control_Mode_Stop://stop					
					Motor_Control_SetMotorMode(Control_Mode_Stop);//����ģʽ	
					break;
				case Control_Mode_SetId:
					//motorId = rcvBuf[2];
					break;
				

			}
			if(rcvBuf[0] == 0x00)mode_sendBack=0x00;//ͨ��id���ش�
			delay_us(20);

			switch(mode_sendBack)//��������
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
