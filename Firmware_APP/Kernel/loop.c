/******
	************************************************************************
	******
	** @project : XDrive_Step
	** @brief   : Stepper motor with multi-function interface and closed loop function. 
	** @brief   : 具有多功能接口和闭环功能的步进电机
	** @author  : unlir (知不知啊)
	** @contacts: QQ.1354077136
	******
	** @address : https://github.com/unlir/XDrive
	******
	************************************************************************
	******
	** {Stepper motor with multi-function interface and closed loop function.}
	** Copyright (c) {2020}  {unlir(知不知啊)}
	** 
	** This program is free software: you can redistribute it and/or modify
	** it under the terms of the GNU General Public License as published by
	** the Free Software Foundation, either version 3 of the License, or
	** (at your option) any later version.
	** 
	** This program is distributed in the hope that it will be useful,
	** but WITHOUT ANY WARRANTY; without even the implied warranty of
	** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	** GNU General Public License for more details.
	** 
	** You should have received a copy of the GNU General Public License
	** along with this program.  If not, see <http://www.gnu.org/licenses/>.
	******
	************************************************************************
******/

/*****
  ** @file     : loop.c/h and loop_it.c
  ** @brief    : MCU系工程主框架
  ** @versions : newest
  ** @time     : newest
  ** @reviser  : unli (WuHu China)
  ** @explain  : null
*****/

//Oneself
#include "loop.h"

//Application_User_Core
#include "dma.h"
#include "adc.h"
#include "usart.h"

//Base_Drivers
#include "kernel_port.h"
#include "mt6816.h"
#include "hw_elec.h"

//Control
#include "signal_port.h"
#include "motor_control.h"
#include "encode_cali.h"

//Debug
#include "button.h"
#include "ssd1306.h"
#include "xdrive_ui.h"

//Save
#include "stockpile_f103cb.h"

/**********************************************************************/
/*********************    Subclock processing    **********************/
/**********************************************************************/
uint32_t time_1ms_count = 0;
uint32_t major_cycle_count = 0;
static uint32_t time_second_1ms = 0;
static uint32_t time_second_10ms = 0;
static uint32_t time_second_20ms = 0;
static uint32_t time_second_50ms = 0;
static uint32_t time_second_100ms = 0;


uint16_t reboot;
uint16_t save;
uint32_t version = 0x01010602;

void load_info(void)
{
	if(*((uint32_t *)stockpile_data.begin_add) == version)
	{
		// pid
		pid.valid_kp = true;
		pid.kp = *((int32_t *)(stockpile_data.begin_add+4));
		pid.valid_ki = true;
		pid.ki = *((int32_t *)(stockpile_data.begin_add+8));
		pid.valid_kd = true;
		pid.kd = *((int32_t *)(stockpile_data.begin_add+12));
		
		// dce
		dce.valid_kp = true;
		dce.kp = *((int32_t *)(stockpile_data.begin_add+16));
		dce.valid_ki = true;
		dce.ki = *((int32_t *)(stockpile_data.begin_add+20));
		dce.valid_kv = true;
		dce.kv = *((int32_t *)(stockpile_data.begin_add+24));
		dce.valid_kd = true;
		dce.kd = *((int32_t *)(stockpile_data.begin_add+28));
		
		// Uart
		dyn_uart1.valid_uart_baudrate = true;
		dyn_uart1.baud_rate_order = *((int32_t *)(stockpile_data.begin_add+32));
		dyn_uart1.valid_uart_mode = true;
		dyn_uart1.uart_order = (Uart_Mode)(*((int16_t *)(stockpile_data.begin_add+36)));
		
		// Modbus
		signal_modbus.valid_modbus_id = true;
		signal_modbus.id_order = *((int16_t *)(stockpile_data.begin_add+38));
	}
}

void save_info(void)
{
	Stockpile_Flash_Data_Empty(&stockpile_data);
	Stockpile_Flash_Data_Begin(&stockpile_data);
	
	Stockpile_Flash_Data_Set_Write_Add(&stockpile_data, stockpile_data.begin_add);
	
	// head
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&version, 1);
	
	// PID
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&pid.kp, 1);
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&pid.ki, 1);
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&pid.kd, 1);
	
	// DCE
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&dce.kp, 1);
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&dce.ki, 1);
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&dce.kv, 1);
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&dce.kd, 1);
	
	// Uart
	Stockpile_Flash_Data_Write_Data32(&stockpile_data, (uint32_t *)&dyn_uart1.baud_rate_order, 1);
	Stockpile_Flash_Data_Write_Data16(&stockpile_data, (uint16_t *)&dyn_uart1.uart_order, 1);
	
	// Modbus
	Stockpile_Flash_Data_Write_Data16(&stockpile_data, (uint16_t *)&signal_modbus.id_order, 1);
	
	Stockpile_Flash_Data_End(&stockpile_data);
	
	save = 0;
}


/**
* @brief 副时钟10ms执行
*/
void time_second_10ms_serve(void)
{
}

/**
* @brief 副时钟20ms执行
*/
void time_second_20ms_serve(void)
{
	//XDrive_REINui(50Hz刷新率)
	XDrive_REINui_Callback_ms(20);
}

/**
* @brief 副时钟50ms执行
*/
void time_second_50ms_serve(void)
{

}

/**
* @brief 副时钟100ms执行
*/
void time_second_100ms_serve(void)
{
	if(reboot) HAL_NVIC_SystemReset();
	if(save) save_info();
}
	
/**
* @brief 副时钟循环执行
**/
void time_second_run(void)
{
	if(time_second_10ms)		{time_second_10ms--;		time_second_10ms_serve();		}
	if(time_second_20ms)		{time_second_20ms--;		time_second_20ms_serve();		}
	if(time_second_50ms)		{time_second_50ms--;		time_second_50ms_serve();		}
	if(time_second_100ms)		{time_second_100ms--;		time_second_100ms_serve();	}
}

/**
* @brief 副时钟1ms时钟
**/
void loop_second_base_1ms(void)
{
	time_1ms_count++;
	time_second_1ms++;
	if(!(time_second_1ms % 10))		{time_second_10ms++;		}
	if(!(time_second_1ms % 20))		{time_second_20ms++;		}
	if(!(time_second_1ms % 50))		{time_second_50ms++;		}
	if(!(time_second_1ms % 100))	{time_second_100ms++;		}
	if(!(time_second_1ms % 1000))	{time_second_1ms = 0;		}
}

/*********************************************************************/
/****************************    LOOP    *****************************/
/*********************************************************************/
/**
* This is Main LOOP
*/
void loop(void)
{
	// 加载配置
	load_info();
	
	//调试工具(Debug)
	Button_Init();				//按键初始化
	SSD1306_Init();				//OLED初始化
	XDrive_REINui_Init();	//UI初始化
	
	//进行关键外设初始化前等待电源稳定
	HAL_Delay(1000);
	
//	//存储数据恢复
//	Slave_Reg_Init();			//校验Flash数据并配置参数
	
	//基本外设初始化(Base_Drivers)(LOOP直接进行)
	REIN_DMA_Init();
	REIN_ADC_Init();	
	
	//基本外设初始化()
	REIN_MT6816_Init();		//MT6816传感器初始化
	REIN_HW_Elec_Init();	//硬件电流控制器
	Signal_MoreIO_Init();	//MoreIO接口初始化
	
	//控制初始化(Control)
	Calibration_Init();		//校准器初始化
	Motor_Control_Init();	//电机控制初始化
	
	//通讯接口初始化
	REIN_GPIO_Modbus_Init();	//RS485_DIR
	REIN_UART_Modbus_Init();	//串口初始化
	Signal_Modbus_Init();			//Modbu接口初始化
	
	//调整中断配置
	LoopIT_Priority_Overlay();	//重写-中断优先级覆盖
	LoopIT_SysTick_20KHz();			//重写-系统计时器修改为20KHz
	
	//启动时使用按键触发校准
	if(HAL_GPIO_ReadPin(BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin) == GPIO_PIN_RESET)
	{
		encode_cali.trigger = true;			//触发校准
		XDrive_REINui_ToCalibration();	//进入UI校准界面
	}
	
	//FOR Circulation
	for(;;)
	{
		major_cycle_count++;
		time_second_run();

		Calibration_Loop_Callback();							//校准器主程序回调					用于校准环节最后的数据解算
	}
}
/*************************************
*********** END Dispatch  ************
*************************************/

