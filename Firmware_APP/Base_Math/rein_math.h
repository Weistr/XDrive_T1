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
  ** @file     : rein_math.c/h
  ** @brief    : REIN数学函数库
  ** @versions : 1.1.16
  ** @time     : 2020/05/22
  ** @reviser  : unli (HeFei China)
  ** @explain  : null
*****/

#ifndef REIN_MATH_H
#define REIN_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>
int32_t i32_abs(int32_t in);
int8_t i32_polAdj(int32_t A,int32_t B);//AB同号输出1，异号输出0

#ifdef __cplusplus
}
#endif
	 
#endif
