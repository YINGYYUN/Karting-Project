/*******************************************************************************
PID
*******************************************************************************/


#ifndef __PID_H__
#define __PID_H__


#include "zf_common_headfile.h"


/**********************************************************/
/*[S] 基础函数 [S]----------------------------------------*/
/**********************************************************/

// 位置式PID结构体
typedef struct {
	float Target;			// 目标
	float Actual;			// 这次实际
	// float Actual1;		// 上次实际（微分先行使用）
	float Out;				// 输出
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;			// 此次误差
	float Error1;			// 上次误差
	float ErrorInt; 		// 误差累积（积分）
	
	float ErrorIntMax;		// 积分最大值
	float ErrorIntMin;		// 积分最小值
	
	float OutMax;			// 输出最大值
	float OutMin;			// 输出最小值
	
	float IntSepThresh;  	// 积分分离阈值（误差大于阈值 积分清零）
	
} PID_POS_t;

// 增量式PID结构体
typedef struct {
	float Target;			// 目标
	float Actual;			// 这次实际
	float Out;				// 输出
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;			// 此次误差
	float Error1;			// 上次误差
	float Error2;			// 上上次误差
	float ErrorInt; 		// 误差累积（积分）
	
	float OutMax;			// 输出最大值
	float OutMin;			// 输出最小值
	
	float OutDeltaMax;		// 单次增量变化上限（=0 时不限制，>0 时每次 Out 变化不超过此值）
} PID_INC_t;


// 位置式PID重置中间量
void	PID_POS_Init		(PID_POS_t *p);
// 位置式PID计算
void 	PID_POS_Update		(PID_POS_t *p);
// 增量式PID重置中间量
void	PID_INC_Init		(PID_INC_t *p);
// 增量式PID计算
void 	PID_INC_Update		(PID_INC_t *p);

/**********************************************************/
/*----------------------------------------[E] 基础函数 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] PID实例 [S]-----------------------------------------*/
/**********************************************************/

// 位置式PID实例


// 增量式PID实例
extern PID_INC_t Motor_1_PID;	// 电机接口 1
extern PID_INC_t Motor_2_PID;	// 电机接口 2
extern PID_INC_t Motor_3_PID;	// 电机接口 3
extern PID_INC_t Motor_4_PID;	// 电机接口 4

// 重置PID的所有中间量
void PID_ALL_Init(void);
/**********************************************************/
/*-----------------------------------------[E] PID实例 [E]*/
/**********************************************************/

// 电机速度环调用包装
// 电机速度环 PID 使能
extern uint8_t Speed_PID_Crtl_Enable;
// 电机速度环 PID
void Speed_PID_Crtl (void);


#endif
