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

#endif
