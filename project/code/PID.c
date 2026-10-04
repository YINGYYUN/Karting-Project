/*******************************************************************************
PID
*******************************************************************************/


#include "zf_common_headfile.h"
#include <math.h>


/**********************************************************/
/*[S] 基础函数 [S]----------------------------------------*/
/**********************************************************/

//--------------------------------------
// 函数简介     位置式PID计算中间量重置
// 参数说明     PID_POS_t *p	要传入的PID结构体变量名称
// 使用示例     PID_POS_Init(&Angle_PID);
// 备注信息     注意需要考虑多个调用位置
//--------------------------------------
void PID_POS_Init(PID_POS_t *p)
{
	p->Target   = 0;
	p->Actual   = 0;
	// p->Actual1  = 0;（微分先行使用）
	p->Out      = 0;
	p->Error0   = 0;
	p->Error1   = 0;
	p->ErrorInt = 0;	
}

//--------------------------------------
// 函数简介     位置式PID计算
// 参数说明     PID_POS_t *p	要传入的PID结构体变量名称
// 使用示例     PID_POS_Update(&Angle_PID);
// 备注信息     包含PID优化逻辑，需要注意赋予的初值
//--------------------------------------
void PID_POS_Update(PID_POS_t *p)
{
	/* 获取误差 */
	p->Error1 = p->Error0;					// Error0 → Error1（保存上次误差）
	p->Error0 = p->Target - p->Actual;		// 目标值 - 实际值 = 此次误差
	
	float C = 1.0f;
	/* 积分分离 */
	if ( fabs(p->Error0) > p->IntSepThresh )
	{
		C = 0.0f;
	}
	
	/* 外环误差积分（累加）*/
	/* 如果Ki不为0，才进行误差积分，这样做的目的是便于调试 */
	if (p->Ki != 0)					// 如果Ki不为0
	{
		p->ErrorInt += p->Error0 * C;	// 进行误差积分
		// 积分限幅
		if (p->ErrorInt > p->ErrorIntMax) p->ErrorInt = p->ErrorIntMax;
		if (p->ErrorInt < p->ErrorIntMin) p->ErrorInt = p->ErrorIntMin;	
	}
	else							// Ki为0
	{
		p->ErrorInt = 0;			// 误差积分归0
	}
	
	/* 位置式PID计算 */
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);     // 标准微分（不使用微分先行）
	//	   - p->Kd * (p->Actual - p->Actual1);    // 微分先行（微分先行使用）
	
	/* 输出限幅 */
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}	// 限制输出值最大为结构体指定的OutMax
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}	// 限制输出值最小为结构体指定的OutMin
	
	// p->Actual1 = p->Actual;// （微分先行使用）
}

//--------------------------------------
// 函数简介     增量式PID计算中间量重置
// 参数说明     PID_INC_t *p	要传入的PID结构体变量名称
// 使用示例     PID_INC_Init(&Motor1_PID);
// 备注信息     注意需要考虑多个调用位置
//--------------------------------------
void PID_INC_Init(PID_INC_t *p)
{
	p->Target   = 0;
	p->Actual   = 0;
	// p->Actual1  = 0;（微分先行使用）
	p->Out      = 0;
	p->Error0   = 0;
	p->Error1   = 0;
	p->Error2   = 0;
	p->ErrorInt = 0;	
}

//--------------------------------------
// 函数简介     增量式PID计算
// 参数说明     PID_INC_t *p	要传入的PID结构体变量名称
// 使用示例     PID_INC_Update(&Motor1_PID);
// 备注信息     包含PID优化逻辑，需要注意赋予的初值
// 大概率只给电机使用，所以存在(int16)的强制转化
//--------------------------------------
void PID_INC_Update(PID_INC_t *p)
{
	/* 获取误差 */
	p->Error2 = p->Error1;					// Error1 → Error2（保存上上次误差）
	p->Error1 = p->Error0;					// Error0 → Error1（保存上次误差）
	p->Error0 = p->Target - p->Actual;		// 目标值 - 实际值 = 此次误差

	/* 增量式PID计算 */
	float delta = p->Kp * (p->Error0 - p->Error1)
				+ p->Ki * p->Error0
				+ p->Kd * (p->Error0 - 2*p->Error1 + p->Error2);

	/* 单次增量限幅（OutDeltaMax > 0 时生效）*/
	if (p->OutDeltaMax > 0)
	{
		if (delta >  p->OutDeltaMax) delta =  p->OutDeltaMax;
		if (delta < -p->OutDeltaMax) delta = -p->OutDeltaMax;
	}

	/* 累加至Out */
	p->Out += delta;

	/* 输出限幅 */
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}	// 限制输出值最大为结构体指定的OutMax
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}	// 限制输出值最小为结构体指定的OutMin
}
/**********************************************************/
/*----------------------------------------[E] 基础函数 [E]*/
/**********************************************************/
