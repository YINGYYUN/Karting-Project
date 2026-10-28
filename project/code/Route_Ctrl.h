/*******************************************************************************
路表页：录制开关 + 遥控接管（回放待 route_pb 迁入后加）
*******************************************************************************/


#ifndef _ROUTE_CTRL_H_
#define _ROUTE_CTRL_H_


#include "zf_common_headfile.h"


// 上电一次：清缓冲、读槽索引、载入 active 槽（阻塞，必须在 PIT 打开前调）
void Route_Ctrl_Init (void);

// 页面：返回 0 = 正常退出
int  Route_Ctrl_Page (void);


#endif
