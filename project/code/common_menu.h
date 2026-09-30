/*******************************************************************************
链式通用菜单框架
参考:【手把手教菜单_框架搭建】 https://www.bilibili.com/video/BV1LCh9zrEMp/?share_source=copy_web&vd_source=befff269c2e0ef1583e2528659770521
*******************************************************************************/


#ifndef __COMMON_MENU_H__
#define __COMMON_MENU_H__


#include "zf_common_headfile.h"

typedef enum MENU_KIND {
    MENU_Folder = 0,
    int32_Box,
    uint32_Box,
    int16_Box,
    uint16_Box,
    int8_Box,
    uint8_Box,
    float_Box,
    bool_Box,
}MENU_KIND;

typedef struct Menu_Item {
    const char *name;
    void *data;                         // 指向存放的变量
    MENU_KIND kind;                     // 记录节点属性

    uint8_t sons;                       // 记录子节点数量
    uint8_t No;                         // 记录当前是父节点的第几个成员
    bool select;                        // 是否被选中的标志位(参数修改的功能)

    bool isLimit;                       // 是否启用限幅
    float limit_min;                    // 最小值限幅
    float limit_max;                    // 最大值限幅

    struct Menu_Item *father;           // 父节点
    struct Menu_Item *first_son;        // 第一个子节点
    struct Menu_Item *next_brother;     // 下一个兄弟节点
    struct Menu_Item *last_brother;     // 上一个兄弟节点
}Menu_Item;


// 节点创建
void        Create_Menu_Folder              (Menu_Item *father, Menu_Item *me, const char name[]);
void        Create_Menu_Number              (Menu_Item *father, Menu_Item *me, const char name[], void *data, MENU_KIND kind);
void        Create_Menu_LimitNumber         (Menu_Item *father, Menu_Item *me, const char name[], void *data, MENU_KIND kind, float limit_min, float limit_max);

// 动态创建
Menu_Item*  DynamicCreate_Menu_Folder       (Menu_Item *father, const char name[]);
void        DynamicCreate_Menu_Number       (Menu_Item *father, const char name[], void *data, MENU_KIND kind);
void        DynamicCreate_Menu_LimitNumber  (Menu_Item *father, const char name[], void *data, MENU_KIND kind, float limit_min, float limit_max);

// 外部直接访问根节点和当前光标
extern Menu_Item    head;
extern Menu_Item    *key;

// 菜单运行
void        menu_reset                      (void);
void        menu_show                       (void);
void        menu_show_all                   (void);     // 清屏 + 刷新显示
void        menu_init                       (void);     // [DEMO] 多级菜单测试, 可删除

// 二选一: 四键方案 或 五键方案
 #define FOUR_BUTTON         // 四键方案(默认)  若要五键方案请注释此行, 并启用下方 FIVE_BUTTON
// #define FIVE_BUTTON

// 外部按键接口(二次封装好的逻辑, 直接在按键扫描中调用)
#ifdef FOUR_BUTTON
void        key_up_btn                      (void);     // 上 / 参数加
void        key_down_btn                    (void);     // 下 / 参数减
void        key_enter_btn                   (void);     // 进入 / 选中
void        key_quit_btn                    (void);     // 退出 / 步进切换
#endif

#ifdef FIVE_BUTTON
void        key_up_btn                      (void);     // 上 / 参数加
void        key_down_btn                    (void);     // 下 / 参数减
void        key_left_btn                    (void);     // 左: 取消选中 / 退出
void        key_right_btn                   (void);     // 右: 进入 
void        key_mid_btn                     (void);     // 中: 步进切换
#endif


#endif
