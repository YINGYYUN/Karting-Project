/*******************************************************************************
链式通用菜单框架
参考:【手把手教菜单_框架搭建】 https://www.bilibili.com/video/BV1LCh9zrEMp/?share_source=copy_web&vd_source=befff269c2e0ef1583e2528659770521
*******************************************************************************/


#include "zf_common_headfile.h"
#include "common_menu.h"


// 静态内存池(用于分配动态创建的菜单)
#define MENU_MAX_SIZE           (24)

static Menu_Item menu_item_arr[MENU_MAX_SIZE];
static uint8_t menu_arr_index = 0;

Menu_Item head = {
    .name           = "PARAM_Menu",
    .data           = NULL,
    .kind           = MENU_Folder,

    .sons           = 0,
    .No             = 0,
    .select         = false,

    .isLimit        = false,
    .limit_min      = 0,
    .limit_max      = 0,

    .father         = NULL,
    .first_son      = NULL,
    .next_brother   = NULL,
    .last_brother   = NULL,
};
Menu_Item *key;

// 步进值控制
// 加了 100 这一档：速度环换算后的增益是几百~上千（如 k_z≈1249），只靠 10 要按上百次
#define SETUP_LEN               (5)
static float SetupNumber[SETUP_LEN] = {0.01f, 0.1f, 1, 10, 100};
static uint8_t SetupIndex = 2;          // 默认步进 = 1


/**********************************************************/
/*[S] 节点创建函数 [S]------------------------------------*/
/**********************************************************/

// 内部通用创建函数
static void __create_menu_item(Menu_Item *father, Menu_Item *me, const char name[], void *data, MENU_KIND kind, bool isLimit, float limit_min, float limit_max)
{
    if (father->kind != MENU_Folder) return;

    me->name = name;
    me->data = data;
    me->kind = kind;

    me->sons = 0;
    me->select = false;

    me->isLimit = isLimit;
    me->limit_min = limit_min;
    me->limit_max = limit_max;

    me->father = father;
    me->first_son = NULL;       // 第一个子节点先设为空
    me->next_brother = NULL;
    me->last_brother = NULL;

    // 开始判定连接
    if (father->sons == 0)
    {
        father->first_son = me;
    }
    else
    {
        Menu_Item *p = father->first_son;
        while(p->next_brother != NULL)
            p = p->next_brother;
        p->next_brother = me;
        me->last_brother = p;
    }
    me->No = father->sons;
    father->sons++;
}

// 创建文件夹
void Create_Menu_Folder(Menu_Item *father, Menu_Item *me, const char name[])
{
    __create_menu_item(father, me, name, NULL, MENU_Folder, false, 0, 0);
}

// 创建文件
void Create_Menu_Number(Menu_Item *father, Menu_Item *me, const char name[], void *data, MENU_KIND kind)
{
    __create_menu_item(father, me, name, data, kind, false, 0, 0);
}

// 创建带限幅的文件
void Create_Menu_LimitNumber(Menu_Item *father, Menu_Item *me, const char name[], void *data, MENU_KIND kind, float limit_min, float limit_max)
{
    __create_menu_item(father, me, name, data, kind, true, limit_min, limit_max);
}

// 动态创建文件夹(返回指针, 用于继续添加子节点)
Menu_Item* DynamicCreate_Menu_Folder(Menu_Item *father, const char name[])
{
    if (menu_arr_index >= MENU_MAX_SIZE) return NULL;
    // 分配内存池
    Menu_Item *me = &menu_item_arr[menu_arr_index++];
    Create_Menu_Folder(father, me, name);

    return me;
}

// 动态创建文件
void DynamicCreate_Menu_Number(Menu_Item *father, const char name[], void *data, MENU_KIND kind)
{
    if (menu_arr_index >= MENU_MAX_SIZE) return;
    // 分配内存池
    Menu_Item *me = &menu_item_arr[menu_arr_index++];
    Create_Menu_Number(father, me, name, data, kind);
}

// 动态创建带限幅的文件
void DynamicCreate_Menu_LimitNumber(Menu_Item *father, const char name[], void *data, MENU_KIND kind, float limit_min, float limit_max)
{
    if (menu_arr_index >= MENU_MAX_SIZE) return;
    // 分配内存池
    Menu_Item *me = &menu_item_arr[menu_arr_index++];
    Create_Menu_LimitNumber(father, me, name, data, kind, limit_min, limit_max);
}
/**********************************************************/
/*------------------------------------[E] 节点创建函数 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 显示调用函数 [S]------------------------------------*/
/**********************************************************/

// 显示标题栏(当前路径)
static void show_title(void)
{
    char buf[31];
    // 填充空格底色
    memset(buf, ' ', 30);
    buf[30] = '\0';

    // 左侧: 父节点名称 + "/"
    int len = sprintf(buf, "%s/", key->father->name);
    buf[len] = ' ';
    buf[20] = '\0';     // 截断, 给右侧步进值留空间

    ips200_show_string(0, 0, buf);
}

// 显示步进值(固定 7 字符宽, 避免短字符串残留旧像素)
static void show_setup(void)
{
    char buf[8];
    memset(buf, ' ', 7);
    buf[7] = '\0';

    if (SetupNumber[SetupIndex] < 1)
    {
        // <0.00>
        int len = sprintf(buf, "<%.2f>", SetupNumber[SetupIndex]);
        buf[len] = ' ';     // 抹掉 sprintf 写入的 '\0', 让后面空格继续显示
    }
    else
    {
        // <1000>
        int len = sprintf(buf, "<%.0f>", SetupNumber[SetupIndex]);
        buf[len] = ' ';
    }

    ips200_show_string(160, 0, buf);
}

// 显示菜单项名称
static void show_task(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;

    for (int i = 0; i < h->sons; i++)
    {
        int y = (i + 2) * 16;       // 第0行标题栏, 第1行分隔线

        switch (s->kind)
        {
            case MENU_Folder:
                ips200_printf(10, y, "%s[%d]", s->name, s->sons);
                break;
            default:
                ips200_show_string(10, y, s->name);
                break;
        }
        s = s->next_brother;
    }

    // 清空剩余行
    for (int i = h->sons; i < 15; i++)
    {
        ips200_show_string(0, (i + 2) * 16, "                              ");
    }
}

// 显示光标(当前位置指示)
static void show_key(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;

    for (int i = 0; i < h->sons; i++)
    {
        int y = (i + 2) * 16;

        if (s == key)
            ips200_show_string(0, y, ">");
        else
            ips200_show_string(0, y, " ");
        s = s->next_brother;
    }
}

// 显示存储的参数
static void show_data(void)
{
    Menu_Item *h = key->father;
    Menu_Item *s = h->first_son;

    for (int i = 0; i < h->sons; i++)
    {
        int y = (i + 2) * 16;

        // 选中标记
        if (s->select)
            ips200_show_string(142, y, "[");
        else
            ips200_show_string(142, y, " ");

        switch (s->kind)
        {
            case MENU_Folder:
                // 文件夹不显示数据
                break;

            case int32_Box:
                ips200_printf(150, y, "%05d", *(int32_t *)s->data);
                break;

            case uint32_Box:
                ips200_printf(150, y, "%05u", *(uint32_t *)s->data);
                break;

            case int16_Box:
                ips200_printf(150, y, "%05d", *(int16_t *)s->data);
                break;

            case uint16_Box:
                ips200_printf(150, y, "%05u", *(uint16_t *)s->data);
                break;

            case int8_Box:
                ips200_printf(150, y, "%05d", *(int8_t *)s->data);
                break;

            case uint8_Box:
                ips200_printf(150, y, "%05u", *(uint8_t *)s->data);
                break;

            case float_Box:
                ips200_printf(150, y, "%3.2f", *(float *)s->data);
                break;

            case bool_Box:
                if (*(bool *)s->data == true)
                    ips200_printf(150, y, "Y");
                else
                    ips200_printf(150, y, "N");
                break;

            default:
                break;
        }

        s = s->next_brother;
    }
}

void menu_show(void)
{
    show_title();
    show_setup();
    ips200_show_string(0, 16, "==============================");
    show_task();
    show_key();
    show_data();
}

// 清屏 + 刷新显示(进入新页面时调用)
void menu_show_all(void)
{
    ips200_clear();
    menu_show();
}
/**********************************************************/
/*------------------------------------[E] 显示调用函数 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 按键一次封装 [S]------------------------------------*/
/**********************************************************/

// 光标上移
static void key_up(void)
{
    if (key->last_brother != NULL)
        key = key->last_brother;
}

// 光标下移
static void key_down(void)
{
    if (key->next_brother != NULL)
        key = key->next_brother;
}

// 进入菜单
static void key_enter(void)
{
    if (key->sons > 0)
    {
        key = key->first_son;
        ips200_clear();
    }
}

// 选中/取消选中
static void key_select(void)
{
    if (key->kind != MENU_Folder && key->kind != bool_Box)
        key->select = !key->select;
}

// 切换 bool 值
static void key_toggle(void)
{
    if (key->kind == bool_Box)
        *(bool *)key->data = !*(bool *)key->data;
}

// 步进值循环切换(加)
static void setup_ctrl_plus(void)
{
    SetupIndex = (SetupIndex + 1) % SETUP_LEN;
}

// // 步进值循环切换(减)
// static void setup_ctrl_sub(void)
// {
//     SetupIndex = (SetupIndex - 1 + SETUP_LEN) % SETUP_LEN;
// }

// 参数加
static void key_plus(void)
{
    float step = SetupNumber[SetupIndex];

    switch (key->kind)
    {
        case int32_Box:
        {
            int32_t val = *(int32_t *)key->data + (int32_t)step;
            if (key->isLimit && val > (int32_t)key->limit_max)
                val = (int32_t)key->limit_max;
            *(int32_t *)key->data = val;
            break;
        }
        case uint32_Box:
        {
            uint32_t val = *(uint32_t *)key->data + (uint32_t)step;
            if (key->isLimit && val > (uint32_t)key->limit_max)
                val = (uint32_t)key->limit_max;
            *(uint32_t *)key->data = val;
            break;
        }
        case int16_Box:
        {
            int16_t val = *(int16_t *)key->data + (int16_t)step;
            if (key->isLimit && val > (int16_t)key->limit_max)
                val = (int16_t)key->limit_max;
            *(int16_t *)key->data = val;
            break;
        }
        case uint16_Box:
        {
            uint16_t val = *(uint16_t *)key->data + (uint16_t)step;
            if (key->isLimit && val > (uint16_t)key->limit_max)
                val = (uint16_t)key->limit_max;
            *(uint16_t *)key->data = val;
            break;
        }
        case int8_Box:
        {
            int8_t val = *(int8_t *)key->data + (int8_t)step;
            if (key->isLimit && val > (int8_t)key->limit_max)
                val = (int8_t)key->limit_max;
            *(int8_t *)key->data = val;
            break;
        }
        case uint8_Box:
        {
            uint8_t val = *(uint8_t *)key->data + (uint8_t)step;
            if (key->isLimit && val > (uint8_t)key->limit_max)
                val = (uint8_t)key->limit_max;
            *(uint8_t *)key->data = val;
            break;
        }
        case float_Box:
        {
            float val = *(float *)key->data + step;
            if (key->isLimit && val > key->limit_max)
                val = key->limit_max;
            *(float *)key->data = val;
            break;
        }

        default:
            break;
    }
}

// 参数减
static void key_sub(void)
{
    float step = SetupNumber[SetupIndex];

    switch (key->kind)
    {
        case int32_Box:
        {
            int32_t val = *(int32_t *)key->data - (int32_t)step;
            if (key->isLimit && val < (int32_t)key->limit_min)
                val = (int32_t)key->limit_min;
            *(int32_t *)key->data = val;
            break;
        }
        case uint32_Box:
        {
            uint32_t val = *(uint32_t *)key->data - (uint32_t)step;
            if (key->isLimit && val < (uint32_t)key->limit_min)
                val = (uint32_t)key->limit_min;
            *(uint32_t *)key->data = val;
            break;
        }
        case int16_Box:
        {
            int16_t val = *(int16_t *)key->data - (int16_t)step;
            if (key->isLimit && val < (int16_t)key->limit_min)
                val = (int16_t)key->limit_min;
            *(int16_t *)key->data = val;
            break;
        }
        case uint16_Box:
        {
            uint16_t val = *(uint16_t *)key->data - (uint16_t)step;
            if (key->isLimit && val < (uint16_t)key->limit_min)
                val = (uint16_t)key->limit_min;
            *(uint16_t *)key->data = val;
            break;
        }
        case int8_Box:
        {
            int8_t val = *(int8_t *)key->data - (int8_t)step;
            if (key->isLimit && val < (int8_t)key->limit_min)
                val = (int8_t)key->limit_min;
            *(int8_t *)key->data = val;
            break;
        }
        case uint8_Box:
        {
            uint8_t val = *(uint8_t *)key->data - (uint8_t)step;
            if (key->isLimit && val < (uint8_t)key->limit_min)
                val = (uint8_t)key->limit_min;
            *(uint8_t *)key->data = val;
            break;
        }
        case float_Box:
        {
            float val = *(float *)key->data - step;
            if (key->isLimit && val < key->limit_min)
                val = key->limit_min;
            *(float *)key->data = val;
            break;
        }

        default:
            break;
    }
}
/**********************************************************/
/*------------------------------------[E] 按键一次封装 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 按键二次封装 [S]------------------------------------*/
/**********************************************************/

#ifdef FOUR_BUTTON

// 四键方案: 上 / 下 / 确定 / 取消

// 按键1: 上 / 参数加
void key_up_btn(void)
{
    if (key->select == false)
        key_up();
    else
        key_plus();
}

// 按键2: 下 / 参数减
void key_down_btn(void)
{
    if (key->select == false)
        key_down();
    else
        key_sub();
}

// 按键3: 进入 / 选中 / 切换bool
void key_enter_btn(void)
{
    if (key->kind == MENU_Folder)
        key_enter();
    else if (key->kind == bool_Box)
        key_toggle();
    else
        key_select();
}

// 按键4: 退出 / 步进切换
void key_quit_btn(void)
{
    // 如果当前选中的是数值类型, 且处于选中状态, 则切换步进值
    if (key->select == true)
    {
        setup_ctrl_plus();
        return;
    }

    // 否则执行退出操作(不允许退到 head 层以上)
    if (key->father->father == NULL) return;
    key = key->father;
    ips200_clear();
}
#endif


#ifdef FIVE_BUTTON

// 五键方案: 上 / 下 / 左 / 右 / 中

// 上键: 上 / 参数加
void key_up_btn(void)
{
    if (key->select == false)
        key_up();
    else
        key_plus();
}

// 下键: 下 / 参数减
void key_down_btn(void)
{
    if (key->select == false)
        key_down();
    else
        key_sub();
}

// 右键: 进入 / 选中 / 切换bool
void key_right_btn(void)
{
    if (key->kind == MENU_Folder)
        key_enter();
    else if (key->kind == bool_Box)
        key_toggle();
    else
        key_select();
}

// 左键: 取消选中 / 退出
void key_left_btn(void)
{
    if (key->select == true)
    {
        key->select = false;    // 正在编辑 → 取消选中
        return;
    }
    //执行退出操作(不允许退到 head 层以上)
    if (key->father->father == NULL) return;
    key = key->father;
    ips200_clear();
}

// 中键: 步进切换
void key_mid_btn(void)
{
    // 如果当前选中的是数值类型, 且处于选中状态, 则切换步进值
    if (key->select == true)
    {
        setup_ctrl_plus();
        return;
    }
}
#endif
/**********************************************************/
/*------------------------------------[E] 按键二次封装 [E]*/
/**********************************************************/


/**********************************************************/
/*[S] 菜单重置 & 测试实例 [S]------------------------------*/
/**********************************************************/

// 重置菜单(切换页面时调用, 清空内存池并重置根节点)
void menu_reset(void)
{
    menu_arr_index = 0;
    memset(menu_item_arr, 0, sizeof(menu_item_arr));

    head.sons = 0;
    head.first_son = NULL;
    head.next_brother = NULL;
    head.last_brother = NULL;

    key = NULL;
}

// // ---- 以下为测试 DEMO, 可删除 ----
// int test = 10;
// float test1 = 2.2f;
// bool test2 = false;
// uint8_t test_u8 = 50;
// uint16_t test_u16 = 1000;

// // 测试菜单(多级文件夹演示)
// void menu_init(void)
// {
//     menu_reset();

//     Menu_Item *folder_1 = DynamicCreate_Menu_Folder(&head, "folder1");
//     DynamicCreate_Menu_Folder(&head, "folder2");
//     DynamicCreate_Menu_Folder(&head, "folder3");
//     DynamicCreate_Menu_Folder(&head, "folder4");

//     DynamicCreate_Menu_Number(folder_1, "aaa", &test, int32_Box);
//     DynamicCreate_Menu_Number(folder_1, "bbb", &test1, float_Box);
//     DynamicCreate_Menu_Number(folder_1, "ccc", &test2, bool_Box);

//     Menu_Item *folder_1_1 = DynamicCreate_Menu_Folder(folder_1, "sub_f1");
//     Menu_Item *folder_1_2 = DynamicCreate_Menu_Folder(folder_1, "sub_f2");

//     DynamicCreate_Menu_LimitNumber(folder_1_1, "u8_num",  &test_u8,  uint8_Box,  0, 100);
//     DynamicCreate_Menu_LimitNumber(folder_1_1, "u16_num", &test_u16, uint16_Box, 0, 10000);

//     key = head.first_son;
// }
/**********************************************************/
/*------------------------------------[E] 菜单重置 & 测试 [E]*/
/**********************************************************/
