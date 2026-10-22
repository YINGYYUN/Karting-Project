/*********************************************************************************************************************
 * Flash 页分配约定（**唯一权威**）
 *
 * 硬件：Work Flash 192 KB = 96 页 × 2 KB；驱动按 uint32 字数写（FLASH_PAGE_LENGTH = 512）。
 *
 * 页 0        ：参数持久化（Param_Storage）
 * 页 1        ：路表槽索引
 * 页 2 ~ 86   ：路表数据槽 5 × 17 页（1 头页 + 16 记录页 × 128 条）
 * 页 87 ~ 95  ：空闲
 *
 * ⚠ 新增功能先在这里登记页段，不要在别处硬编码页号。
 ********************************************************************************************************************/

#ifndef _FLASH_MAP_H
#define _FLASH_MAP_H

#include "zf_common_headfile.h"

/* ---- 总容量 ---- */
#define FM_PAGE_SIZE            (2048u)                         /* 每页字节 */
#define FM_PAGE_TOTAL           (96u)                           /* 总页数(0~95) */

/* ---- 页 0：参数 ---- */
#define FM_PARM_PAGE            (0u)

/* ---- 页 1：槽索引 ---- */
#define FM_SLOT_INDEX_PAGE      (1u)

/* ---- 数据槽池：5 槽 × 17 页 ---- */
#define FM_REC_SIZE             (16u)                           /* 每条记录字节 */
#define FM_REC_PER_PAGE         (FM_PAGE_SIZE / FM_REC_SIZE)    /* = 128 条/页 */

#define FM_SLOT_BASE            (2u)
#define FM_SLOT_COUNT           (5u)
#define FM_SLOT_PAGES           (17u)                           /* 1 头页 + 16 记录页 */
#define FM_SLOT_REC_PAGES       (FM_SLOT_PAGES - 1u)            /* = 16 */
#define FM_SLOT_BASE_PAGE(k)    ((uint32)FM_SLOT_BASE + (uint32)(k) * (uint32)FM_SLOT_PAGES)
#define FM_SLOT_PAGES_TOTAL     (FM_SLOT_COUNT * FM_SLOT_PAGES) /* = 85 页（2~86）*/
#define FM_SLOT_CAPACITY        (FM_SLOT_REC_PAGES * FM_REC_PER_PAGE)   /* = 2048 条 */

/* ---- 空闲段（页 87~95）---- */
#define FM_FREE_BASE            (FM_SLOT_BASE + FM_SLOT_PAGES_TOTAL)    /* = 87 */
#define FM_FREE_PAGES           (FM_PAGE_TOTAL - FM_FREE_BASE)          /* = 9  */

/* ---- 槽号语义（菜单 / 上位机 / 文档统一用这一份）---- */
#define FM_SLOT_MANUAL          (0u)    /* 手推 / 人开车录制 */
#define FM_SLOT_MAP_RACE        (1u)    /* 电子制图 · 正式赛道 */
#define FM_SLOT_MAP_DEBUG       (2u)    /* 电子制图 · 调试 */
#define FM_SLOT_TRAJ_A          (3u)    /* 实测轨迹 A */
#define FM_SLOT_TRAJ_B          (4u)    /* 实测轨迹 B */

/* 编译期守卫：各段不越界、不重叠 */
typedef char fm_param_fit_check[(FM_PARM_PAGE + 1u <= FM_SLOT_INDEX_PAGE) ? 1 : -1];
typedef char fm_index_fit_check[(FM_SLOT_INDEX_PAGE + 1u <= FM_SLOT_BASE) ? 1 : -1];
typedef char fm_slot_fit_check [(FM_SLOT_BASE + FM_SLOT_PAGES_TOTAL <= FM_FREE_BASE) ? 1 : -1];
typedef char fm_free_fit_check [(FM_FREE_BASE + FM_FREE_PAGES <= FM_PAGE_TOTAL) ? 1 : -1];
typedef char fm_rec_fit_check  [(FM_REC_PER_PAGE * FM_REC_SIZE <= FM_PAGE_SIZE) ? 1 : -1];

#endif
