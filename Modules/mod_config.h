/**
 * @file mod_config.h
 * @brief 配置Modules文件夹下所有模块的共有参数
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-04-29
 *
 * THINK DIFFERENTLY
 */

#ifndef __MOD_CONFIG_H
#define __MOD_CONFIG_H

#include "main.h"
#include "stdint.h"

/****************************** 配置项 ******************************/
#define _MOD_USE_PERF_COUNTER 1  // 是否使用perf_counter模块提供所有时基
/*******************************************************************/

#if _MOD_USE_PERF_COUNTER
#include "perf_counter.h"
typedef int64_t m_time_t;
#define Init_Module_Timebase() init_cycle_counter(1);
#define m_time_ms() get_system_ms()
#define m_time_us() get_system_us()
#define m_time_ns() (get_system_us() * 1000)
#define m_time_s() (get_system_ms() / 1000)
#define m_delay_ms(x) delay_ms(x)
#define m_delay_us(x) delay_us(x)
#define m_delay_ns(x) delay_us(x / 1000)
#define m_delay_s(x) delay_ms(x * 1000)
#define m_tick() get_system_ticks()
#define m_tick_clk (SystemCoreClock)
#define m_tick_per_ms ((double)SystemCoreClock / 1000)
#define m_tick_per_us ((double)SystemCoreClock / 1000000)
#else
typedef uint32_t m_time_t;
#define Init_Module_Timebase() ((void)0)
#define m_time_ms() HAL_GetTick()
#define m_time_us() (HAL_GetTick() * 1000)
#define m_time_ns() (HAL_GetTick() * 1000000)
#define m_time_s() (HAL_GetTick() / 1000)
#define m_delay_ms(x) HAL_Delay(x)
#define m_delay_us(x) HAL_Delay(x / 1000)
#define m_delay_ns(x) HAL_Delay(x / 1000000)
#define m_delay_s(x) HAL_Delay(x * 1000)
#define m_tick() HAL_GetTick()
#define m_tick_clk (1000)
#define m_tick_per_ms (1)
#define m_tick_per_us (0.001)
#endif  // _MOD_USE_PERF_COUNTER
#endif  // __MOD_CONFIG_H
