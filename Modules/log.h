/**
 * @file log.h
 * @brief  日志系统
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-05-08
 *
 * THINK DIFFERENTLY
 */

#ifndef __LOG_H__
#define __LOG_H__

#include "mod_config.h"
#include "uart_pack.h"
/********** 日志系统设置 **********/
// 调试信息设置
#define _ENABLE_LOG 1            // 是否输出调试信息
#define _ENABLE_LOG_TIMESTAMP 0  // 调试信息是否添加时间戳
#define _ENABLE_LOG_COLOR 1      // 调试信息是否按等级添加颜色
#define _ENABLE_LOG_FUNC_LINE 0  // 调试信息是否添加函数名和行号
#define _ENABLE_LOG_ASSERT 1     // 是否开启ASSERT
// 调试信息等级
#define _ENABLE_LOG_DEBUG 1  // 是否输出DEBUG信息
#define _ENABLE_LOG_INFO 1   // 是否输出INFO信息
#define _ENABLE_LOG_WARN 1   // 是否输出WARN信息
#define _ENABLE_LOG_ERROR 1  // 是否输出ERROR信息
#define _ENABLE_LOG_FATAL 1  // 是否输出FATAL信息
// LOG输出设置
#define _LOG_PRINTF printf  // 调试信息输出函数
#define _GET_TIMESTAMP() ((double)m_time_us() / 1000000)  // 获取时间戳
#define _TIMESTAMP_FMT "%.6f"                             // 时间戳格式
/********** ********** **********/

#if _ENABLE_LOG
#if _ENABLE_LOG_TIMESTAMP && _ENABLE_LOG_COLOR
#define _DBG_LOG(level, color, fmt, args...)                        \
  _LOG_PRINTF("\033[" #color "m[" level "/" _TIMESTAMP_FMT "] " fmt \
              "\033[0m\r\n",                                        \
              _GET_TIMESTAMP(), ##args)
#elif !_ENABLE_LOG_TIMESTAMP && _ENABLE_LOG_COLOR
#define _DBG_LOG(level, color, fmt, args...) \
  _LOG_PRINTF("\033[" #color "m[" level "] " fmt "\033[0m\r\n", ##args)
#elif _ENABLE_LOG_TIMESTAMP && !_ENABLE_LOG_COLOR
#define _DBG_LOG(level, color, fmt, args...)                                  \
  _LOG_PRINTF("[" level "/" _TIMESTAMP_FMT "] " fmt "\r\n", _GET_TIMESTAMP(), \
              ##args)
#elif !_ENABLE_LOG_TIMESTAMP && !_ENABLE_LOG_COLOR
#define _DBG_LOG(level, color, fmt, args...) \
  _LOG_PRINTF("[" level "] " fmt "\r\n", ##args)
#endif
#if _ENABLE_LOG_FUNC_LINE
#define _DBG_LOG_FUNC(level, color, fmt, args...) \
  _DBG_LOG(level, color, "[%s:%d] " fmt, __func__, __LINE__, ##args)
#else
#define _DBG_LOG_FUNC _DBG_LOG
#endif  // _ENABLE_LOG_FUNC_LINE
#if _ENABLE_LOG_DEBUG
#define LOG_D(fmt, args...) _DBG_LOG_FUNC("D", 36, fmt, ##args)
#endif
#if _ENABLE_LOG_INFO
#define LOG_I(fmt, args...) _DBG_LOG_FUNC("I", 32, fmt, ##args)
#endif
#if _ENABLE_LOG_WARN
#define LOG_W(fmt, args...) _DBG_LOG_FUNC("W", 33, fmt, ##args)
#endif
#if _ENABLE_LOG_ERROR
#define LOG_E(fmt, args...) _DBG_LOG_FUNC("E", 31, fmt, ##args)
#endif
#if _ENABLE_LOG_FATAL
#define LOG_F(fmt, args...) _DBG_LOG_FUNC("F", 35, fmt, ##args)
#endif
#define LOG_RAW(fmt, args...) _LOG_PRINTF(fmt, ##args)

#if _ENABLE_LOG_COLOR
#define LOG_REFRESH(fmt, args...) \
  _LOG_PRINTF("\r\033[34m[R] " fmt "   \033[0m", ##args)
#define LOG_LIMIT(limit_ms, fmt, args...)                    \
  {                                                          \
    static m_time_t SAFE_NAME(limited_log_t) = 0;            \
    if (m_time_ms() > SAFE_NAME(limited_log_t) + limit_ms) { \
      SAFE_NAME(limited_log_t) = m_time_ms();                \
      _LOG_PRINTF("\033[34m[L] " fmt "\033[0m\r\n", ##args); \
    }                                                        \
  }
#else
#define LOG_REFRESH(fmt, args...) _LOG_PRINTF("\r[R] " fmt "   ", ##args)
#define LOG_LIMIT(limit_ms, fmt, args...)                    \
  {                                                          \
    static m_time_t SAFE_NAME(limited_log_t) = 0;            \
    if (m_time_ms() > SAFE_NAME(limited_log_t) + limit_ms) { \
      SAFE_NAME(limited_log_t) = m_time_ms();                \
      _LOG_PRINTF("[L] " fmt "\r\n", ##args);                \
    }                                                        \
  }
#endif  // _ENABLE_LOG_COLOR
#define LOG_ENDL() LOG_RAW("\r\n")
#endif  // _ENABLE_LOG

#ifndef LOG_D
#define LOG_D(...) ((void)0)
#endif
#ifndef LOG_I
#define LOG_I(...) ((void)0)
#endif
#ifndef LOG_W
#define LOG_W(...) ((void)0)
#endif
#ifndef LOG_E
#define LOG_E(...) ((void)0)
#endif
#ifndef LOG_F
#define LOG_F(...) ((void)0)
#endif
#ifndef LOG_RAW
#define LOG_RAW(...) ((void)0)
#endif
#ifndef LOG_REFRESH
#define LOG_REFRESH(...) ((void)0)
#endif
#ifndef LOG_LIMIT
#define LOG_LIMIT(...) ((void)0)
#endif
#ifndef LOG_ENDL
#define LOG_ENDL(...) ((void)0)
#endif

#define _TERM_COLOR(color) "\033[" #color "m"
#define _TERM_RESET _TERM_COLOR(0)
#define _TERM_RED _TERM_COLOR(31)
#define _TERM_GREEN _TERM_COLOR(32)
#define _TERM_YELLOW _TERM_COLOR(33)
#define _TERM_BLUE _TERM_COLOR(34)
#define _TERM_MAGENTA _TERM_COLOR(35)
#define _TERM_CYAN _TERM_COLOR(36)
#define _TERM_WHITE _TERM_COLOR(37)

#if _ENABLE_LOG_ASSERT
#define __ASSERT_PRINT(text) _DBG_LOG("A", 31, text)
#define __ASSERT_0(expr)                       \
  if (!(expr)) {                               \
    __ASSERT_PRINT("Failed expr: " #expr);     \
    Assert_Failed_Handler(__FILE__, __LINE__); \
  }
#define __ASSERT_1(expr, text)                 \
  if (!(expr)) {                               \
    __ASSERT_PRINT(text);                      \
    Assert_Failed_Handler(__FILE__, __LINE__); \
  }
#define __ASSERT_2(expr, text, cmd) \
  if (!(expr)) {                    \
    __ASSERT_PRINT(text);           \
    cmd;                            \
  }

// 断言, param: 表达式, 错误信息(可选), 自定义语句(可选)
// 断言失败时默认调用Assert_Failed_Handler
#define ASSERT(expr, ...)      \
  EVAL(__ASSERT_, __VA_ARGS__) \
  (expr, ##__VA_ARGS__)
#else
#define ASSERT(expr, ...) ((void)0)
#endif

#if _MOD_USE_PERF_COUNTER
#define timeit(NAME)                                                         \
  __cycleof__("", {                                                          \
    LOG_D("timeit(" NAME ")=%fus", (double)_ / (SystemCoreClock / 1000000)); \
  })

#define cycleit(NAME) __cycleof__("", { LOG_D("cycleit(" NAME ")=%d", _); })

#define timeit_limit(NAME, limit_ms)                  \
  __cycleof__("", {                                   \
    LOG_LIMIT(limit_ms, "timeit(" NAME ")=%fus",      \
              (double)_ * 1000000 / SystemCoreClock); \
  })
#define cycleit_limit(NAME, limit_ms) \
  __cycleof__("", { LOG_LIMIT(limit_ms, "cycleit(" NAME ")=%d", _); })

#define timeit_avg(NAME, N)                                               \
  double SAFE_NAME(timeit_avg) = N;                                       \
  __cycleof__("", {                                                       \
    LOG_D("timeit_avg(" NAME ")=%fus",                                    \
          (double)_ * 1000000 / SystemCoreClock / SAFE_NAME(timeit_avg)); \
  })
#endif

#endif  // __LOG_H
