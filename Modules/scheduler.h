/**
 * @file scheduler.h
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2021-12-11
 *
 * THINK DIFFERENTLY
 */

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "mod_config.h"
/**********************调度器设置**********************/
#define _SUPPORT_HIGH_PRIORITY 1  // 支持高优先级任务
#define _SUPPORT_COROUTINE 0      // 支持宏协程
#define _SUPPORT_CALL_LATER 0     // 支持延时调用

#define _SCH_COMP_RANGE 1 * m_tick_per_ms  // 任务调度自动补偿范围(TICK)

#define _ENABLE_SCH_DEBUG 0        // 调试模式(统计任务信息)
#define _SCH_DEBUG_INFO_PERIOD 10  // 调试报告打印周期(s)
/*****************************************************/

#ifndef ENABLE
#define ENABLE 1
#define DISABLE 0
#define TOGGLE 0xFF
#endif

typedef struct {       // 用户任务结构
  void (*task)(void);  // 任务函数指针
  float rateHz;        // 任务调度频率
  m_time_t period;     // 任务调度周期(Tick)
  m_time_t lastRun;    // 上次执行时间(Tick)
  uint8_t enable;      // 是否使能
  uint8_t dTParam;     // 是否传递dT参数
  uint16_t taskId;     // 任务ID
#if _ENABLE_SCH_DEBUG
  m_time_t max_cost;    // 任务最大执行时间(Tick)
  m_time_t total_cost;  // 任务总执行时间(Tick)
  m_time_t max_lat;     // 任务调度延迟(Tick)
  m_time_t total_lat;   // 任务调度延迟总和(Tick)
  uint16_t run_cnt;     // 任务执行次数
#endif
  void *next;
} __attribute__((packed)) scheduler_task_t;

uint16_t Add_SchTask(void (*task)(void), float rateHz, uint8_t enable);
uint16_t Add_SchTask_dTParam(void (*task)(m_time_t dT), float rateHz,
                             uint8_t enable);
extern void Scheduler_Run(const uint8_t block);

extern void Set_SchTask_Enable(uint16_t taskId, uint8_t enable);
extern void Del_SchTask(uint16_t taskId);
extern void Set_SchTask_Freq(uint16_t taskId, float freq);
extern int Get_SchTask_Num(void);
extern uint16_t Get_SchTask_Id(void (*task)(void));

#if _SUPPORT_HIGH_PRIORITY
extern void Set_SchTask_HighPriority(uint16_t taskId);
#endif

#define ADD_TASK(x, s)  // 保留兼容

#if _SUPPORT_COROUTINE

typedef struct {       // 协程任务结构
  void (*task)(void);  // 任务函数指针
  uint8_t enable;      // 是否使能
  uint8_t loop;        // 是否循环执行
  uint8_t autodel;     // 是否自动删除
  uint16_t taskId;     // 任务ID
  m_time_t idleUntil;  // 无阻塞延时结束时间(us)
  long ptr;            // 协程跳入地址
  void *next;
} scheduler_cron_t;

extern scheduler_task_t *task_p;
extern scheduler_cron_t *cron_p;
/**
 * @brief 初始化协程, 在协程函数开头调用
 */
#define TH_BEGIN()      \
  crap:;                \
  void *pcrap = &&crap; \
  if ((cron_p->ptr) != 0) goto *(void *)(cron_p->ptr)

/**
 * @brief 释放CPU, 让出时间片, 下次调度时从此处继续执行
 */
#define TH_YIELD()             \
  do {                         \
    __label__ l;               \
    (cron_p->ptr) = (long)&&l; \
    return;                    \
  l:;                          \
    (cron_p->ptr) = 0;         \
  } while (0)

/**
 * @brief 无阻塞等待直到条件满足
 */
#define TH_YIELD_UNTIL(cond)   \
  do {                         \
    __label__ l;               \
    (cron_p->ptr) = (long)&&l; \
  l:;                          \
    if (!(cond)) return;       \
    (cron_p->ptr) = 0;         \
  } while (0)

/**
 * @brief 无阻塞延时, 单位us
 */
#define TH_DELAY_US(us)                       \
  do {                                        \
    cron_p->idleUntil = get_system_us() + us; \
    TH_YIELD();                               \
  } while (0)

/**
 * @brief 无阻塞延时, 单位ms
 */
#define TH_DELAY(ms) TH_DELAY_US(ms * 1000)

/**
 * @brief 等待直到指定时间, 单位us
 */
#define TH_UNTIL_US(us)     \
  do {                      \
    cron_p->idleUntil = us; \
    TH_YIELD();             \
  } while (0)

/**
 * @brief 等待直到指定时间, 单位ms
 */
#define TH_UNTIL(ms) TH_UNTIL_US(ms * 1000)

extern uint16_t Add_Corontine(void (*task)(void), uint8_t enable, uint8_t loop,
                              uint8_t autodel, m_time_t delay);
extern void Set_Corontine_Enable(uint16_t taskId, uint8_t enable);
extern void Del_Corontine(uint16_t taskId);
extern int Get_Corontine_Num(void);
extern uint16_t Get_Corontine_Id(void (*task)(void));

/**
 * @brief 运行一次协程, 执行完毕后自动删除
 */
#define Run_Corontine(task) Add_Corontine(task, 1, 0, 1, 0)

/**
 * @brief 运行一次协程, 执行完毕后自动删除, 延时执行
 * @param delay 延时时间(us)
 */
#define Run_Corontine_Later(task, delay) Add_Corontine(task, 1, 0, 1, delay)
#endif  // _SUPPORT_COROUTINE

#if _SUPPORT_CALL_LATER
typedef struct {       // 延时调用任务结构
  void (*task)(void);  // 任务函数指针
  m_time_t runTimeUs;  // 执行时间(us)
  void *next;
} scheduler_call_later_t;

extern void Call_Later(void (*task)(void), m_time_t delayUs);
extern void Del_CallLater(void (*task)(void));
#endif  // _SUPPORT_CALL_LATER

#endif  // _SCHEDULER_H_
