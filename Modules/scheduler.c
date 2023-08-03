/**
 * @file scheduler.c
 * @brief 时分调度器和宏协程实现
 * @author Ellu (lutaoyu@163.com)
 * @version 3.0
 * @date 2023年3月20日
 *
 * THINK DIFFERENTLY
 */

#include <scheduler.h>
#include <stdlib.h>

scheduler_task_t *schTaskEntry = NULL;
scheduler_task_t *task_p = NULL;
#if _SUPPORT_HIGH_PRIORITY
scheduler_task_t *task_highPriority_p = NULL;
scheduler_task_t *task_hpBack_p = NULL;
static uint8_t schTask_hpJmp = 0;
#endif
#if _SUPPORT_COROUTINE
scheduler_cron_t *schCronEntry = NULL;
scheduler_cron_t *cron_p = NULL;
#endif
#if _SUPPORT_CALL_LATER
scheduler_call_later_t *schCallLaterEntry = NULL;
scheduler_call_later_t *callLater_p = NULL;
#endif

/**
 * @brief 添加一个任务到调度器
 * @param  task             任务函数指针
 * @param  rateHz           任务调度频率
 * @param  enable           初始化时是否使能
 * @retval uint16_t          任务ID(0xffff表示堆内存分配失败)
 */
uint16_t Add_SchTask(void (*task)(void), float rateHz, uint8_t enable) {
  scheduler_task_t *p = (scheduler_task_t *)malloc(sizeof(scheduler_task_t));
  if (p == NULL) return 0xffff;
  p->task = task;
  p->rateHz = rateHz;
  p->period = (double)m_tick_clk / (double)p->rateHz;
  p->lastRun = m_tick() - p->period;
  p->enable = enable;
  p->dTParam = 0;
  p->taskId = 0;
  p->next = NULL;
  if (schTaskEntry == NULL) {
    schTaskEntry = p;
  } else {
    scheduler_task_t *q = schTaskEntry;
    while (q->next != NULL) {
      q = q->next;
    }
    q->next = p;
    p->taskId = q->taskId + 1;
  }
  return p->taskId;
}

/**
 * @brief 添加一个任务到调度器, 调度时传递上一次调度到本次调度的时间差
 * @param  task             任务函数指针
 * @param  rateHz           任务调度频率
 * @param  enable           初始化时是否使能
 * @retval uint16_t          任务ID(0xffff表示堆内存分配失败)
 */
uint16_t Add_SchTask_dTParam(void (*task)(m_time_t dT), float rateHz,
                             uint8_t enable) {
  scheduler_task_t *p = (scheduler_task_t *)malloc(sizeof(scheduler_task_t));
  if (p == NULL) return 0xffff;
  p->task = (void (*)(void))task;
  p->rateHz = rateHz;
  p->period = (double)m_tick_clk / (double)p->rateHz;
  p->lastRun = m_tick() - p->period;
  p->enable = enable;
  p->dTParam = 1;
  p->taskId = 0;
  p->next = NULL;
  if (schTaskEntry == NULL) {
    schTaskEntry = p;
  } else {
    scheduler_task_t *q = schTaskEntry;
    while (q->next != NULL) {
      q = q->next;
    }
    q->next = p;
    p->taskId = q->taskId + 1;
  }
  return p->taskId;
}

#if _ENABLE_SCH_DEBUG
#warning 调度器调试模式已开启
#include <uart_pack.h>

void Print_Debug_info(m_time_t period) {
  static uint8_t first_print = 1;
  double temp = m_tick_per_us;
  m_time_t other = period;
  scheduler_task_t *p = schTaskEntry;
  printf("\r\n-------------- Task %ds stat ----------------\r\n",
         _SCH_DEBUG_INFO_PERIOD);
  printf(" ID | Run | Tmax/us | Usage/%% | Late(max)/us\r\n");
  while (p != NULL) {
    if (p->enable) {
      printf(" #%-3d %-5d %-9.3f %-9.3f %.3f(%.2f) \r\n", p->taskId, p->run_cnt,
             (double)p->max_cost / temp, (double)p->total_cost / period * 100,
             (double)p->total_lat / p->run_cnt / temp,
             (double)p->max_lat / temp);
    }
    other -= p->total_cost;
    p = p->next;
  }
  printf("Other: %.3f %% (%.3fus)\r\n", (double)other / period * 100,
         (double)other / temp);
  printf("Coreclock: %.3f Mhz\r\n", temp);
  if (first_print) {
    printf("Note: the first report is inaccurate.\r\n");
    first_print = 0;
  }
  printf("---------------------------------------------\r\n");
  printf_flush();
  p = schTaskEntry;
  other = m_tick();
  while (p != NULL) {
    p->lastRun = other;
    p->max_cost = 0;
    p->total_cost = 0;
    p->run_cnt = 0;
    p->max_lat = 0;
    p->total_lat = 0;
    p = p->next;
  }
}

#endif  // _ENABLE_SCH_DEBUG

/**
 * @brief 时分调度器主函数
 * @param  block            是否阻塞
 **/
void __attribute__((always_inline)) Scheduler_Run(const uint8_t block) {
  static m_time_t now = 0;
  static m_time_t latency = 0;
#if _ENABLE_SCH_DEBUG
  static m_time_t _sch_debug_task_tick = 0;
  static m_time_t _sch_debug_last_print = 0;
#endif  // _ENABLE_SCH_DEBUG

  do {
#if _ENABLE_SCH_DEBUG
    if (task_p == NULL && m_tick() - _sch_debug_last_print >
                              _SCH_DEBUG_INFO_PERIOD * m_tick_clk) {
      Print_Debug_info(m_tick() - _sch_debug_last_print);
      _sch_debug_last_print = m_tick();
    }
#endif  // _ENABLE_SCH_DEBUG

  RUN_SCH:
    now = m_tick();
    if (schTaskEntry != NULL) {
      if (task_p == NULL) task_p = schTaskEntry;
      if (task_p->enable && (now >= task_p->lastRun + task_p->period)) {
        latency = now - (task_p->lastRun + task_p->period);
#if _ENABLE_SCH_DEBUG
        _sch_debug_task_tick = m_tick();
        task_p->task();
        _sch_debug_task_tick = m_tick() - _sch_debug_task_tick;
        if (task_p->max_cost < _sch_debug_task_tick)
          task_p->max_cost = _sch_debug_task_tick;
        if (latency > task_p->max_lat) task_p->max_lat = latency;
        task_p->total_cost += _sch_debug_task_tick;
        task_p->total_lat += latency;
        task_p->run_cnt++;
#else
        if (task_p->dTParam)
          ((void (*)(m_time_t))task_p->task)(now - task_p->lastRun);
        else
          task_p->task();
#endif  // _ENABLE_SCH_DEBUG
        if (latency <= _SCH_COMP_RANGE)
          task_p->lastRun += task_p->period;
        else
          task_p->lastRun = now;
      }
      if (task_p != NULL) task_p = task_p->next;

#if _SUPPORT_HIGH_PRIORITY
      if (task_highPriority_p != NULL && !schTask_hpJmp) {
        task_hpBack_p = task_p;
        task_p = task_highPriority_p;
        schTask_hpJmp = 1;
        goto RUN_SCH;
      } else if (schTask_hpJmp) {
        task_p = task_hpBack_p;
        schTask_hpJmp = 0;
      }
#endif  // _SUPPORT_HIGH_PRIORITY
    }

#if _SUPPORT_COROUTINE
    if (schCronEntry != NULL) {
      if (cron_p == NULL) cron_p = schCronEntry;
      if (cron_p->enable && m_time_us() >= cron_p->idleUntil) {
        cron_p->task();
        if (!cron_p->ptr) {
          if (cron_p->autodel) {
            Del_Corontine(cron_p->taskId);
            continue;
          } else if (cron_p->loop == 0)
            cron_p->enable = 0;
        }
      }
      cron_p = cron_p->next;
    }
#endif  // _SUPPORT_COROUTINE

#if _SUPPORT_CALL_LATER
    if (schCallLaterEntry != NULL) {
      if (callLater_p == NULL) callLater_p = schCallLaterEntry;
      if (m_time_us() >= callLater_p->runTimeUs) {
        callLater_p->task();
        Del_CallLater(callLater_p->task);
        continue;
      }
      callLater_p = callLater_p->next;
    }
#endif  // _SUPPORT_CALL_LATER

  } while (block);
}

/**
 * @brief 切换任务使能状态
 * @param  taskId           目标任务ID
 * @param  enable           使能状态(0xff:切换)
 */
void Set_SchTask_Enable(uint16_t taskId, uint8_t enable) {
  scheduler_task_t *p = schTaskEntry;
  while (p != NULL) {
    if (p->taskId == taskId) {
      if (enable == 0xff)
        p->enable = !p->enable;
      else
        p->enable = enable;
      break;
    }
    p = p->next;
  }
}

/**
 * @brief 删除一个任务
 * @param  taskId           目标任务ID
 */
void Del_SchTask(uint16_t taskId) {
  scheduler_task_t *p = schTaskEntry;
  scheduler_task_t *q = NULL;
  while (p != NULL) {
    if (p->taskId == taskId) {
      if (q == NULL) {
        schTaskEntry = p->next;
      } else {
        q->next = p->next;
      }
      if (task_p == p) task_p = p->next;
#if _SUPPORT_HIGH_PRIORITY
      if (task_highPriority_p == p) task_highPriority_p = NULL;
#endif
      free(p);
      break;
    }
    q = p;
    p = p->next;
  }
}

/**
 * @brief 设置任务调度频率
 * @param  taskId           目标任务ID
 * @param  freq             调度频率
 */
void Set_SchTask_Freq(uint16_t taskId, float freq) {
  scheduler_task_t *p = schTaskEntry;
  while (p != NULL) {
    if (p->taskId == taskId) {
      p->rateHz = freq;
      p->period = (double)m_tick_clk / (double)p->rateHz;
      if (p->period == 0) {
        p->period = 1;
      }
      break;
    }
    p = p->next;
  }
}

/**
 * @brief 获取调度器内任务数量
 */
int Get_SchTask_Num(void) {
  scheduler_task_t *p = schTaskEntry;
  int num = 0;
  while (p != NULL) {
    num++;
    p = p->next;
  }
  return num;
}

/**
 * @brief 查询指定函数对应的任务ID
 * @param  task             目标任务函数指针
 * @retval uint16_t         任务ID(0xffff:未找到)
 */
uint16_t Get_SchTask_Id(void (*task)(void)) {
  scheduler_task_t *p = schTaskEntry;
  while (p != NULL) {
    if (p->task == task) {
      return p->taskId;
    }
    p = p->next;
  }
  return 0xffff;
}
#if _SUPPORT_HIGH_PRIORITY
/**
 * @brief 设置高优先级任务(仅支持一个)
 * @param  taskId           目标任务ID(0xffff:取消)
 */
void Set_SchTask_HighPriority(uint16_t taskId) {
  scheduler_task_t *p = schTaskEntry;
  task_highPriority_p = NULL;
  if (taskId == 0xffff) return;
  while (p != NULL) {
    if (p->taskId == taskId) {
      task_highPriority_p = p;
      break;
    }
    p = p->next;
  }
}
#endif

#if _SUPPORT_COROUTINE

/**
 * @brief 添加一个协程
 * @param  task             任务函数指针
 * @param  enable           是否立即启动
 * @param  loop             是否循环执行
 * @param  autodel          是否自动删除
 * @param  delay            延时启动时间(us)
 * @retval uint16_t         任务ID(0xffff表示堆内存分配失败)
 */
uint16_t Add_Corontine(void (*task)(void), uint8_t enable, uint8_t loop,
                       uint8_t autodel, m_time_t delay) {
  scheduler_cron_t *p = (scheduler_cron_t *)malloc(sizeof(scheduler_cron_t));
  if (p == NULL) return 0xffff;
  p->task = task;
  p->enable = enable;
  p->loop = loop;
  p->idleUntil = delay + m_time_us();
  p->autodel = autodel;
  p->taskId = 0;
  p->next = NULL;
  if (schCronEntry == NULL) {
    schCronEntry = p;
  } else {
    scheduler_cron_t *q = schCronEntry;
    while (q->next != NULL) {
      q = q->next;
    }
    q->next = p;
    p->taskId = q->taskId + 1;
  }
  return p->taskId;
}

/**
 * @brief 设置协程使能状态
 * @param  taskId           目标任务ID
 * @param  enable           使能状态(0xff: 切换)
 */
void Set_Corontine_Enable(uint16_t taskId, uint8_t enable) {
  scheduler_cron_t *p = schCronEntry;
  while (p != NULL) {
    if (p->taskId == taskId) {
      if (enable == 0xff) {
        p->enable = !p->enable;
      } else {
        p->enable = enable;
      }
      break;
    }
    p = p->next;
  }
}

/**
 * @brief 删除一个协程
 * @param  taskId           目标任务ID
 */
void Del_Corontine(uint16_t taskId) {
  scheduler_cron_t *p = schCronEntry;
  scheduler_cron_t *q = NULL;
  while (p != NULL) {
    if (p->taskId == taskId) {
      if (q == NULL) {
        schCronEntry = p->next;
      } else {
        q->next = p->next;
      }
      if (cron_p == p) cron_p = p->next;
      free(p);
      break;
    }
    q = p;
    p = p->next;
  }
}

/**
 * @brief 获取调度器内协程数量
 */
int Get_Corontine_Num(void) {
  scheduler_cron_t *p = schCronEntry;
  int num = 0;
  while (p != NULL) {
    num++;
    p = p->next;
  }
  return num;
}

/**
 * @brief 查询指定函数对应的协程ID
 * @param  task             目标任务函数指针
 * @retval uint16_t         任务ID(0xffff:未找到)
 */
uint16_t Get_Corontine_Id(void (*task)(void)) {
  scheduler_cron_t *p = schCronEntry;
  while (p != NULL) {
    if (p->task == task) {
      return p->taskId;
    }
    p = p->next;
  }
  return 0xffff;
}
#endif  // _SUPPORT_COROUTINE

#if _SUPPORT_CALL_LATER
/**
 * @brief 在指定时间后执行目标函数
 * @param  task             任务函数指针
 * @param  delay            延时启动时间(us)
 */
void Call_Later(void (*task)(void), m_time_t delay) {
  scheduler_call_later_t *p =
      (scheduler_call_later_t *)malloc(sizeof(scheduler_call_later_t));
  if (p == NULL) return;
  p->task = task;
  p->runTimeUs = delay + m_time_us();
  p->next = NULL;
  if (schCallLaterEntry == NULL) {
    schCallLaterEntry = p;
  } else {
    scheduler_call_later_t *q = schCallLaterEntry;
    while (q->next != NULL) {
      q = q->next;
    }
    q->next = p;
  }
}

/**
 * @brief 删除一个已经添加的延时调用任务
 * @param task              任务函数指针
 */
void Del_CallLater(void (*task)(void)) {
  scheduler_call_later_t *q = schCallLaterEntry;
  scheduler_call_later_t *p = NULL;
  while (q != NULL) {
    if (q->task == task) {
      if (p == NULL) {
        schCallLaterEntry = q->next;
      } else {
        p->next = q->next;
      }
      free(q);
      break;
    }
    p = q;
    q = q->next;
  }
}
#endif  // _SUPPORT_CALL_LATER
