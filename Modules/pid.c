/**
 * @file pid.c
 * @brief 一个健壮的PID实现
 * @author Ellu (lutaoyu@163.com)
 * @version 2.0
 * @date 2023-04-03
 * @note 使用说明见pid.h
 *
 * THINK DIFFERENTLY
 */

#include "pid.h"

#include "math.h"

/**
 * @brief 计算位置式PID
 * @retval 输出
 */
float __attribute__((flatten)) PID_Calculate(pid_t *PIDx, float nextPoint) {
  PIDx->output = PIDx->base;
  /* Error */
  PIDx->error_1 = PIDx->error_0;
  PIDx->error_0 = PIDx->setPoint - nextPoint;
  /* Dead band */
  if (PIDx->deadBand > 0)
    if (PIDx->error_0 < PIDx->deadBand && PIDx->error_0 > -PIDx->deadBand) {
      PIDx->error_0 = 0;
    }
  /* Proportion */
  switch (PIDx->propMode) {
    case 0:  // const
      PIDx->output += PIDx->proportion * PIDx->error_0;
      break;
    case 1:  // 基于测量值的比例项(PonM)
      // PIDx->sumP = PIDx->proportion * (PIDx->base - nextPoint);
      if (PIDx->lastPoint != 0)
        PIDx->sumP -= PIDx->proportion * (nextPoint - PIDx->lastPoint);
      PIDx->output += PIDx->sumP;
      break;
    case 2:  // 自适应比例值
      PIDx->output +=
          PIDx->proportion *
          logf(PIDx->pModeK1 * fabsf(PIDx->error_0) + PIDx->pModeK2) *
          PIDx->error_0;
      break;
  }
  /* Integral */
  // 输出限幅时不积分, 但是允许通过积分退出限幅
  if (PIDx->limitFlag == 0 ||
      PIDx->limitFlag * PIDx->integral * PIDx->error_0 < 0) {
    switch (PIDx->integMode) {
      case 0:  // const
        PIDx->sumI += PIDx->integral * PIDx->error_0 * PIDx->Ts;
        break;
      case 1:  // 积分分离
        if (fabsf(PIDx->error_0) < PIDx->iModeK1)
          PIDx->sumI += PIDx->integral * PIDx->error_0 * PIDx->Ts;
        break;
      case 2:  // 变速积分
        PIDx->sumI += fmaxf(0, fminf(1, (PIDx->iModeK1 - fabsf(PIDx->error_0)) /
                                            PIDx->iModeK2)) *
                      PIDx->integral * PIDx->error_0 * PIDx->Ts;
        break;
    }
    // 积分限幅
    if (PIDx->sumILimit > 0) {
      if (PIDx->sumI > PIDx->sumILimit) {
        PIDx->sumI = PIDx->sumILimit;
      } else if (PIDx->sumI < -PIDx->sumILimit) {
        PIDx->sumI = -PIDx->sumILimit;
      }
    }
  }
  PIDx->output += PIDx->sumI;
  /* Derivative */
  if (PIDx->Ts != 0 && PIDx->lastPoint != 0)
    PIDx->output -= PIDx->derivative * (nextPoint - PIDx->lastPoint) / PIDx->Ts;

  PIDx->lastPoint = nextPoint;
  /* Output Limit */
  if (PIDx->maxOutput > PIDx->minOutput) {
    if (PIDx->output >= PIDx->maxOutput) {
      PIDx->output = PIDx->maxOutput;
      PIDx->limitFlag = 1;
    } else if (PIDx->output <= PIDx->minOutput) {
      PIDx->output = PIDx->minOutput;
      PIDx->limitFlag = -1;
    } else {
      PIDx->limitFlag = 0;
    }
  }
  return PIDx->output;
}

/**
 * @brief 快速增量式PID
 * @retval 增量输出
 * @note 该函数内联且不可重入，只能被单一控制器专用
 */
float __attribute__((always_inline))
PID_QuickInc_Calculate(float setPoint, float nextPoint) {
  const float QUICKINC_P = 1.0f, QUICKINC_I = 0.5f, QUICKINC_D = 0.5f;
  static float error_0 = 0, error_1 = 0, error_2 = 0;
  error_2 = error_1;
  error_1 = error_0;
  error_0 = setPoint - nextPoint;
  return QUICKINC_P * (error_0 - error_1) + QUICKINC_I * error_0 +
         QUICKINC_D * (error_0 - 2 * error_1 + error_2);
}

/**
 * @brief 位置式PID(调度间隔自适应)
 * @retval 输出
 * @note 暂停->恢复时，pid->lastTime需要手动置为0
 */
float PID_Calculate_TimeAdaptive(pid_tad_t *PIDx, float nextPoint) {
  int64_t time = m_time_ms();
  if (PIDx->lastTime != 0) {
    PIDx->pid.Ts = (time - PIDx->lastTime) / 1000000.0;
  } else {
    PIDx->pid.Ts = 0;  // 首次调用, 避免积分突变
  }
  PIDx->lastTime = time;
  return PID_Calculate(&PIDx->pid, nextPoint);
}

/**
 * @brief 前馈PID
 * @note 前馈环节的传递函数需根据实际情况整定，无通用模型
 */
float PID_Calculate_FeedForward(pid_t *PIDx, float nextPoint) {
  float feed = 0;
  float pid = PID_Calculate(PIDx, nextPoint);
  /****Add model here****/
  // 例子：电机转速环前馈

  /*********************/
  return pid + feed;
}

/**
 * @brief 重置PID初始值并清空PID状态
 * @param  startPoint      PID初始值, 防止PID输出突变
 */
void PID_Reset_StartPoint(pid_t *PIDx, float startPoint) {
  PID_Reset(PIDx);
  PIDx->output = startPoint;
  PIDx->base = startPoint;
}

/**
 * @brief 清空PID状态
 */
void PID_Reset(pid_t *PIDx) {
  PIDx->sumI = 0;
  PIDx->sumP = 0;
  PIDx->error_0 = 0;
  PIDx->error_1 = 0;
  PIDx->limitFlag = 0;
  PIDx->lastPoint = 0;
  PIDx->output = 0;
}

/**
 * @brief 设置PID参数
 */
void PID_Set_Tunings(pid_t *PIDx, float kp, float ki, float kd) {
  PIDx->proportion = kp;
  PIDx->derivative = kd;
  PIDx->integral = ki;
}
