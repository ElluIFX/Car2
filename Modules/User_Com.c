/**
 * @file User_Com.c
 * @brief 用户下位机通信模块
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-08
 *
 * THINK DIFFERENTLY
 */

#include "User_Com.h"

#include <stdarg.h>
#include <stdio.h>

#include "Wireless_Com.h"
#include "log.h"
#include "motor.h"
#include "string.h"

void UserCom_SendData(u8* dataToSend, u8 Length);
void UserCom_CheckAck();
void UserCom_SendAck(uint8_t option, uint8_t* data_p, uint8_t data_len);

u8 user_connected = 0;              // 用户下位机是否连接
static u16 user_heartbeat_cnt = 0;  // 用户下位机心跳计数
static uint8_t user_ack_buf[32];    // ACK数据
static uint16_t user_ack_cnt = 0;   // ACK计数
uint8_t user_data_temp[280];        // 数据接受缓存
uint8_t data_to_send[280];
user_data_t user_data = {0};

/***接收****/
static u8 _user_data_cnt = 0;  // 数据计数
static u8 _data_len = 0;       // 数据长度
static u8 state = 0;           // 状态机
/**
 * @brief 用户协议数据获取,在串口中断中调用,解析完成后调用UserCom_DataAnl
 * @param  data             数据
 */
void UserCom_GetOneByte(u8 data) {
  if (state == 0 && data == 0xAA) {
    state = 1;
    user_data_temp[0] = data;
  } else if (state == 1 && data == 0x22) {
    state = 2;
    user_data_temp[1] = data;
  } else if (state == 2)  // 功能字
  {
    state = 3;
    user_data_temp[2] = data;
  } else if (state == 3)  // 长度
  {
    state = 4;
    user_data_temp[3] = data;
    _data_len = data;  // 数据长度
    _user_data_cnt = 0;
  } else if (state == 4 && _data_len > 0) {
    _data_len--;
    user_data_temp[4 + _user_data_cnt++] = data;  // 数据
    if (_data_len == 0) state = 5;
  } else if (state == 5) {
    state = 0;
    user_data_temp[4 + _user_data_cnt] = data;  // check sum
    user_data_temp[5 + _user_data_cnt] = 0;
    UserCom_DataAnl(user_data_temp, 4 + _user_data_cnt);
  } else
    state = 0;
}

void UserCom_GetBuffer(u8* data_buf, u32 len) {
  for (u32 i = 0; i < len; i++) {
    if (state == 0 && data_buf[i] == 0xAA) {
      state = 1;
      user_data_temp[0] = data_buf[i];
    } else if (state == 1 && data_buf[i] == 0x22) {
      state = 2;
      user_data_temp[1] = data_buf[i];
    } else if (state == 2)  // 功能字
    {
      state = 3;
      user_data_temp[2] = data_buf[i];
    } else if (state == 3)  // 长度
    {
      state = 4;
      user_data_temp[3] = data_buf[i];
      _data_len = data_buf[i];  // 数据长度
      _user_data_cnt = 0;
    } else if (state == 4 && _data_len > 0) {
      for (; i < len; i++) {
        _data_len--;
        user_data_temp[4 + _user_data_cnt++] = data_buf[i];  // 数据
        if (_data_len == 0) {
          state = 5;
          break;
        }
      }
    } else if (state == 5) {
      state = 0;
      user_data_temp[4 + _user_data_cnt] = data_buf[i];  // check sum
      user_data_temp[5 + _user_data_cnt] = 0;
      UserCom_DataAnl(user_data_temp, 4 + _user_data_cnt);
    } else
      state = 0;
  }
}

extern motor_t motor_l;
extern motor_t motor_r;

/**
 * @brief 用户命令解析执行,数据接收完成后自动调用
 * @param  data_buf         数据缓存
 * @param  data_len         数据长度
 */
void UserCom_DataAnl(u8* data_buf, u16 data_len) {
  s8* p_s8;
  s16* p_s16;
  s32* p_s32;
  u32* p_u32;
  float* p_float;
  u8 u8_temp;
  u32 u32_temp;
  u8* p_data = (uint8_t*)(data_buf + 4);
  u8 option = data_buf[2] & 0b01111111;  // 功能字(最高位为1表示需要ACK)
  u8 send_ack = data_buf[2] & 0b10000000;
  u8 len = data_buf[3];
  u8 calc_check = 0;
  for (u8 i = 0; i < len + 4; i++) {
    calc_check += data_buf[i];
  }
  if (calc_check != data_buf[data_len]) {
    return;
  }
  switch (option) {
    case 0x00:  // 心跳包
      if (p_data[0] == 0x01) {
        if (!user_connected) {
          user_connected = 1;
        }
        user_heartbeat_cnt = 0;
        return;
      }
    case 0x01:  // 转发到无线串口
      Send_To_Wireless(p_data, len);
      break;
    case 0x02:  // 设置激光
      HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, p_data[0]);
      break;
    case 0x03:  // 设置电机模式
      if (p_data[0] & 0x01) motor_l.mode = p_data[1];
      if (p_data[0] & 0x02) motor_r.mode = p_data[1];
    case 0x04:  // 设置电机pwm
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) motor_l.pwmDuty = *p_float;
      if (p_data[0] & 0x02) motor_r.pwmDuty = *p_float;
      break;
    case 0x05:  // 设置电机目标速度
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) motor_l.spdPID.setPoint = *p_float;
      if (p_data[0] & 0x02) motor_r.spdPID.setPoint = *p_float;
      break;
    case 0x06:  // 设置电机目标速度
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) MOTOR_SET_SPEED_MPS(motor_l, *p_float);
      if (p_data[0] & 0x02) MOTOR_SET_SPEED_MPS(motor_r, *p_float);
      break;
    case 0x07:
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) motor_l.posTargetSpd = *p_float;
      if (p_data[0] & 0x02) motor_r.posTargetSpd = *p_float;
      break;
    case 0x08:
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) MOTOR_SET_POS_SPEED_MPS(motor_l, *p_float);
      if (p_data[0] & 0x02) MOTOR_SET_POS_SPEED_MPS(motor_r, *p_float);
      break;
    case 0x09:
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) MOTOR_SET_DEGREE(motor_l, *p_float);
      if (p_data[0] & 0x02) MOTOR_SET_DEGREE(motor_r, *p_float);
      break;
    case 0x0A:
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) MOTOR_GO_DEGREE(motor_l, *p_float);
      if (p_data[0] & 0x02) MOTOR_GO_DEGREE(motor_r, *p_float);
      break;
    case 0x0B:
      p_float = (float*)(p_data + 1);
      if (p_data[0] & 0x01) MOTOR_GO_METER(motor_l, *p_float);
      if (p_data[0] & 0x02) MOTOR_GO_METER(motor_r, *p_float);
      break;
    case 0x0C:
      if (p_data[0] & 0x01) MOTOR_RESET(motor_l);
      if (p_data[0] & 0x02) MOTOR_RESET(motor_r);
      break;
    case 0x0D:  // 舵机 (TIM2 - 50Hz - 自动重装20000-1)
      p_u32 = (uint32_t*)(p_data + 1);
      u32_temp = (*p_u32);
      if (u32_temp > 20000) u32_temp = 20000;
      if (p_data[0] & 0x01)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, u32_temp);
      if (p_data[0] & 0x02)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, u32_temp);
      if (p_data[0] & 0x04)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, u32_temp);
      if (p_data[0] & 0x08)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, u32_temp);
    case 0x0E:
      p_u32 = (uint32_t*)(p_data);
      u32_temp = (*p_u32);
      if (u32_temp > 20000) u32_temp = 20000;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, u32_temp);
      p_u32++;
      u32_temp = (*p_u32);
      if (u32_temp > 20000) u32_temp = 20000;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, u32_temp);
    default:
      break;
  }
  if (send_ack) {
    UserCom_SendAck(option, p_data, len);
  }
}

void UserCom_SendAck(uint8_t option, uint8_t* data_p, uint8_t data_len) {
  uint8_t ack_data;
  if (user_ack_cnt >= 32) return;
  ack_data = option;
  for (uint8_t i = 0; i < data_len; i++) {
    ack_data += data_p[i];
  }
  user_ack_buf[user_ack_cnt] = ack_data;
  user_ack_cnt++;
}

uint16_t data_exchange_cnt = 0;
/**
 * @brief 用户通讯持续性任务，在调度器中调用
 */
void UserCom_Task(void) {  // 1ms
  if (!user_connected) return;
  // 心跳超时检查
  user_heartbeat_cnt++;
  if (user_heartbeat_cnt >= 1000) {  // 1s
    user_connected = 0;
    return;
  }

  // ACK发送检查
  if (user_ack_cnt) UserCom_CheckAck();

  // 数据交换
  data_exchange_cnt++;
  if (data_exchange_cnt >= 25) {  // 25ms
    data_exchange_cnt = 0;
    UserCom_DataExchange();
  }
}

/**
 * @brief 交换飞控数据
 */
void UserCom_DataExchange(void) {
  const uint8_t size = sizeof(user_data_t);

  user_data.motor_l_speed = motor_l.speed;
  user_data.motor_l_speed_mps = MOTOR_GET_SPEED_MPS(motor_l);
  user_data.motor_l_pos = motor_l.pos;
  user_data.motor_l_degree = MOTOR_GET_DEGREE(motor_l);

  user_data.motor_r_speed = motor_r.speed;
  user_data.motor_r_speed_mps = MOTOR_GET_SPEED_MPS(motor_r);
  user_data.motor_r_pos = motor_r.pos;
  user_data.motor_r_degree = MOTOR_GET_DEGREE(motor_r);

  // 初始化数据
  data_to_send[0] = 0xAA;
  data_to_send[1] = 0x55;
  data_to_send[2] = size + 1;
  data_to_send[3] = 0x01;

  // 数据赋值
  for (u8 i = 0; i < size; i++) {
    data_to_send[4 + i] = ((u8*)(&user_data))[i];
  }

  // 校验和
  data_to_send[4 + size] = 0;
  for (u8 i = 0; i < 4 + size; i++) {
    data_to_send[4 + size] += data_to_send[i];
  }

  UserCom_SendData(data_to_send, 5 + size);
}

/**
 * @brief 检查ACK队列并发送
 */
void UserCom_CheckAck() {
  while (user_ack_cnt) {
    user_ack_cnt--;
    data_to_send[0] = 0xAA;                        // head1
    data_to_send[1] = 0x55;                        // head2
    data_to_send[2] = 0x02;                        // length
    data_to_send[3] = 0x02;                        // cmd
    data_to_send[4] = user_ack_buf[user_ack_cnt];  // data
    data_to_send[5] = 0;                           // check_sum
    for (uint8_t i = 0; i < 5; i++) {
      data_to_send[5] += data_to_send[i];
    }
    UserCom_SendData(data_to_send, 6);
  }
}

/**
 * @brief 发送事件
 * @param  event            事件代码
 * @param  op               操作代码
 */
void UserCom_SendEvent(u8 event, u8 op) {
  data_to_send[0] = 0xAA;   // head1
  data_to_send[1] = 0x55;   // head2
  data_to_send[2] = 0x03;   // length
  data_to_send[3] = 0x03;   // cmd
  data_to_send[4] = event;  // event code
  data_to_send[5] = op;     // op code
  data_to_send[6] = 0;      // check_sum
  for (u8 i = 0; i < 6; i++) {
    data_to_send[6] += data_to_send[i];
  }
  UserCom_SendData(data_to_send, 7);
}

/**
 * @brief 用户通讯数据发送
 */
void UserCom_SendData(u8* dataToSend, u8 Length) {
  Uart_Send_Buffered(&huart2, dataToSend, Length);
}

#define STRLENMAX 100

static u8 strBuf[STRLENMAX + 5];
static u8 strBufPrintf[STRLENMAX];

/**
 * @brief 发送LOG到用户端
 * @param  str             字符串
 */
void UserStringSend(char* str) {
  u8 len = strlen(str);
  strBuf[0] = 0xAA;     // head1
  strBuf[1] = 0x55;     // head2
  strBuf[2] = len + 1;  // length
  strBuf[3] = 0x06;     // cmd
  if (len > STRLENMAX) len = STRLENMAX;
  memcpy(strBuf + 4, str, len);
  strBuf[4 + len] = 0;  // check_sum
  for (u8 j = 0; j < 4 + len; j++) {
    strBuf[4 + len] += strBuf[j];
  }
  UserCom_SendData(strBuf, 5 + len);
}

/**
 * @brief 发送LOG到用户端
 */
void UserPrintf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf((char*)strBufPrintf, STRLENMAX, fmt, ap);
  va_end(ap);
  UserStringSend((char*)strBufPrintf);
}
