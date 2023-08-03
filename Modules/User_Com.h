/**
 * @file User_Com.h
 * @brief see User_Com.c
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2022-06-08
 *
 * THINK DIFFERENTLY
 */

#ifndef _USER_COM_H
#define _USER_COM_H

#include "macro.h"
#include "main.h"

// 事件代码
#define USER_EVENT_KEY_SHORT 0x01
#define USER_EVENT_KEY_LONG 0x02
#define USER_EVENT_KEY_DOUBLE 0x03
// 事件操作
#define USER_EVENT_OP_SET 0x01
#define USER_EVENT_OP_CLEAR 0x02

extern s16 user_pwm[4];
extern u8 user_connected;
typedef struct {
  float motor_l_speed;
  float motor_l_speed_mps;
  int32_t motor_l_pos;
  float motor_l_degree;
  float motor_r_speed;
  float motor_r_speed_mps;
  int32_t motor_r_pos;
  float motor_r_degree;
} __attribute__((packed)) user_data_t;

extern user_data_t user_data;

void UserCom_GetOneByte(u8 data);

void UserCom_GetBuffer(u8* data_buf, u32 len);

void UserCom_DataAnl(u8* data_buf, u16 data_len);

void UserCom_Task(void);

void UserCom_SendEvent(u8 event, u8 op);

void UserStringSend(char* str);

void UserPrintf(const char* fmt, ...);

void UserCom_DataExchange(void);
#endif
