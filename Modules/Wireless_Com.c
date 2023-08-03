/**
 * @file Wireless_Com.c
 * @brief 无线通信
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-08-02
 *
 * THINK DIFFERENTLY
 */

#include "Wireless_Com.h"

#include "log.h"
#include "string.h"
uint8_t wl_send_temp[256];

void Send_To_Wireless(u8* data, u8 len) {
  wl_send_temp[0] = 0xBB;
  wl_send_temp[1] = 0x33;
  wl_send_temp[2] = len;
  memcpy(&wl_send_temp[3], data, len);
  Uart_Send(&huart1, wl_send_temp, len + 3);
}

uint8_t wl_back_temp[256];
void Wireless_DataAnl(u8* data, u8 len) {
  wl_back_temp[0] = 0xAA;
  wl_back_temp[1] = 0x55;
  wl_back_temp[2] = len + 1;
  wl_back_temp[3] = 0x07;
  memcpy(&wl_back_temp[4], data, len);
  wl_back_temp[4 + len] = 0;
  for (u8 i = 0; i < len + 4; i++) {
    wl_back_temp[4 + len] += wl_back_temp[i]; // 校验和
  }
    Uart_Send(&huart2, wl_back_temp, len + 5);
}

uint8_t wl_data_temp[256];   // 数据接受缓存
static u8 _wl_data_cnt = 0;  // 数据计数
static u8 _wl_data_len = 0;  // 数据长度
static u8 _wl_state = 0;     // 状态机
void Uart_Wireless_GetOneByte(u8 data) {
  if (_wl_state == 0 && data == 0xBB) {
    _wl_state = 1;
  } else if (_wl_state == 1 && data == 0x33) {
    _wl_state = 2;
  } else if (_wl_state == 2)  // 长度
  {
    _wl_state = 3;
    _wl_data_len = data;  // 数据长度
    _wl_data_cnt = 0;
  } else if (_wl_state == 3 && _wl_data_len > 0) {
    _wl_data_len--;
    wl_data_temp[_wl_data_cnt++] = data;  // 数据
    if (_wl_data_len == 0) {
      Wireless_DataAnl(wl_data_temp, _wl_data_cnt);
    }
  } else
    _wl_state = 0;
}
