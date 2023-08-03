/**
 * @file Wireless_Com.h
 * @brief see Wireless_Com.c
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-08-02
 *
 * THINK DIFFERENTLY
 */

#ifndef _WIRELESS_COM_H
#define _WIRELESS_COM_H
#include "main.h"
#include "macro.h"

void Uart_Wireless_GetOneByte(u8 data);

void Send_To_Wireless(u8* data, u8 len);

#endif
