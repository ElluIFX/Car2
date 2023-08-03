/**
 * @file uartPack.h
 * @author Ellu (lutaoyu@163.com)
 *
 * THINK DIFFERENTLY
 */

#ifndef __UART_PACK_H__
#define __UART_PACK_H__

#include "macro.h"
#include "mod_config.h"
#include "stdio.h"
#include "usart.h"

/********** 串口功能包设置 **********/
// 串口设置
#define _ENABLE_UART_DMA 1  // 是否开启串口DMA支持
#define _ENABLE_USB_CDC 0   // 是否开启USB CDC虚拟串口支持

#define _UART_RECV_BUFFER_SIZE 512  // 串口接收缓冲区大小
#define _UART_SEND_BUFFER_SIZE 512  // 串口发送缓冲区大小
#define _UART_PINGPONG_BUFFER 1     // 是否使用双缓冲区
#define _REWRITE_HAL_HANLDER 1  // 是否重写HAL库中的串口中断处理函数
#define _USB_CDC_TIMEOUT 5      // USB CDC发送等待超时时间(ms)
// printf重定向设置
#define _PRINT_USE_IT 1          // 是否使用中断发送
#define _PRINT_USE_DMA 1         // 是否使用DMA发送
#define _PRINT_UART_PORT huart2  // printf重定向串口
#define _PRINT_USE_CDC 0         // printf重定向到USB CDC
// VOFA+
#define _ENABLE_VOFA 1        // 是否开启VOFA相关函数
#define _VOFA_BUFFER_SIZE 32  // VOFA缓冲区大小
/********** ********** **********/

#if _PRINT_USE_CDC && _ENABLE_USB_CDC
#define printf(...) printcdc(__VA_ARGS__)
#define printf_flush() printcdc_flush()
#undef putchar
#define putchar(c) CDC_Putchar(c)
#undef puts
#define puts(s) CDC_Puts(s)
#else
#undef printf
#define printf(...) printft(&_PRINT_UART_PORT, __VA_ARGS__)
#define printf_flush() printft_flush(&_PRINT_UART_PORT)
#if 1  // 重定向putchar, getchar, puts, gets
#undef putchar
#define putchar(c) Uart_Putchar(&_PRINT_UART_PORT, c)
#undef getchar
#define getchar() Uart_Getchar(&_PRINT_UART_PORT)
#undef puts
#define puts(s) Uart_Puts(&_PRINT_UART_PORT, s)
#undef gets
#define gets(s) Uart_Gets(&_PRINT_UART_PORT, s)
#endif
#endif  // _PRINT_USE_CDC && _ENABLE_USB_CDC

// typedef
typedef struct {                           // 超时UART控制结构体
  uint8_t rxBuf[_UART_RECV_BUFFER_SIZE];   // 接收缓冲区
  uint8_t buffer[_UART_RECV_BUFFER_SIZE];  // 接收保存缓冲区
  __IO uint8_t finished;                   // 接收完成标志位
  __IO uint16_t len;                       // 接收保存区长度
  uint16_t rxIdx;                          // 接收缓冲区索引
  m_time_t rxTime;                         // 接收超时计时器
  m_time_t rxTimeout;                      // 接收超时时间
  UART_HandleTypeDef *huart;               // 串口句柄
  void (*rxCallback)(char *, uint16_t);    // 接收完成回调函数
  uint8_t cbkInIRQ;  // 回调函数是否在中断中执行
} uart_ctrl_t;

#if _ENABLE_UART_DMA
typedef struct {                           // DMA UART控制结构体
  uint8_t rxBuf[_UART_RECV_BUFFER_SIZE];   // 接收缓冲区
  uint8_t buffer[_UART_RECV_BUFFER_SIZE];  // 接收保存缓冲区
  __IO uint8_t finished;                   // 接收完成标志位
  __IO uint16_t len;                       // 接收保存区长度
  UART_HandleTypeDef *huart;               // 串口句柄
  void (*rxCallback)(char *, uint16_t);    // 接收完成回调函数
  uint8_t cbkInIRQ;  // 回调函数是否在中断中执行
} uart_dma_ctrl_t;
#endif

#if _ENABLE_USB_CDC
#include "usbd_cdc_if.h"
typedef struct {                         // CDC型UART控制结构体
  uint8_t buffer[APP_RX_DATA_SIZE];      // 接收保存缓冲区
  __IO uint8_t finished;                 // 接收完成标志位
  __IO uint16_t len;                     // 接收保存区计数器
  void (*rxCallback)(char *, uint16_t);  // 接收完成回调函数
  uint8_t cbkInIRQ;                      // 回调函数是否在中断中执行
} usb_cdc_ctrl_t;

// USB CDC 串口接收
extern usb_cdc_ctrl_t usb_cdc;
#endif

#define RX_DONE(uart_t) uart_t.finished
#define RX_DATA(uart_t) ((char *)uart_t.buffer)
#define RX_LEN(uart_t) uart_t.len
#define RX_CLEAR(uart_t) (uart_t.finished = 0)

/**
 * @brief 串口错误状态
 * @note 1:奇偶校验错误 2:帧错误 3:噪声错误 4:接收溢出
 */
extern uint8_t uart_error_state;

// public functions

extern void Assert_Failed_Handler(char *file, uint32_t line);

extern int printft(UART_HandleTypeDef *huart, char *fmt, ...);
extern void printft_flush(UART_HandleTypeDef *huart);
extern void print_hex(const char *text, uint8_t *data, uint16_t len);
extern void Uart_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);
extern void Uart_Putchar(UART_HandleTypeDef *huart, uint8_t data);
extern void Uart_Puts(UART_HandleTypeDef *huart, char *str);
extern int Uart_Getchar(UART_HandleTypeDef *huart);
extern char *Uart_Gets(UART_HandleTypeDef *huart, char *str);
extern void Uart_SendFast(UART_HandleTypeDef *huart, uint8_t *data,
                          uint16_t len);
extern void Uart_Init(uart_ctrl_t *ctrl, UART_HandleTypeDef *huart,
                      m_time_t rxTimeout,
                      void (*rxCallback)(char *data, uint16_t len),
                      uint8_t cbkInIRQ);
extern void Uart_Process(UART_HandleTypeDef *huart);
extern void Uart_Timeout_Check(void);
extern void Uart_Callback_Check(void);

#if _ENABLE_UART_DMA
extern void Uart_DMA_Init(uart_dma_ctrl_t *ctrl, UART_HandleTypeDef *huart,
                          void (*rxCallback)(char *data, uint16_t len),
                          uint8_t cbkInIRQ);
extern void Uart_DMA_Process(UART_HandleTypeDef *huart, uint16_t Size);
#endif  // _ENABLE_UART_DMA

#if _ENABLE_USB_CDC
extern int printcdc(char *fmt, ...);
extern void printcdc_flush(void);
extern void CDC_Send(uint8_t *buf, uint16_t len);
extern void CDC_Wait_Connect(int timeout_ms);
extern void CDC_Register_Callback(void (*callback)(char *buf, uint16_t len),
                                  uint8_t cbkInIRQ);
extern void CDC_Putchar(uint8_t data);
extern void CDC_Puts(char *data);
extern uint8_t USB_Connected(void);
#endif  // _ENABLE_USB_CDC

#if _ENABLE_VOFA
extern void Vofa_Add(float value);
extern void Vofa_AddSeq(float *value, uint8_t len);
extern void Vofa_Clear(void);
extern void Vofa_Send(UART_HandleTypeDef *huart);
extern void Vofa_SendFast(UART_HandleTypeDef *huart);
extern void Vofa_SendCDC(void);
#endif  // _ENABLE_VOFA

#endif  // __UART_H
