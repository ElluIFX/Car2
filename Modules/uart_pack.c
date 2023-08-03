/**
 * @file uart_pack.c
 * @brief
 * 完善的HAL库串口封装包，包括日志、CDC、DMA发送接收、带超时的中断发送接收等功能
 * @author Ellu (lutaoyu@163.com)
 * @version 3.0
 * @date 2021-12-19
 *
 * THINK DIFFERENTLY
 */

#include "uart_pack.h"

#include <stdlib.h>

#include "stdarg.h"
#include "string.h"

#if _PRINT_USE_DMA
#define _UART_NOT_READY                     \
  (huart->gState != HAL_UART_STATE_READY || \
   huart->hdmatx->State != HAL_DMA_STATE_READY)
#else
#define _UART_NOT_READY huart->gState != HAL_UART_STATE_READY
#endif

char *get_buffer(void) {
#if _UART_PINGPONG_BUFFER
  static char sendBuff1[_UART_SEND_BUFFER_SIZE];  // 发送缓冲区1
  static char sendBuff2[_UART_SEND_BUFFER_SIZE];  // 发送缓冲区2
  static uint8_t bufSelect = 0;                   // 缓冲区选择
  if (bufSelect == 0) {
    bufSelect = 1;
    return sendBuff1;
  } else {
    bufSelect = 0;
    return sendBuff2;
  }
#else
  static char sendBuff[_UART_SEND_BUFFER_SIZE];  // 发送缓冲区
  while (_UART_NOT_READY) {                      // 检查串口是否打开
    __NOP();
  }
  return sendBuff;
#endif
}

/**
 * @brief 向指定串口发送格式化字符串
 * @param  huart            目标串口
 * @param  fmt              类似printf的格式化字符串
 * @retval 发送的字节数
 */
int printft(UART_HandleTypeDef *huart, char *fmt, ...) {
  char *sendBuffP = get_buffer();
  va_list ap;         // typedef char *va_list
  va_start(ap, fmt);  // 找到第一个可变形参的地址赋给ap
  int sendLen = vsnprintf(sendBuffP, _UART_SEND_BUFFER_SIZE, fmt, ap);
  va_end(ap);
  Uart_Send(huart, (uint8_t *)sendBuffP, sendLen);
  return sendLen;
}

/**
 * @brief 等待串口发送完成
 */
void printft_flush(UART_HandleTypeDef *huart) {
  while (_UART_NOT_READY) {
    __NOP();
  }
}

/**
 * @brief 打印十六进制数组
 */
void print_hex(const char *text, uint8_t *data, uint16_t len) {
  uint16_t i;
  printf("%s [ ", text);
  for (i = 0; i < len; i++) {
    printf("%02X ", data[i]);
  }
  printf("]\r\n");
}

/**
 * @brief 串口发送数据
 * @param  huart            目标串口
 * @param  data             数据指针
 * @param  len              数据长度
 */
void Uart_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len) {
  if (len > 0) {
    while (_UART_NOT_READY) {  // 检查串口是否打开
      __NOP();
    }
#if _PRINT_USE_DMA & _ENABLE_UART_DMA
    HAL_UART_Transmit_DMA(huart, data, len);
#elif _PRINT_USE_IT
    HAL_UART_Transmit_IT(huart, data, len);
#else
    HAL_UART_Transmit(huart, data, len, 0xFFFF);
#endif
  }
}

void Uart_Putchar(UART_HandleTypeDef *huart, uint8_t data) {
  while (_UART_NOT_READY) {  // 检查串口是否打开
    __NOP();
  }
  HAL_UART_Transmit(huart, &data, 1, 0xFFFF);
}

void Uart_Puts(UART_HandleTypeDef *huart, char *str) {
  while (*str) {
    Uart_Putchar(huart, *str++);
  }
}

int Uart_Getchar(UART_HandleTypeDef *huart) {
  static uint8_t data;
  if (HAL_UART_Receive(huart, &data, 1, 0xFFFF) != HAL_OK) {
    return EOF;
  }
  return data;
}

char *Uart_Gets(UART_HandleTypeDef *huart, char *str) {
  char *p = str;
  while (1) {
    int c = Uart_Getchar(huart);
    if (c == EOF) {
      return NULL;
    }
    *p++ = c;
    if (c == '\n') {
      break;
    }
  }
  *p = '\0';
  return str;
}

/**
 * @brief 串口发送数据，阻塞时不等待
 */
void Uart_SendFast(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len) {
  if (len > 0) {
    if (_UART_NOT_READY) return;
#if _PRINT_USE_DMA & _ENABLE_UART_DMA
    HAL_UART_Transmit_DMA(huart, data, len);
#elif _PRINT_USE_IT
    HAL_UART_Transmit_IT(huart, data, len);
#else
    HAL_UART_Transmit(huart, data, len, 0xFFFF);
#endif
  }
}

#if !_UART_CALLBACK_IN_IRQ
void (*swRxCallback)(char *data, uint16_t len) = NULL;
char *swRxData = NULL;
uint16_t swRxLen = 0;

/**
 * @brief 轮询以在主循环中响应串口接收完成回调
 * @note 若轮询频率小于接收频率, 回调请求会被覆盖
 */
void Uart_Callback_Check(void) {
  if (swRxCallback != NULL) {
    swRxCallback(swRxData, swRxLen);
    swRxCallback = NULL;
  }
}
#endif

typedef struct {
  uart_ctrl_t *ctrl;
  void *next;
} uart_ctrl_list_t;

static uart_ctrl_list_t *uart_ctrl_list = NULL;

/**
 * @brief 串口中断接收初始化
 * @param  ctrl             结构体指针
 * @param  huart            目标串口
 * @param  rxTimeout        接收超时时间
 * @param  rxCallback       接收完成回调函数
 * @param  cbkInIRQ         回调函数是否在中断中执行
 */
void Uart_Init(uart_ctrl_t *ctrl, UART_HandleTypeDef *huart, m_time_t rxTimeout,
               void (*rxCallback)(char *data, uint16_t len), uint8_t cbkInIRQ) {
  ctrl->rxTimeout = rxTimeout;
  ctrl->finished = 0;
  ctrl->rxIdx = 0;
  ctrl->len = 0;
  ctrl->huart = huart;
  ctrl->rxCallback = rxCallback;
  ctrl->cbkInIRQ = cbkInIRQ;
  HAL_UART_Receive_IT(huart, ctrl->rxBuf, 1);
  // add to list
  uart_ctrl_list_t *item = (uart_ctrl_list_t *)malloc(sizeof(uart_ctrl_list_t));
  item->ctrl = ctrl;
  item->next = uart_ctrl_list;
  uart_ctrl_list = item;
}

/**
 * @brief 串口中断接收处理，在函数HAL_UART_RxCpltCallback中调用
 */
void Uart_Process(UART_HandleTypeDef *huart) {
  static uart_ctrl_list_t *item;
  item = uart_ctrl_list;
  while (item) {
    if (item->ctrl->huart == huart) {
      item->ctrl->rxTime = m_time_ms();
      if (++item->ctrl->rxIdx >= _UART_RECV_BUFFER_SIZE - 1) {
        memcpy(item->ctrl->buffer, item->ctrl->rxBuf, item->ctrl->rxIdx);
        item->ctrl->len = item->ctrl->rxIdx;
        item->ctrl->buffer[item->ctrl->rxIdx] = 0;
        item->ctrl->finished = 1;
        item->ctrl->rxIdx = 0;
        if (item->ctrl->rxCallback) {
          if (item->ctrl->cbkInIRQ) {
            item->ctrl->rxCallback((char *)item->ctrl->buffer, item->ctrl->len);
          } else {
            swRxCallback = item->ctrl->rxCallback;
            swRxData = (char *)item->ctrl->buffer;
            swRxLen = item->ctrl->len;
          }
        }
      }
      HAL_UART_Receive_IT(item->ctrl->huart,
                          item->ctrl->rxBuf + item->ctrl->rxIdx, 1);
      return;
    }
    item = item->next;
  }
}

/**
 * @brief 串口中断接收超时判断，在调度器中调用
 */
void Uart_Timeout_Check(void) {
  static uart_ctrl_list_t *item;
  item = uart_ctrl_list;
  while (item) {
    if (item->ctrl->rxIdx &&
        m_time_ms() - item->ctrl->rxTime > item->ctrl->rxTimeout) {
      HAL_UART_AbortReceive_IT(item->ctrl->huart);
      memcpy(item->ctrl->buffer, item->ctrl->rxBuf, item->ctrl->rxIdx);
      item->ctrl->len = item->ctrl->rxIdx;
      item->ctrl->buffer[item->ctrl->rxIdx] = 0;
      item->ctrl->finished = 1;
      item->ctrl->rxIdx = 0;
      HAL_UART_Receive_IT(item->ctrl->huart, item->ctrl->rxBuf, 1);
      if (item->ctrl->rxCallback) {
        item->ctrl->rxCallback((char *)item->ctrl->buffer, item->ctrl->len);
      }
    }
    return;
    item = item->next;
  }
}

#if _REWRITE_HAL_HANLDER
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { Uart_Process(huart); }
#endif

#if _ENABLE_UART_DMA

typedef struct {
  uart_dma_ctrl_t *ctrl;
  void *next;
} dma_ctrl_list_t;

dma_ctrl_list_t *dma_ctrl_list = NULL;

/**
 * @brief 串口DMA接收初始化
 * @param  ctrl             结构体指针
 * @param  huart            目标串口
 * @param  rxCallback       接收完成回调函数
 * @param  cbkInIRQ         回调函数是否在中断中执行
 */
void Uart_DMA_Init(uart_dma_ctrl_t *ctrl, UART_HandleTypeDef *huart,
                   void (*rxCallback)(char *data, uint16_t len),
                   uint8_t cbkInIRQ) {
  ctrl->huart = huart;
  ctrl->finished = 0;
  ctrl->len = 0;
  ctrl->buffer[0] = 0;
  ctrl->rxCallback = rxCallback;
  ctrl->cbkInIRQ = cbkInIRQ;
  HAL_UARTEx_ReceiveToIdle_DMA(huart, ctrl->rxBuf, _UART_RECV_BUFFER_SIZE - 1);
  dma_ctrl_list_t *new_ctrl =
      (dma_ctrl_list_t *)malloc(sizeof(dma_ctrl_list_t));
  new_ctrl->ctrl = ctrl;
  new_ctrl->next = dma_ctrl_list;
  dma_ctrl_list = new_ctrl;
}

/**
 * @brief 串口DMA接收处理，在函数HAL_UARTEx_RxEventCallback中调用
 */
void Uart_DMA_Process(UART_HandleTypeDef *huart, uint16_t Size) {
  static dma_ctrl_list_t *item;
  item = dma_ctrl_list;
  while (item) {
    if (item->ctrl->huart == huart) {
      memcpy(item->ctrl->buffer, item->ctrl->rxBuf, Size);
      item->ctrl->len = Size;
      item->ctrl->buffer[Size] = 0;
      item->ctrl->finished = 1;
      HAL_UARTEx_ReceiveToIdle_DMA(item->ctrl->huart, item->ctrl->rxBuf,
                                   _UART_RECV_BUFFER_SIZE - 1);
      if (item->ctrl->rxCallback) {
        if (item->ctrl->cbkInIRQ) {
          item->ctrl->rxCallback((char *)item->ctrl->buffer, item->ctrl->len);
        } else {
          swRxCallback = item->ctrl->rxCallback;
          swRxData = (char *)item->ctrl->buffer;
          swRxLen = item->ctrl->len;
        }
      }
      return;
    }
    item = item->next;
  }
}

#if _REWRITE_HAL_HANLDER
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  Uart_DMA_Process(huart, Size);
}
#endif

#endif

uint8_t uart_error_state = 0;  // 串口错误状态

// 错误处理
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_PE)) != RESET) {  // 奇偶校验错误
    __HAL_UNLOCK(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    uart_error_state = 1;
  }
  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) != RESET) {  // 帧错误
    __HAL_UNLOCK(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    uart_error_state = 2;
  }

  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) != RESET) {  // 噪声错误
    __HAL_UNLOCK(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    uart_error_state = 3;
  }

  if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) != RESET) {  // 接收溢出
    __HAL_UNLOCK(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    uart_error_state = 4;
  }
  // 自动重启 DMA
#if _ENABLE_UART_DMA
  dma_ctrl_list_t *item = dma_ctrl_list;
  while (item != NULL) {
    if (item->ctrl->huart->Instance == huart->Instance) {
      HAL_UART_DMAStop(item->ctrl->huart);
      HAL_UARTEx_ReceiveToIdle_DMA(item->ctrl->huart, item->ctrl->rxBuf,
                                   _UART_RECV_BUFFER_SIZE - 1);
      // break;
    }
    item = item->next;
  }
#endif
  // 自动重启中断
  uart_ctrl_list_t *item2 = uart_ctrl_list;
  while (item2 != NULL) {
    if (item2->ctrl->huart->Instance == huart->Instance) {
      HAL_UART_AbortReceive_IT(item2->ctrl->huart);
      HAL_UART_Receive_IT(item2->ctrl->huart, item2->ctrl->rxBuf, 1);
      // break;
    }
    item2 = item2->next;
  }
}

__weak void Assert_Failed_Handler(char *file, uint32_t line) {}

#if _ENABLE_USB_CDC
usb_cdc_ctrl_t usb_cdc;  // USB CDC 接收控制器
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
__IO static USBD_CDC_HandleTypeDef *hcdc = NULL;

static int8_t Hook_CDC_Init_FS(void) {
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  return (USBD_OK);
}

static int8_t Hook_CDC_DeInit_FS(void) { return (USBD_OK); }

static int8_t Hook_CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
  // detect baudrate change
  return (USBD_OK);
}

static int8_t Hook_CDC_Receive_FS(uint8_t *Buf, uint32_t *Len) {
  memcpy(usb_cdc.buffer, Buf, *Len);
  usb_cdc.len = *Len;
  usb_cdc.buffer[*Len] = 0;
  usb_cdc.finished = 1;
  if (usb_cdc.cbkInIRQ) {
    usb_cdc.rxCallback((char *)usb_cdc.buffer, usb_cdc.len);
  } else {
    swRxCallback = usb_cdc.rxCallback;
    swRxData = (char *)usb_cdc.buffer;
    swRxLen = usb_cdc.len;
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

static int8_t Hook_CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len,
                                       uint8_t epnum) {
  return USBD_OK;
}

__attribute__((constructor(255))) // 自动Hook
void Hook_CDC_Init(void) {
  // hook USBD_Interface_fops_FS
  USBD_Interface_fops_FS.Init = Hook_CDC_Init_FS;
  USBD_Interface_fops_FS.DeInit = Hook_CDC_DeInit_FS;
  USBD_Interface_fops_FS.Control = Hook_CDC_Control_FS;
  USBD_Interface_fops_FS.Receive = Hook_CDC_Receive_FS;
#if 1  // TransmitCplt 只在部分MCU上支持
  USBD_Interface_fops_FS.TransmitCplt = Hook_CDC_TransmitCplt_FS;
#endif
}

int32_t _cdc_start_time = 0;
/**
 * @brief USB CDC 发送格式化字符串
 */
int printcdc(char *fmt, ...) {
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return -1;
  _cdc_start_time = m_time_ms();
  while (hcdc->TxState != 0) {  // 检查上一次发送是否完成
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return -1;
    if (m_time_ms() - _cdc_start_time > _USB_CDC_TIMEOUT) return -1;
  }
  va_list ap;
  va_start(ap, fmt);
  int sendLen = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, fmt, ap);
  va_end(ap);
  if (sendLen > 0) {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, sendLen);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  }
  return sendLen;
}

/**
 * @brief USB CDC 等待发送完成
 */
void printcdc_flush(void) {
  _cdc_start_time = m_time_ms();
  while (hcdc->TxState != 0) {
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
    if (m_time_ms() - _cdc_start_time > _USB_CDC_TIMEOUT) return;
  }
}

/**
 * @brief USB CDC 发送数据
 */
void CDC_Send(uint8_t *buf, uint16_t len) {
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
  _cdc_start_time = m_time_ms();
  while (hcdc->TxState != 0) {  // 检查上一次发送是否完成
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
    if (m_time_ms() - _cdc_start_time > _USB_CDC_TIMEOUT) return;
  }
  // memcpy(UserTxBufferFS, buf, len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buf, len);
  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

char _cbuf[] = "\r\n";
/**
 * @brief USB CDC 阻塞等待连接
 */
void CDC_Wait_Connect(int timeout_ms) {
  _cdc_start_time = m_time_ms();
  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    if (timeout_ms > 0 && m_time_ms() - _cdc_start_time > timeout_ms) return;
  }
  CDC_Send((uint8_t *)_cbuf, 2);
  while (hcdc->TxState != 0 ||
         hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
    if (timeout_ms > 0 && m_time_ms() - _cdc_start_time > timeout_ms) return;
  }
}

/**
 * @brief 注册USB CDC接收回调函数
 * @param callback 回调函数
 */
void CDC_Register_Callback(void (*callback)(char *buf, uint16_t len),
                           uint8_t cbkInIRQ) {
  usb_cdc.rxCallback = callback;
  usb_cdc.cbkInIRQ = cbkInIRQ;
}

/**
 * @brief USB是否已连接
 */
uint8_t USB_Connected(void) {
  return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

void CDC_Putchar(uint8_t data) { CDC_Send(&data, 1); }

void CDC_Puts(char *data) { CDC_Send((uint8_t *)data, strlen(data)); }

#endif  // _ENABLE_USB_CDC

#if _ENABLE_VOFA

float VOFA_Buffer[_VOFA_BUFFER_SIZE + 1];
uint8_t vofa_index = 0;
uint32_t vofa_endbit = 0x7F800000;

__attribute__((always_inline, flatten)) void Vofa_Add(float value) {
  if (vofa_index < _VOFA_BUFFER_SIZE) VOFA_Buffer[vofa_index++] = value;
}

void Vofa_AddSeq(float *value, uint8_t len) {
  if (vofa_index + len >= _VOFA_BUFFER_SIZE) return;
  memcpy(&VOFA_Buffer[vofa_index], value, len * sizeof(float));
  vofa_index += len;
}

void Vofa_Clear(void) { vofa_index = 0; }

void Vofa_Send(UART_HandleTypeDef *huart) {
  if (vofa_index == 0) return;
  Vofa_AddSeq((float *)&vofa_endbit, 1);
  Uart_Send(huart, (uint8_t *)VOFA_Buffer, vofa_index * sizeof(float));
  vofa_index = 0;
}

void Vofa_SendFast(UART_HandleTypeDef *huart) {
  if (vofa_index == 0) return;
  memcpy(&VOFA_Buffer[vofa_index], &vofa_endbit, sizeof(float));
  Uart_SendFast(huart, (uint8_t *)VOFA_Buffer, ++vofa_index * sizeof(float));
  vofa_index = 0;
}

void Vofa_SendCDC(void) {
  if (vofa_index == 0) return;
  Vofa_AddSeq((float *)&vofa_endbit, 1);
#if _ENABLE_USB_CDC
  CDC_Send((uint8_t *)VOFA_Buffer, vofa_index * sizeof(float));
#endif
  vofa_index = 0;
}

#endif  // _ENABLE_VOFA
