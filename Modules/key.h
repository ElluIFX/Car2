#ifndef __key_H
#define __key_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define KEY_EVENT_NULL 0x0000
#define KEY_EVENT_DOWN 0x0001
#define KEY_EVENT_SHORT 0x0002
#define KEY_EVENT_LONG 0x0003
#define KEY_EVENT_DOUBLE 0x0004
#define KEY_EVENT_HOLD 0x0005
#define KEY_EVENT_UP_HOLD 0x0006          // 按住后松开事件
#define KEY_EVENT_UP_DOUBLE 0x0007        // 双击后松开事件
#define KEY_EVENT_HOLD_CONTINUE 0x0008    // 按住连发事件
#define KEY_EVENT_DOUBLE_CONTINUE 0x0009  // 双击按住连发事件

/******************************************************************************
                           User Interface [START]
*******************************************************************************/

#define KEY_DOWN(num) (KEY_EVENT_DOWN | num << 8)
#define KEY_SHORT(num) (KEY_EVENT_SHORT | num << 8)
#define KEY_LONG(num) (KEY_EVENT_LONG | num << 8)
#define KEY_DOUBLE(num) (KEY_EVENT_DOUBLE | num << 8)
#define KEY_HOLD(num) (KEY_EVENT_HOLD | num << 8)
#define KEY_UP_HOLD(num) (KEY_EVENT_UP_HOLD | num << 8)
#define KEY_UP_DOUBLE(num) (KEY_EVENT_UP_DOUBLE | num << 8)
#define KEY_HOLD_CONTINUE(num) (KEY_EVENT_HOLD_CONTINUE | num << 8)
#define KEY_DOUBLE_CONTINUE(num) (KEY_EVENT_DOUBLE_CONTINUE | num << 8)

/******************************************************************************
                           User Interface [END]
*******************************************************************************/

extern unsigned short KEY_CHECK_MS;
extern unsigned short key_set_long_ms;
extern unsigned short key_set_hold_ms;
extern unsigned short key_set_shakefilter_ms;
extern unsigned short key_set_double_ms;
extern unsigned short key_set_continue_send_ms;

extern void key_check_all(void);
extern unsigned short key_read_value(void);

#ifdef __cplusplus
}
#endif

#endif
