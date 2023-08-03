#include "key.h"

#define KEY_READ_DOWN 0x00 /* key is pressed          */
#define KEY_READ_UP 0x01   /* Key isn't pressed       */
#define KEY_BUF_SIZE 10    /* key buffer size         */

#define KEY_CHECK_MS 10                      // 按键检测周期，单位ms
unsigned short key_set_long_ms = 300;        // 长按时间
unsigned short key_set_hold_ms = 800;        // 按住时间
unsigned short key_set_shakefilter_ms = 20;  // 按键抖动滤波时间
unsigned short key_set_double_ms = 0;  // 双击最大间隔时间, 为0不检测
unsigned short key_set_continue_wait_ms = 300;  // 按住/双击按住连发等待时间
unsigned short key_set_continue_send_ms = 100;  // 按住/双击按住连发执行间隔

struct {
  unsigned short value[KEY_BUF_SIZE];
  unsigned char rd;
  unsigned char wr;
} key_buf;

struct key_dev {
  unsigned char (*read_key)(void); /* key read pin status function pointer*/
  unsigned char num;               /* number                              */
  void (*status)(struct key_dev *key_dev); /* state machine status */
  unsigned short count_ms; /* ms counter                          */
};

static void key_write_value(unsigned short key_val);
static void key_status_down_check(struct key_dev *key_dev);
static void key_status_down_shake(struct key_dev *key_dev);
static void key_status_down_handle(struct key_dev *key_dev);
static void key_status_hold_check(struct key_dev *key_dev);
static void key_status_short_up_shake(struct key_dev *key_dev);
static void key_status_short_up_handle(struct key_dev *key_dev);
static void key_status_long_up_shake(struct key_dev *key_dev);
static void key_status_long_up_handle(struct key_dev *key_dev);
static void key_status_double_check(struct key_dev *key_dev);
static void key_status_double_down_shake(struct key_dev *key_dev);
static void key_status_double_continue_wait_check(struct key_dev *key_dev);
static void key_status_double_continue_check(struct key_dev *key_dev);
static void key_status_double_up_check(struct key_dev *key_dev);
static void key_status_double_up_shake(struct key_dev *key_dev);
static void key_status_double_up_handle(struct key_dev *key_dev);
static void key_status_hold_handle(struct key_dev *key_dev);
static void key_status_hold_continue_wait_check(struct key_dev *key_dev);
static void key_status_hold_continue_check(struct key_dev *key_dev);
static void key_status_hold_up_shake(struct key_dev *key_dev);
static void key_status_hold_up_handle(struct key_dev *key_dev);

/******************************************************************************
                           User Interface [START]
*******************************************************************************/
#ifdef KEY_GPIO_Port
static unsigned char key_read(void) {
  return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY1_GPIO_Port
static unsigned char key1_read(void) {
  return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY2_GPIO_Port
static unsigned char key2_read(void) {
  return HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY3_GPIO_Port
static unsigned char key3_read(void) {
  return HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY4_GPIO_Port
static unsigned char key4_read(void) {
  return HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY5_GPIO_Port
static unsigned char key5_read(void) {
  return HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY6_GPIO_Port
static unsigned char key6_read(void) {
  return HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY7_GPIO_Port
static unsigned char key7_read(void) {
  return HAL_GPIO_ReadPin(KEY7_GPIO_Port, KEY7_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY8_GPIO_Port
static unsigned char key8_read(void) {
  return HAL_GPIO_ReadPin(KEY8_GPIO_Port, KEY8_Pin) != GPIO_PIN_RESET;
}
#endif
#ifdef KEY9_GPIO_Port
static unsigned char key9_read(void) {
  return HAL_GPIO_ReadPin(KEY9_GPIO_Port, KEY9_Pin) != GPIO_PIN_RESET;
}
#endif

struct key_dev key_dev[] = {
#ifdef KEY_GPIO_Port
    {
        (unsigned char (*)(void))key_read,
        0x0000,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY1_GPIO_Port
    {
        (unsigned char (*)(void))key1_read,
        0x0001,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY2_GPIO_Port
    {
        (unsigned char (*)(void))key2_read,
        0x0002,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY3_GPIO_Port
    {
        (unsigned char (*)(void))key3_read,
        0x0003,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY4_GPIO_Port
    {
        (unsigned char (*)(void))key4_read,
        0x0004,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY5_GPIO_Port
    {
        (unsigned char (*)(void))key5_read,
        0x0005,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY6_GPIO_Port
    {
        (unsigned char (*)(void))key6_read,
        0x0006,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY7_GPIO_Port
    {
        (unsigned char (*)(void))key7_read,
        0x0007,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY8_GPIO_Port
    {
        (unsigned char (*)(void))key8_read,
        0x0008,
        key_status_down_check,
        0,
    },
#endif
#ifdef KEY9_GPIO_Port
    {
        (unsigned char (*)(void))key9_read,
        0x0009,
        key_status_down_check,
        0,
    },
#endif
};

const unsigned char key_dev_num = sizeof(key_dev) / sizeof(key_dev[0]);

/******************************************************************************
                           User Interface [END]
*******************************************************************************/

/**
description : write key vaule to buffer
param :  key_val - key value , (KEY_EVENT | KEY_NUMBER<<8)
retval : None
*/
static void key_write_value(unsigned short key_val) {
  key_buf.value[key_buf.wr++] = key_val;
  key_buf.wr %= KEY_BUF_SIZE;

  /*
      overflow handle
  */
  if (key_buf.wr == key_buf.rd) {
    key_buf.rd++;
    key_buf.rd %= KEY_BUF_SIZE;
  }
}

/**
description : read key vaule from buffer
param : None
retval : key_val - key value , (KEY_EVENT | KEY_NUMBER<<8)
*/
unsigned short key_read_value(void) {
  if (key_buf.wr == key_buf.rd) {
    return KEY_EVENT_NULL;
  } else {
    unsigned short key_val = key_buf.value[key_buf.rd++];
    key_buf.rd %= KEY_BUF_SIZE;
    return key_val;
  }
}

/**
description : check key whether press down
param : key_dev - key device pointer
retval : None
*/
static void key_status_down_check(struct key_dev *key_dev) {
  unsigned char key_read;

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_DOWN) {
    key_dev->status = key_status_down_shake;
    key_dev->count_ms = 0;
  }
}

/**
description : filter shake after key pressed down
param : key_dev - key device pointer
retval : None
*/
static void key_status_down_shake(struct key_dev *key_dev) {
  unsigned char key_read;

  key_dev->count_ms += KEY_CHECK_MS;

  if (key_dev->count_ms < key_set_shakefilter_ms) {
    return;
  }

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_DOWN) {
    key_dev->status = key_status_down_handle;
  } else {
    key_dev->status = key_status_down_check;
  }
}

/**
description : key press down handle after pressed down filter shake
param : key_dev - key device pointer
retval : None
*/
static void key_status_down_handle(struct key_dev *key_dev) {
  unsigned short key_val = key_dev->num << 8 | KEY_EVENT_DOWN;

  key_write_value(key_val);

  key_dev->status = key_status_hold_check;
  key_dev->count_ms = 0;
}

/**
description : check key whether hold click
param : key_dev - key device pointer
retval : None
*/
static void key_status_hold_check(struct key_dev *key_dev) {
  unsigned char key_read;

  key_dev->count_ms += KEY_CHECK_MS;
  key_read = key_dev->read_key();

  if (key_dev->count_ms < key_set_long_ms) {
    if (key_read == KEY_READ_UP) {
      key_dev->status = key_status_short_up_shake;
    }
    return;
  }

  if (key_dev->count_ms < key_set_hold_ms) {
    if (key_read == KEY_READ_UP) {
      key_dev->status = key_status_long_up_shake;
    }
    return;
  }

  key_dev->status = key_status_hold_handle;
}

/**
description : short cilck key up filter shake
param : key_dev - key device pointer
retval : None
*/
static void key_status_short_up_shake(struct key_dev *key_dev) {
  unsigned char key_read;
  static unsigned short old = 0xffff;

  if (old == 0xffff) {
    old = key_dev->count_ms;
    key_dev->count_ms = 0;
  }

  key_dev->count_ms += KEY_CHECK_MS;

  if (key_dev->count_ms < key_set_shakefilter_ms) {
    return;
  }

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_double_check;
    key_dev->count_ms = 0;
  } else {
    key_dev->status = key_status_hold_check;
    key_dev->count_ms += old;
  }

  old = 0xffff;
}

/**
description : short click key up handle
param : key_dev - key device pointer
retval : None
*/
static void key_status_short_up_handle(struct key_dev *key_dev) {
  unsigned short key_val;

  key_val = key_dev->num << 8 | KEY_EVENT_SHORT;

  key_write_value(key_val);

  key_dev->status = key_status_down_check;
}

/**
description : long cilck key up filter shake
param : key_dev - key device pointer
retval : None
*/
static void key_status_long_up_shake(struct key_dev *key_dev) {
  unsigned char key_read;
  static unsigned short old = 0xffff;

  if (old == 0xffff) {
    old = key_dev->count_ms;
    key_dev->count_ms = 0;
  }

  key_dev->count_ms += KEY_CHECK_MS;

  if (key_dev->count_ms < key_set_shakefilter_ms) {
    return;
  }

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_long_up_handle;
    key_dev->count_ms = 0;
  } else {
    key_dev->status = key_status_hold_check;
    key_dev->count_ms += old;
  }

  old = 0xffff;
}

/**
description : long click key up handle
param : key_dev - key device pointer
retval : None
*/
static void key_status_long_up_handle(struct key_dev *key_dev) {
  unsigned short key_val;

  key_val = key_dev->num << 8 | KEY_EVENT_LONG;

  key_write_value(key_val);

  key_dev->status = key_status_down_check;
}

/**
description : double cilck check. we consider double click event if key pressed
              down when after short click up and within max double click
interval param : key_dev - key device pointer retval : None */
static void key_status_double_check(struct key_dev *key_dev) {
  unsigned char key_read;

  key_dev->count_ms += KEY_CHECK_MS;
  key_read = key_dev->read_key();

  if (key_dev->count_ms < key_set_double_ms) {
    if (key_read == KEY_READ_DOWN) {
      key_dev->status = key_status_double_down_shake;
      key_dev->count_ms = 0;
    }
  } else {
    key_dev->status = key_status_short_up_handle;
  }
}

/**
description : double click key down filter shake
param : key_dev - key device pointer
retval : None
*/
static void key_status_double_down_shake(struct key_dev *key_dev) {
  unsigned char key_read;
  static unsigned short old = 0xffff;

  if (old == 0xffff) {
    old = key_dev->count_ms;
    key_dev->count_ms = 0;
  }

  key_dev->count_ms += KEY_CHECK_MS;

  if (key_dev->count_ms < key_set_shakefilter_ms) {
    return;
  }

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_DOWN) {
    unsigned short key_val;

    key_val = key_dev->num << 8 | KEY_EVENT_DOUBLE;

    key_write_value(key_val);

    key_dev->status = key_status_double_continue_wait_check;
    key_dev->count_ms = 0;
  } else {
    key_dev->status = key_status_double_check;
    key_dev->count_ms += old;
  }

  old = 0xffff;
}

/**
description : continue send if double click time overflow
param : key_dev - key device pointer
retval : None
*/
static void key_status_double_continue_wait_check(struct key_dev *key_dev) {
  unsigned char key_read;
  unsigned short key_val;

  key_dev->count_ms += KEY_CHECK_MS;

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_double_up_shake;
  }

  if (key_dev->count_ms < key_set_continue_wait_ms) {
    return;
  }

  if (key_set_continue_send_ms == 0) {
    return;
  }

  key_val = key_dev->num << 8 | KEY_EVENT_DOUBLE_CONTINUE;

  key_write_value(key_val);
  key_dev->status = key_status_double_continue_check;
  key_dev->count_ms = 0;
}

/**
description : continue send if double click time overflow
param : key_dev - key device pointer
retval : None
*/
static void key_status_double_continue_check(struct key_dev *key_dev) {
  unsigned char key_read;
  unsigned short key_val;

  key_dev->count_ms += KEY_CHECK_MS;

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_double_up_shake;
  }

  if (key_dev->count_ms < key_set_continue_send_ms) {
    return;
  }

  key_val = key_dev->num << 8 | KEY_EVENT_DOUBLE_CONTINUE;

  key_write_value(key_val);
  key_dev->count_ms = 0;
}

/**
description : double click key up check
param : key_dev - key device pointer
retval : None
*/
static void key_status_double_up_check(struct key_dev *key_dev) {
  unsigned char key_read;

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_double_up_shake;
    key_dev->count_ms = 0;
  }
}

/**
description : double click key up filter shake
param : key_dev - key device pointer
retval : None
*/
static void key_status_double_up_shake(struct key_dev *key_dev) {
  unsigned char key_read;

  key_dev->count_ms += KEY_CHECK_MS;

  if (key_dev->count_ms < key_set_shakefilter_ms) {
    return;
  }

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_double_up_handle;
  } else {
    key_dev->status = key_status_double_up_check;
  }
}

/**
description : double click key up handle
param : key_dev - key device pointer
retval : None
*/
static void key_status_double_up_handle(struct key_dev *key_dev) {
  unsigned short key_val;

  key_val = key_dev->num << 8 | KEY_EVENT_UP_DOUBLE;

  key_write_value(key_val);

  key_dev->status = key_status_down_check;
}

/**
description : hold click handle after hold click check
param : key_dev - key device pointer
retval : None
*/
static void key_status_hold_handle(struct key_dev *key_dev) {
  unsigned short key_val;

  key_val = key_dev->num << 8 | KEY_EVENT_HOLD;

  key_write_value(key_val);

  key_dev->status = key_status_hold_continue_wait_check;
  key_dev->count_ms = 0;
}

/**
description : continue send if hold click time overflow
param : key_dev - key device pointer
retval : None
*/
static void key_status_hold_continue_wait_check(struct key_dev *key_dev) {
  unsigned char key_read;
  unsigned short key_val;

  key_dev->count_ms += KEY_CHECK_MS;

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_hold_up_shake;
  }

  if (key_dev->count_ms < key_set_continue_wait_ms) {
    return;
  }

  if (key_set_continue_send_ms == 0) {
    return;
  }

  key_val = key_dev->num << 8 | KEY_EVENT_HOLD_CONTINUE;

  key_write_value(key_val);
  key_dev->status = key_status_hold_continue_check;
  key_dev->count_ms = 0;
}
/**
description : continue send if hold click time overflow
param : key_dev - key device pointer
retval : None
*/
static void key_status_hold_continue_check(struct key_dev *key_dev) {
  unsigned char key_read;
  unsigned short key_val;

  key_dev->count_ms += KEY_CHECK_MS;

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_hold_up_shake;
  }

  if (key_dev->count_ms < key_set_continue_send_ms) {
    return;
  }

  key_val = key_dev->num << 8 | KEY_EVENT_HOLD_CONTINUE;

  key_write_value(key_val);
  key_dev->count_ms = 0;
}

/**
description : hold click key up filter shake
param : key_dev - key device pointer
retval : None
*/
static void key_status_hold_up_shake(struct key_dev *key_dev) {
  unsigned char key_read;
  static unsigned short old = 0xffff;

  if (old == 0xffff) {
    old = key_dev->count_ms;
    key_dev->count_ms = 0;
  }

  key_dev->count_ms += KEY_CHECK_MS;

  if (key_dev->count_ms < key_set_shakefilter_ms) {
    return;
  }

  key_read = key_dev->read_key();

  if (key_read == KEY_READ_UP) {
    key_dev->status = key_status_hold_up_handle;
  } else {
    key_dev->status = key_status_hold_continue_check;
    key_dev->count_ms += old;
  }

  old = 0xffff;
}

/**
description : hold click key up filter handle
param : key_dev - key device pointer
retval : None
*/
static void key_status_hold_up_handle(struct key_dev *key_dev) {
  unsigned short key_val;

  key_val = key_dev->num << 8 | KEY_EVENT_UP_HOLD;

  key_write_value(key_val);

  key_dev->status = key_status_down_check;
}

/**
description : run all key state machine once every KEY_CHECK_MS
param : key_dev - key device pointer
retval : None
*/
void key_check_all(void) {
  for (unsigned char i = 0; i < key_dev_num; i++) {
    key_dev[i].status(&key_dev[i]);
  }
}
