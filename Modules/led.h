/**
 * @file led.h
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-05-15
 *
 * THINK DIFFERENTLY
 */

#ifndef _LED_H_
#define _LED_H_

#include "mod_config.h"

#if defined(LED_R_Pin) && defined(LED_G_Pin) && defined(LED_B_Pin)
extern void LED(uint8_t R, uint8_t G, uint8_t B);
#elif defined(LED_Pin)
extern void LED(uint8_t act);
#endif

#endif
