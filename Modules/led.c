/**
 * @file led.c
 * @brief LED控制
 * @author Ellu (lutaoyu@163.com)
 * @version 1.0
 * @date 2023-05-15
 *
 * THINK DIFFERENTLY
 */

#include "led.h"

#include "log.h"

#if defined(LED_R_Pin) && defined(LED_G_Pin) && defined(LED_B_Pin)
/**
 * @brief LED控制
 * @param  R/G/B: 0-熄灭 1-点亮 2-翻转 其他-忽略
 */
void LED(uint8_t R, uint8_t G, uint8_t B) {
  if (R <= 1)
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,
                      R ? GPIO_PIN_RESET : GPIO_PIN_SET);
  else if (R == 2)
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
  if (G <= 1)
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,
                      G ? GPIO_PIN_RESET : GPIO_PIN_SET);
  else if (G == 2)
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  if (B <= 1)
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,
                      B ? GPIO_PIN_RESET : GPIO_PIN_SET);
  else if (B == 2)
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}
#elif defined(LED_Pin)
/**
 * @brief LED控制
 * @param  act: 0-熄灭 1-点亮 2-翻转 其他-忽略
 */
void LED(uint8_t act) {
  if (act <= 1)
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                      act ? GPIO_PIN_SET : GPIO_PIN_RESET);
  else if (act == 2)
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
#endif
