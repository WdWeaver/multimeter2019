#include "stm32f1xx_hal.h"

int panel_initialize(
  TIM_HandleTypeDef *b, TIM_HandleTypeDef *p, TIM_HandleTypeDef *z
 ,I2C_HandleTypeDef *h, uint16_t a);
int panel_setsegment(const uint8_t *value, const uint8_t option);
int panel_update_view(void);
int panel_refresh(void);
int panel_blank(void);
int panel_update_buttons();
void panel_measure_transient();
void panel_trigger_config();
int decimalToBCD(uint16_t s, uint8_t d[]);
int panel_toggle_mode(int dir);
int panel_set_config(int dir);
void panel_set_illum();
int panel_set_buzz(int on);
void panel_updaterpm(uint16_t rpm);
