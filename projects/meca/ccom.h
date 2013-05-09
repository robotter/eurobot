#ifndef CCOM_H
#define CCOM_H

#include <avarix.h>
#include <uart/uart.h>
#include "candle_defs.h"
#include <stdbool.h>

typedef struct
{
  /// uart accessors
  int (*uart_recv_no_wait)(void);
  void (*uart_send)(uint8_t v);
  
  acm_candle_color_t candle_color;  // last candle color received
  bool cmu_cam_ready;       // status of the camera
  bool lightning_led_on;    // status of the led that light the candles
} ccom_t;


typedef enum 
{
  ccom_bit_pos_candle_color = 0,
  ccom_bit_pos_candle_color_valid,
  ccom_bit_pos_camera_ready,
  ccom_bit_pos_lightning_led_status,
} ccom_com_bitfield;

void ccom_init(ccom_t *s);

void ccom_update(ccom_t *s);

bool ccom_is_camera_ready(ccom_t *s);

acm_candle_color_t ccom_get_candle_color(ccom_t *s);


#endif //CCOM_H
