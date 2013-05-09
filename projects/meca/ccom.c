#include <avarix.h>
#include "ccom.h"

void ccom_init(ccom_t *s)
{
  s->uart_recv_no_wait = NULL;
  s->uart_send = NULL;

  s->cmu_cam_ready = false;
  s->lightning_led_on = false;
  s->candle_color = ACM_CANDLE_UNKNOWN;
}

void ccom_update(ccom_t *s)
{
  int c = (s->uart_recv_no_wait)();

  if (c!= -1)
  {
    // check if data is valid (4 MSB bits inverted from 4 LSB one)
    // 4 LSB bits are not inverted

    // compute inverted data
    uint8_t inverted_data = ( ~((uint8_t)c >>4)) & 0x0F; 
    uint8_t cmu_cam_data =  (uint8_t)c & 0x0F;

    if (cmu_cam_data == inverted_data) // the most simple check
    {
      if (cmu_cam_data & (1<< ccom_bit_pos_candle_color_valid))
      {
        s->candle_color = (cmu_cam_data & (1<< ccom_bit_pos_candle_color) ? ACM_CANDLE_BLUE : ACM_CANDLE_RED);
      }
      else
      {
        s-> candle_color = ACM_CANDLE_UNKNOWN;
      }

      s->cmu_cam_ready = (cmu_cam_data & (1<< ccom_bit_pos_camera_ready) ? true : false);

      s->lightning_led_on = (cmu_cam_data & (1<< ccom_bit_pos_lightning_led_status) ? true : false);


      PORTA.DIRSET = 0x0F;
      PORTA.OUTTGL = _BV(0);
      if (s->cmu_cam_ready)
      {
        PORTA.OUTSET = _BV(1);
      }

      if (s->candle_color == ACM_CANDLE_UNKNOWN)
      {
        PORTA.OUTCLR = _BV(2);
        PORTA.OUTCLR = _BV(3);
      }
      else if (s->candle_color == ACM_CANDLE_RED)
      {
        PORTA.OUTSET = _BV(2);
        PORTA.OUTCLR = _BV(3);
      }
      else if (s->candle_color == ACM_CANDLE_BLUE)
      {
        PORTA.OUTSET = _BV(2);
        PORTA.OUTSET = _BV(3);
      }
    }
  }
}

bool ccom_is_camera_ready(ccom_t *s)
{
  return s->cmu_cam_ready;
}

acm_candle_color_t ccom_get_candle_color(ccom_t *s)
{
  return s->candle_color;
  acm_candle_color_t color = s->candle_color;
  s->candle_color = ACM_CANDLE_UNKNOWN;
  return color;
}
