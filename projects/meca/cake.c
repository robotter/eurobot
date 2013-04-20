#include <math.h>
#include "cake.h"
#include "config.h"

static aeat_t cake_enc;


void cake_init(void)
{
  aeat_spi_init();
  aeat_init(&cake_enc, ENCODER_CS_PP);
}


void cake_set_angle(int16_t a)
{
  int32_t v = a * ((4096 * (float)CAKE_RADIUS) / (2*M_PI * CAKE_ENCODER_RADIUS));
  aeat_set_value(&cake_enc, v);
}


int16_t cake_get_angle(void)
{
  int32_t v = aeat_get_value(&cake_enc);
  return v * (2*M_PI * (float)CAKE_ENCODER_RADIUS / (float)CAKE_RADIUS * 4096);
}


