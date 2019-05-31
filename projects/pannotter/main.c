#include <stdlib.h>
#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <avarix/register.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include <xbee/xbee.h>
#include <util/delay.h>
#include <timer/timer.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include "battery.h"
#include "leds.h"
#include "ws2812.h"
#include "tyrolienne.h"
#include "draw.h"
#include "resources.inc.c"
#include "config.h"

// ROME and XBee interfaces
xbee_intf_t xbee_paddock;

#define ROME_DEVICE  ROME_ENUM_DEVICE_PANNOTTER

#define SCORE_EXPERIENCE_INSTALLED  5
#define SCORE_EXPERIENCE_ACTIVATED  15
#define SCORE_EXPERIENCE_SUCCESS  20


// Define the global screen
// Allocate then define a pointer, initialize widh/height in main
// This allows to still but screen in bss.
uint8_t screen_data[sizeof(texture_t) + sizeof(pixel_t) * SCREEN_W * SCREEN_H];
texture_t *const screen = (texture_t*)screen_data;

static bool battery_discharged = false;
static bool battery_on_stand = false;

typedef struct {
  uint8_t celebration_duration;
  // Robot scores
  struct {
    uint16_t galipeur;
    uint16_t galipette;
    uint16_t pannotter;
  } scores;
  uint16_t timer;  // timer in sconds
  uint16_t timer_last_update;  // uptime in seconds

} match_state_t;

static match_state_t match_state;

static void start_experiment(void){
  ROME_LOGF(UART_XBEE, INFO, "pannotter: start experience");
  if(!tyrolienne_started()) {
    match_state.scores.pannotter += SCORE_EXPERIENCE_ACTIVATED;
  }
  tyrolienne_start();
}

static void draw_score(void)
{
  static uint16_t previous_total_score = 0;
  static uint16_t displayed_score = 0;
  uint16_t total_score = match_state.scores.galipeur + match_state.scores.galipette +  + match_state.scores.pannotter;
  /*
  if(total_score == previous_total_score) {
    return;
  }
  */

  if(total_score > previous_total_score) {
    // points gain
    match_state.celebration_duration = 40;
    previous_total_score = total_score;
  } else if(total_score < previous_total_score) {
    // points loss (don't celebrate)
    previous_total_score = total_score;
  }

  if(total_score > displayed_score) {
    displayed_score += (total_score - displayed_score) / 5 + 1;
  } else if(total_score < displayed_score) {
    displayed_score -= (displayed_score - total_score) / 5 + 1;
  }

  char buf[4];
  snprintf(buf, sizeof(buf), "%u", displayed_score);
  uint8_t len = strlen(buf);
  uint8_t pos = (SCREEN_W - 3*len - (len-1))/2;

  blend_text(screen, &font_score, pos, 1, buf, blend_gray_set);
  FOREACH_RECT_PIXEL(screen, screen_rect) {
    if(p->r) {
      *p = RGB(0x4f,0x4f,0x4f);
    }
  }
}

static void draw_celebration(void)
{
  static const pixel_t celebration_colors[] = {
    {0x00,0x00,0x06},
    {0x06,0x00,0x06},
    {0x06,0x00,0x00},
    {0x06,0x06,0x00},
    {0x00,0x06,0x00},
    {0x00,0x06,0x06},
  };

  if(match_state.celebration_duration) {
    FOREACH_PIXEL(screen) {
      if(p->r == 0 && p->g == 0 && p->b == 0) {
        *p = celebration_colors[(match_state.celebration_duration + x + y) % (sizeof(celebration_colors)/sizeof(*celebration_colors))];
      }
    }
    match_state.celebration_duration--;
  }
}

static void draw_match(void)
{
  draw_score();
  draw_celebration();
}


static void update_display(void)
{
  // if robots have not updated the timer, match has ended
  uint16_t uptime = uptime_us() / 1000000;
  if(match_state.timer_last_update != 0 && uptime >= match_state.timer_last_update + ROBOTS_ALIVE_TIMEOUT) {
    match_state.timer_last_update = 0;
  }

  portpin_outtgl(&LED_RUN_PP);
  texture_clear(screen);

  draw_match();

#if 0 // battery
  // if battery is low, display a red rectangle
  if(battery_discharged) {
    draw_rect(screen, &(draw_rect_t){0, 0, 6, 6}, RGB(0x30, 0, 0));
  }
#endif

  portpin_outtgl(&LED_RUN_PP);
  display_screen(screen);
}


static void rome_xbee_handler(uint16_t addr, const rome_frame_t *frame)
{
  portpin_outtgl(&LED_COM_PP);

  switch(frame->mid) {
    case ROME_MID_ACK:
      // should not happen
      rome_free_ack(frame->ack.ack);
      return;

    case ROME_MID_RESET: {
      software_reset();
    } break;

    case ROME_MID_TM_SCORE:
      switch(frame->tm_score.device) {
        case ROME_ENUM_DEVICE_GALIPEUR_STRAT:
          match_state.scores.galipeur = frame->tm_score.points;
          break;
        case ROME_ENUM_DEVICE_GALIPETTE_STRAT:
          match_state.scores.galipette = frame->tm_score.points;
          break;
        default:
          break;
      }
      if (frame->tm_score.points > 0)
        start_experiment();
      break;

    case ROME_MID_TM_MATCH_TIMER: {
      match_state.timer = frame->tm_match_timer.seconds;
      match_state.timer_last_update = uptime_us() / 1000000;
      if(0) {//match_state.timer > 90) {
        ROME_LOGF(UART_XBEE, INFO, "pannotter: stop motor");
        tyrolienne_stop();
      } else if(match_state.timer > 1) {
        start_experiment();
      }
    } break;

    case ROME_MID_PANNOTTER_START_EXPERIENCE: {
      start_experiment();
    } break;

#if 0  // should not be needed anymore
    case ROME_MID_ASSERV_TM_XYA: {
      switch(frame->asserv_tm_xya.device) {
        case ROME_ENUM_DEVICE_GALIPETTE_ASSERV:
          ROME_SEND_TM_ROBOT_POSITION(&rome_intf, ROME_ENUM_DEVICE_GALIPETTE_STRAT, frame->asserv_tm_xya.x, frame->asserv_tm_xya.y, frame->asserv_tm_xya.a);
          break;
        case ROME_ENUM_DEVICE_GALIPEUR_ASSERV:
          ROME_SEND_TM_ROBOT_POSITION(&rome_intf, ROME_ENUM_DEVICE_GALIPEUR_STRAT, frame->asserv_tm_xya.x, frame->asserv_tm_xya.y, frame->asserv_tm_xya.a);
          break;
        default:
          break;
      }
    } break;
#endif

    default:
      break;
  }
}

void update_rome_interfaces(void)
{
  xbee_handle_input(&xbee_paddock);
}


static void update_battery(void)
{
  static uint8_t it = 0;
  if(++it > 10) {
    it = 0;
    uint16_t voltage = battery_get_value();
    battery_discharged = voltage < BATTERY_ALERT_LIMIT;
    battery_on_stand = voltage >= BATTERY_STAND_THRESHOLD;
    ROME_SEND_TM_BATTERY(UART_XBEE, ROME_DEVICE, voltage);
  }
}

static void xbee_paddock_handler(xbee_intf_t *intf, const xbee_frame_t *frame)
{
  switch(frame->api_id) {
    case XBEE_ID_RX16: {
      const rome_frame_t *rome_frame = rome_parse_frame(frame->rx16.data, frame->length - 5);
      if(rome_frame) {
        const uint16_t addr = (frame->rx16.addr_be << 8) | (frame->rx16.addr_be & 0xff);
        rome_xbee_handler(addr, rome_frame);
      }
    } break;
    default:
      break;  // ignore
  }
}


int main(void)
{
  portpin_dirset(&LED_RUN_PP);
  portpin_dirset(&LED_ERROR_PP);
  portpin_dirset(&LED_COM_PP);

  portpin_outset(&LED_COM_PP);
  portpin_outset(&LED_RUN_PP);
  portpin_outset(&LED_ERROR_PP);

  clock_init();
  portpin_outclr(&LED_COM_PP);

  uart_init();
  //uart_fopen(UART_XBEE);
  portpin_outclr(&LED_RUN_PP);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  battery_init();

  xbee_intf_init(&xbee_paddock, UART_XBEE);
  xbee_paddock.handler = xbee_paddock_handler;

  ROME_LOGF(UART_XBEE, INFO, "pannotter booting");
  RST.STATUS = 0;
  portpin_outclr(&LED_ERROR_PP);

  timer_init();
  uptime_init();

  ws2812_init();
  tyrolienne_init();

#if 0 // battery
  update_battery(); // make sure to update battery at startup
  TIMER_SET_CALLBACK_US(E0, 'B', 50e3, INTLVL_HI, update_battery);
#else
  (void)update_battery;
#endif

  idle_set_callback(rome_update, update_rome_interfaces);
  idle_set_callback(display_update, update_display);

  //XXX tyrolienne_start();

  _delay_ms(500);

  //XXX pannoter initial score disabled, galipette will count it
  //match_state.scores.pannotter = SCORE_EXPERIENCE_INSTALLED;

  // initialize the screen
  screen->width = SCREEN_W;
  screen->height = SCREEN_H;
  texture_clear(screen);

  *TEXTURE_PIXEL(screen, 0, 0) = RGB(0x10, 0x00, 0x00);
  *TEXTURE_PIXEL(screen, 1, 1) = RGB(0x00, 0x10, 0x00);
  *TEXTURE_PIXEL(screen, 2, 2) = RGB(0x00, 0x00, 0x10);

  display_screen(screen);

  portpin_outset(&LED_RUN_PP);

  // wait for "arrived" state to increase score
  for(;;) {
    idle();
    if(tyrolienne_arrived()) {
      match_state.scores.pannotter += SCORE_EXPERIENCE_SUCCESS;
      break;
    }
  }

  for(;;) {
    idle();
  }
}

