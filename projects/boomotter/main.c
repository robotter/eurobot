#include <avarix.h>
#include <avarix/intlvl.h>
#include <avarix/portpin.h>
#include <avarix/register.h>
#include <uart/uart.h>
#include <clock/clock.h>
#include <rome/rome.h>
#include <util/delay.h>
#include <timer/timer.h>
#include <timer/uptime.h>
#include <idle/idle.h>
#include "battery.h"
#include "leds.h"
#include "amplifier.h"
#include "dfplayer_mini.h"
#include "ws2812.h"
#include "draw.h"
#include "font_bitmap.inc.c"
#include "config.h"

#define BATTERY_ALERT_LIMIT  10000

rome_intf_t rome_intf;

#define ROME_DEVICE  ROME_ENUM_DEVICE_BOOMOTTER


// Define the global screen
// Allocate then define a pointer, initialize widh/height in main
// This allows to still but screen in bss.
uint8_t screen_data[sizeof(texture_t) + sizeof(pixel_t) * SCREEN_W * SCREEN_H];
texture_t *const screen = (texture_t*)screen_data;

// Robot scores
static struct {
  uint16_t galipeur;
  uint16_t galipette;
} scores;

static bool battery_discharged = false;

uint8_t celebration_duration = 0;


static void switch_mode(rome_enum_boomotter_mode_t mode)
{
  ROME_LOGF(&rome_intf, INFO, "boomotter: switch to mode %u", mode);

  switch(mode) {
    case ROME_ENUM_BOOMOTTER_MODE_MATCH:
      // maximum volume, no auto play
      _delay_ms(50);
      dfplayer_set_volume(30);
      if(dfplayer_is_busy()) {
        _delay_ms(50);
        dfplayer_pause();
      }
      break;

    case ROME_ENUM_BOOMOTTER_MODE_STAND:
      // medium volume, loop tracks
      _delay_ms(50);
      dfplayer_set_volume(10);
      _delay_ms(50);
      //TODO start a random track, or randomize mp3 directory
      dfplayer_random_all();
      if(!dfplayer_is_busy()) {
        _delay_ms(50);
        dfplayer_set_pause(false);
      }
      break;

    case ROME_ENUM_BOOMOTTER_MODE_SILENT:
      // just mute the MP3 player
      _delay_ms(50);
      dfplayer_set_volume(0);
      break;

    default:
      break;
  }
}


static void update_display_debug_team(void)
{
  static const char *scrolling_text = "DEBUG TEAM  ";
  static uint8_t scrolling_text_width = 0;
  static int8_t pos = 0;
  if(scrolling_text_width == 0) {
    scrolling_text_width = get_text_width(&font_base, scrolling_text);
  }

  texture_clear(screen);

  blend_text(screen, &font_base, pos, 0, scrolling_text, blend_gray_set);
  if(pos < 0) {
    blend_text(screen, &font_base, pos + scrolling_text_width, 0, scrolling_text, blend_gray_set);
  }
  for(uint8_t y = screen_upper_rect.y0; y < screen_upper_rect.y1; y++) {
    for(uint8_t x = screen_upper_rect.x0; x < screen_upper_rect.x1; x++) {
      pixel_t *p = TEXTURE_PIXEL(screen, x, y);
      if(p->r == 0) {
        *p = RGB(0,25,0x40);
      } else {
        *p = RGB(0x40,25,0);
      }
    }
  }

  if(--pos <= -scrolling_text_width) {
    pos = 0;
  }
}


static void draw_score(void)
{
  static uint16_t previous_total_score = 0;
  static uint16_t displayed_score = 0;
  uint16_t total_score = scores.galipeur + scores.galipette;
  if(total_score > 999) {
    total_score = 999;
  }
  if(total_score != previous_total_score) {
    celebration_duration = 40;
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
  uint8_t pos = (SCREEN_LW - 3*len - (len-1))/2;

  blend_text(screen, &font_score, pos, SCREEN_UH+1, buf, blend_gray_set);
  FOREACH_RECT_PIXEL(screen, screen_lower_rect) {
    if(p->r) {
      *p = RGB(0x4f,0x4f,0x4f);
    }
  }
}

static void draw_scrolling_robotter(void)
{
  static scrolling_text_t scroll;
  if(scroll.text_width == 0) {
    scrolling_text_init(&scroll, &font_base, "ROB'OTTER  ", 2);
  }
  scrolling_text_draw(&scroll, screen, 0);
  FOREACH_RECT_PIXEL(screen, screen_upper_rect) {
    if(p->r) {
      *p = RGB(0,0x40,0);
    }
  }
  scrolling_text_scroll(&scroll, -1);
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

  if(celebration_duration) {
    FOREACH_PIXEL(screen) {
      if(p->r == 0 && p->g == 0 && p->b == 0) {
        *p = celebration_colors[(celebration_duration + x + y) % (sizeof(celebration_colors)/sizeof(*celebration_colors))];
      }
    }
    celebration_duration--;
  }
}


static void update_display(void)
{
  texture_clear(screen);

  (void)update_display_debug_team;
  draw_score();
  draw_scrolling_robotter();
  draw_celebration();

  // if battery is low, display a red rectangle
  if(battery_discharged) {
    draw_rect(screen, &(draw_rect_t){0, 0, 6, 6}, RGB(0x30, 0, 0));
  }
  display_screen(screen);
}


static void rome_handler(rome_intf_t *intf, const rome_frame_t *frame)
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
          scores.galipeur = frame->tm_score.points;
          break;
        case ROME_ENUM_DEVICE_GALIPETTE_STRAT:
          scores.galipette = frame->tm_score.points;
          break;
        default:
          break;
      }
      break;

    case ROME_MID_BOOMOTTER_MP3_CMD:
      dfplayer_send_cmd(frame->boomotter_mp3_cmd.cmd, frame->boomotter_mp3_cmd.param);
      rome_reply_ack(intf, frame);
      break;

    case ROME_MID_BOOMOTTER_SET_MODE:
      switch_mode(frame->boomotter_set_mode.mode);
      break;

    default:
      break;
  }
}

static void update_battery(void)
{
  static uint8_t it = 0;
  if(++it > 10) {
    it = 0;
    uint16_t voltage = battery_get_value();
    battery_discharged = voltage < BATTERY_ALERT_LIMIT;
    ROME_SEND_TM_BATTERY(&rome_intf, ROME_DEVICE, voltage);
  }
}

static void update_rome(void)
{
  rome_handle_input(&rome_intf);
}

static void handle_dfplayer_input(void)
{
  dfplayer_handle_input(0);
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
  uart_fopen(UART_ROME);
  portpin_outclr(&LED_RUN_PP);

  INTLVL_ENABLE_ALL();
  __asm__("sei");

  battery_init();

  rome_intf_init(&rome_intf);
  rome_intf.uart = UART_ROME;
  rome_intf.handler = rome_handler;

  ROME_LOGF(&rome_intf, INFO, "boomotter booting");
  RST.STATUS = 0;
  portpin_outclr(&LED_ERROR_PP);

  timer_init();
  uptime_init();

  dfplayer_init();
  amplifier_init();
  ws2812_init();

  update_battery(); // make sure to update battery at startup
  TIMER_SET_CALLBACK_US(E0, 'B', 50e3, INTLVL_HI, update_battery);

  idle_set_callback(rome_update, update_rome);
  idle_set_callback(dfplayer_input, handle_dfplayer_input);
  idle_set_callback(display_update, update_display);

  _delay_ms(500);
  dfplayer_set_volume(0);

  // switch on audio
  amplifier_shutdown(false);
  amplifier_mute(false);
  //amplifier_set_gain(GAIN_26DB);
  //dfplayer_set_volume(20);

  // set to match mode by default
  switch_mode(ROME_ENUM_BOOMOTTER_MODE_MATCH);

  // initialize the screen
  screen->width = SCREEN_W;
  screen->height = SCREEN_H;
  texture_clear(screen);
  display_screen(screen);

  portpin_outset(&LED_RUN_PP);

  for(;;) {
    idle();
  }
}

