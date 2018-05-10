#include <stdlib.h>
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
#include "dfplayer_tracks.h"
#include "dfplayer_mini_defs.h"
#include "ws2812.h"
#include "draw.h"
#include "resources.inc.c"
#include "config.h"

#define BATTERY_ALERT_LIMIT  12000

rome_intf_t rome_intf;

#define ROME_DEVICE  ROME_ENUM_DEVICE_BOOMOTTER


// Define the global screen
// Allocate then define a pointer, initialize widh/height in main
// This allows to still but screen in bss.
uint8_t screen_data[sizeof(texture_t) + sizeof(pixel_t) * SCREEN_W * SCREEN_H];
texture_t *const screen = (texture_t*)screen_data;

static bool battery_discharged = false;

// Match sounds, ordered by priority (higher is better)
typedef enum {
  SOUND_NONE = 0,
  SOUND_COLLECTING_WATER,
  SOUND_COLLECTING_WATER_END,
  SOUND_BEE_LAUNCHED,
  SOUND_POINTS_LOSS,
  SOUND_ROBOTS_DEAD,
  SOUND_POINTS_GAIN,
  SOUND_MATCH_END,
  SOUND_AFTER_MATCH,
  SOUND_BOOT,
  SOUND_LOW_BATTERY,
} sound_t;

typedef struct {
  uint8_t celebration_duration;
  // Robot scores
  struct {
    uint16_t galipeur;
    uint16_t galipette;
  } scores;
  uint16_t timer;  // timer in sconds
  uint16_t timer_last_update;  // uptime in seconds
  uint16_t after_match_uptime_s;  // uptime at which play after match
  rome_enum_meca_state_t meca_state;
  // Currently playing sound
  sound_t current_sound;
  // True if current sound has been actually started
  bool current_sound_started;

} match_state_t;

static match_state_t match_state;
static rome_enum_boomotter_mode_t boomotter_mode;


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

    case ROME_ENUM_BOOMOTTER_MODE_MUSIC:
      // medium volume, loop tracks
      _delay_ms(50);
      dfplayer_set_volume(15);
      _delay_ms(50);
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

    case ROME_ENUM_BOOMOTTER_MODE_LOITUMA:
    case ROME_ENUM_BOOMOTTER_MODE_NYANCAT:
      if(dfplayer_is_busy()) {
        dfplayer_set_pause(true);
      }
      break;

    default:
      break;
  }
  boomotter_mode = mode;
}

static void start_sound(sound_t sound)
{
  uint16_t folder_track = 0;
  switch(sound) {
    case SOUND_NONE:
      dfplayer_pause();
      return;
    case SOUND_BEE_LAUNCHED:
      folder_track = TRACK_MUSICS_FRENCH_TRAIN_REMIX_SNCF_BY_JAUGS;
      break;
    case SOUND_POINTS_LOSS:
      folder_track = TRACK_SOUNDS_POUIK_HAHAHA;
      break;
    case SOUND_AFTER_MATCH:
      folder_track = TRACK_MUSICS_THIS_AEROBIC_VIDEO_WINS_EVERYTHING_480P_EXTENDED;
      break;
    case SOUND_ROBOTS_DEAD:
      folder_track = TRACK_SOUNDS_TRAVAIL_TERMINE;
      break;
    case SOUND_POINTS_GAIN:
      folder_track = TRACK_SOUNDS_ZELDA_SECRET_FOUND;
      break;
    case SOUND_BOOT:
      folder_track = TRACK_SOUNDS_WIII_ENCORE_DU_TRAVAIL;
      break;
    case SOUND_COLLECTING_WATER:
      folder_track = TRACK_SOUNDS_BALLS_CHARGE;
      break;
    case SOUND_COLLECTING_WATER_END:
      folder_track = TRACK_SOUNDS_BALLS_FANFARE;
      break;
    case SOUND_MATCH_END:
      folder_track = TRACK_SOUNDS_FINAL_FANTASY_VII_VICTORY_FANFARE;
      break;
    case SOUND_LOW_BATTERY:
      folder_track = TRACK_SOUNDS_ZELDA_LOW_HEALTH;
      break;
  }

  if(folder_track) {
    dfplayer_send_cmd(DF_PLAY_FOLDER_TRACK, folder_track);
  }
}

static void play_sound(sound_t sound)
{
  if(!dfplayer_is_busy()) {
    match_state.current_sound = SOUND_NONE;
  }
  if(sound <= match_state.current_sound) {
    // sound with higher priority is playing
    return;
  }
  match_state.current_sound = sound;
  match_state.current_sound_started = false;
  start_sound(sound);
}

void stop_sound(sound_t sound)
{
  // stop sound if currently playing
  if(dfplayer_is_busy()) {
    if(sound == match_state.current_sound) {
      dfplayer_pause();
    }
  }
  match_state.current_sound = SOUND_NONE;
  match_state.current_sound_started = false;
}


static void draw_score(void)
{
  static uint16_t previous_total_score = 0;
  static uint16_t displayed_score = 0;
  uint16_t total_score = match_state.scores.galipeur + match_state.scores.galipette;
  if(total_score > 999) {
    total_score = 999;
  }

  // water
  //   10 for open recuperator
  //   5 points for our color in water tower
  //   5 points for opponent color in in treatment plant
  // buildings
  //   N points for cube at level N
  //   30 points for building matching schema
  // automation panel
  //   5 points for being there
  //   25 points for being on at end of match
  // bee
  //   5 points for being there
  //   50 for foraged flower

  if(total_score > previous_total_score) {
    // points gain
    match_state.celebration_duration = 40;
    if(total_score - previous_total_score == 50) {
      // when launching the bee, play a special song!
      play_sound(SOUND_BEE_LAUNCHED);
    } else {
      play_sound(SOUND_POINTS_GAIN);
    }
    previous_total_score = total_score;
  } else if(total_score < previous_total_score) {
    // points loss (don't celebrate)
    play_sound(SOUND_POINTS_LOSS);
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

static void draw_scrolling_debug_team(void)
{
  static scrolling_text_t scroll;
  if(scroll.text_width == 0) {
    scrolling_text_init(&scroll, &font_base, "DEBUG TEAM  ", 2);
  }
  scrolling_text_draw(&scroll, screen, 0);
  FOREACH_RECT_PIXEL(screen, screen_upper_rect) {
    if(p->r) {
      *p = RGB(0x10,0x10,0x40);
    }
  }
  scrolling_text_scroll(&scroll, -1);
}

static void draw_yellow_lines(void)
{
  static uint8_t yellow_offset = 0;
  yellow_offset++;
  FOREACH_PIXEL(screen) {
    uint8_t offset = yellow_offset / 4;
    if(p->r == 0 && p->g == 0 && p->b == 0) {
      if(((offset + x + y) / 4) % 2 == 0) {
        *p = RGB(0x06,0x06,0);
      }
    }
  }
}

static void draw_lower_image(const image_t *image)
{
  draw_pixels_pgm(screen, (SCREEN_LW - image->width)/2, SCREEN_UH + (SCREEN_LH - image->height)/2, image->width, image->height, image->data);
}

static void draw_nyancat(void)
{
  static uint8_t tick;
  tick++;
  uint8_t frame = tick / 8 % 2;
  if(frame == 0) {
    draw_pixels_pgm(screen, SCREEN_UW - image_nyancat1.width, 0, image_nyancat1.width, image_nyancat1.height, image_nyancat1.data);
  } else {
    draw_pixels_pgm(screen, SCREEN_UW - image_nyancat2.width, 0, image_nyancat2.width, image_nyancat2.height, image_nyancat2.data);
  }

  static const pixel_t rainbow_colors[] = {
    {0x00,0x00,0x00},
    {0x18,0x00,0x00},
    {0x14,0x04,0x00},
    {0x10,0x10,0x00},
    {0x00,0x18,0x00},
    {0x00,0x10,0x10},
    {0x10,0x00,0x10},
    {0x00,0x00,0x00},
  };

  // rainbow
  FOREACH_RECT_PIXEL(screen, RECT(0, 0, 12, 7)) {
    if(p->r == 0 && p->g == 0 && p->b == 0) {
      uint8_t color = y + (((tick / 4) % 2) ^ ((x / 3) % 2));
      *p = rainbow_colors[color];
    }
  }

  // sky
  FOREACH_PIXEL(screen) {
    if(p->r == 0 && p->g == 0 && p->b == 0) {
      *p = RGB(0,0,4);
    }
  }

  // stars
  static const uint8_t initial_dx[] = {1, 5, 7, 12};
  typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t prescaler;
  } star_t;
  static star_t stars[6] = {0};
  for(uint8_t i = 0; i < sizeof(stars)/sizeof(*stars); i++) {
    star_t *star = &stars[i];
    if(star->x == 0) {
      star->x = SCREEN_LW + initial_dx[rand() % sizeof(initial_dx)/sizeof(*initial_dx)];
      star->y = SCREEN_UH + 1 + (rand() % (SCREEN_LH - 1));
      star->prescaler = rand() % 3;
    }
    if(star->x < SCREEN_UW) {
      *TEXTURE_PIXEL(screen, star->x, star->y) = GRAY(1 << (7 - star->prescaler));
    }
    if(tick % (1 << star->prescaler) == 0) {
      star->x--;
    }
  }

  if(!dfplayer_is_busy()) {
    dfplayer_send_cmd(DF_PLAY_FOLDER_TRACK, TRACK_MUSICS_NYAN_CAT);
  }
}

static void draw_loituma(void)
{
  static uint8_t tick;
  static const image_t *leeks[] = {
    &image_loituma_leek_n,
    &image_loituma_leek_ne,
    &image_loituma_leek_e,
    &image_loituma_leek_se,
    &image_loituma_leek_s,
    &image_loituma_leek_sw,
    &image_loituma_leek_w,
    &image_loituma_leek_nw,
  };
  tick++;

  // head
  if((tick / 8) % 2 == 0) {
    draw_pixels_pgm(screen, SCREEN_LW + 1, 0, image_loituma_head_open.width, image_loituma_head_open.height, image_loituma_head_open.data);
  } else {
    draw_pixels_pgm(screen, SCREEN_LW + 1, 0, image_loituma_head_closed.width, image_loituma_head_closed.height, image_loituma_head_closed.data);
  }

  // leek
  const image_t *img = leeks[tick % sizeof(leeks)/sizeof(*leeks)];
  draw_pixels_pgm(screen, 1, SCREEN_UH + 1, img->width, img->height, img->data);

  if(!dfplayer_is_busy()) {
    dfplayer_send_cmd(DF_PLAY_FOLDER_TRACK, TRACK_MUSICS_LOITUMA_IEVAS_POLKA_ORIGINAL);
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


static void update_display(void)
{
  static uint8_t bug_frame;
  texture_clear(screen);

  if(boomotter_mode == ROME_ENUM_BOOMOTTER_MODE_NYANCAT) {
    draw_nyancat();
  } else if(boomotter_mode == ROME_ENUM_BOOMOTTER_MODE_LOITUMA) {
    draw_loituma();

  } else if(match_state.timer_last_update == 0) {
    // stand mode
    draw_scrolling_debug_team();
    if((++bug_frame / 8) % 2 == 0) {
      draw_lower_image(&image_bug1);
    } else {
      draw_lower_image(&image_bug2);
    }
    draw_yellow_lines();
  } else {
    // match mode
    draw_score();
    draw_scrolling_robotter();
    draw_celebration();
  }

  // if robots have not updated the timer, play a special sound
  uint16_t uptime = uptime_us() / 1000000;
  if(match_state.timer_last_update != 0 && uptime >= match_state.timer_last_update + ROBOTS_ALIVE_TIMEOUT) {
    if(match_state.scores.galipeur != 0 || match_state.scores.galipette != 0) {
      play_sound(SOUND_ROBOTS_DEAD);
    }
    match_state.timer_last_update = 0;
  }

  // if battery is low, display a red rectangle
  if(battery_discharged) {
    draw_rect(screen, &(draw_rect_t){0, 0, 6, 6}, RGB(0x30, 0, 0));
    if(!dfplayer_is_busy()) {
      play_sound(SOUND_LOW_BATTERY);
    }
  }

  // retry in case of play_sound() miss
  if(match_state.current_sound != SOUND_NONE && !match_state.current_sound_started) {
    if(dfplayer_is_busy()) {
      match_state.current_sound_started = true;
    } else {
      start_sound(match_state.current_sound);
    }
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

    case ROME_MID_BOOMOTTER_MP3_CMD:
      dfplayer_send_cmd(frame->boomotter_mp3_cmd.cmd, frame->boomotter_mp3_cmd.param);
      rome_reply_ack(intf, frame);
      break;

    case ROME_MID_BOOMOTTER_SET_MODE:
      switch_mode(frame->boomotter_set_mode.mode);
      break;

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
      break;

    case ROME_MID_TM_MATCH_TIMER: {
      if(match_state.timer < MATCH_DURATION_SECS &&
         frame->tm_match_timer.seconds >= MATCH_DURATION_SECS) {
        match_state.after_match_uptime_s = uptime_us() / 1000000 + AFTER_MATCH_DELAY_SECS;
        play_sound(SOUND_MATCH_END);
      } else if(match_state.after_match_uptime_s != 0) {
        if(uptime_us() / 1000000 > match_state.after_match_uptime_s) {
          play_sound(SOUND_AFTER_MATCH);
          match_state.after_match_uptime_s = 0;
        }
      }
      match_state.timer = frame->tm_match_timer.seconds;
      match_state.timer_last_update = uptime_us() / 1000000;
    } break;

    case ROME_MID_MECA_TM_STATE: {
      rome_enum_meca_state_t new_state = frame->meca_tm_state.state;
      if(new_state != match_state.meca_state) {
        // meca state changed
        if(new_state == ROME_ENUM_MECA_STATE_BUSY) {
          play_sound(SOUND_COLLECTING_WATER);
        } else if(match_state.meca_state == ROME_ENUM_MECA_STATE_BUSY) {
          play_sound(SOUND_COLLECTING_WATER_END);
        }
        match_state.meca_state = new_state;
      }
    } break;

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

  play_sound(SOUND_BOOT);
  for(;;) {
    idle();
  }
}

