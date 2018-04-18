#include <avr/io.h>
#include <avarix.h>
#include <uart/uart.h>
#include "config.h"
#include "jevois_cam.h"
#include <string.h>
#include <avarix/portpin.h>
#include <clock/clock.h>
#include <util/delay.h>
#include "fletcher16.h"

#include <timer/timer.h>
#include <timer/uptime.h>

static rome_enum_jevois_color_t enum_from_char(char c) {
  switch(c) {
    case 'G':
      return ROME_ENUM_JEVOIS_COLOR_GREEN;

    case 'O':
      return ROME_ENUM_JEVOIS_COLOR_ORANGE;

    default:
      return ROME_ENUM_JEVOIS_COLOR_NONE;
  }
}

static void process_frame(jevois_cam_t *cam) {

  jevois_frame_t frame;
  memcpy(&frame, cam->buffer, sizeof(frame));

  fletcher16_t csum;
  fletcher16_reset(&csum);
  fletcher16_update_block(&csum, cam->buffer, sizeof(cam->buffer) - sizeof(uint16_t));

  // checksum is valid
  //if(frame.checksum == fletcher16_get(&csum)) {
  //  portpin_outtgl(&LED_AN_PP(1));
  //}

  cam->received_ts = uptime_us();

  cam->entry_color = enum_from_char(frame.entry_color);
  cam->cylinder_color = enum_from_char(frame.cylinder_color);
  cam->entry_height = frame.entry_height;

}

static void update_frame(jevois_cam_t *cam, char c) {

  switch(cam->state) {
    case JEVOIS_FRAME_STATE_WAITING_FOR_START:
      if(c == 0x55) {
        portpin_outtgl(&LED_AN_PP(0));
        cam->buffer_it = 0;
        cam->buffer[cam->buffer_it++] = c;
        cam->state = JEVOIS_FRAME_STATE_FEEDING_BUFFER;
      }
      break;

    case JEVOIS_FRAME_STATE_FEEDING_BUFFER:
      // feed buffer
      cam->buffer[cam->buffer_it++] = c;
      if(cam->buffer_it >= sizeof(cam->buffer)) {
        cam->state = JEVOIS_FRAME_STATE_WAITING_FOR_START;

        // process buffer
        process_frame(cam);
      }

    default:
      // XXX put some ERROR here
      break;

  }
}

void jevois_cam_init(jevois_cam_t *cam) {

  // clear buffer
  memset(cam->buffer, 0, sizeof(cam->buffer));
  cam->buffer_it = 0;

  cam->received_ts = 0;
  cam->state = JEVOIS_FRAME_STATE_WAITING_FOR_START;
}

void jevois_cam_update(jevois_cam_t *cam) {
  int n = 0;
  int16_t c;
  while((c = uart_recv_nowait(UART_JEVOIS)) >= 0) {
    update_frame(cam,c);
    n++;
    // in case jevois is flooding, limit batch read to 1024
    if(n >= 1024)
      break;
  }
}

uint8_t jevois_cam_is_valid(jevois_cam_t* cam) {
  return cam->received_ts + JEVOIS_CAM_TIMEOUT_US > uptime_us();
}

uint16_t jevois_cam_get_entry_height(jevois_cam_t* cam) {
  return cam->entry_height;
}

rome_enum_jevois_color_t jevois_cam_get_entry_color(jevois_cam_t* cam) {
  return cam->entry_color;
}

rome_enum_jevois_color_t jevois_cam_get_cylinder_color(jevois_cam_t* cam) {
  return cam->cylinder_color;
}
