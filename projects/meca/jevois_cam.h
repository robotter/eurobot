#ifndef JE_VOIS_CAM_H
#define JE_VOIS_CAM_H

#include <avarix.h>
#include <avr/io.h>
#include <rome/rome.h>

typedef struct __attribute__((__packed__)) {

  char start;
  char entry_color;
  char cylinder_color;
  uint16_t entry_height;
  uint32_t entry_area;
  uint32_t cylinder_area;

  uint16_t checksum;

} jevois_frame_t;

typedef enum {

  JEVOIS_FRAME_STATE_WAITING_FOR_START,
  JEVOIS_FRAME_STATE_FEEDING_BUFFER,

} jevois_frame_state_t;

typedef struct {

  char buffer[sizeof(jevois_frame_t)];

  unsigned int buffer_it;

  jevois_frame_state_t state;

  uint32_t received_ts;

  uint16_t entry_height;
  rome_enum_jevois_color_t entry_color;
  rome_enum_jevois_color_t cylinder_color;

}jevois_cam_t;

#define JEVOIS_UART uartF1

#define JEVOIS_CAM_TIMEOUT_US 1000000

void jevois_cam_init(jevois_cam_t*);

void jevois_cam_update(jevois_cam_t*);

uint16_t jevois_cam_get_entry_height(jevois_cam_t*);

rome_enum_jevois_color_t jevois_cam_get_entry_color(jevois_cam_t*);

rome_enum_jevois_color_t jevois_cam_get_cylinder_color(jevois_cam_t*);

uint8_t jevois_cam_is_valid(jevois_cam_t*);

#endif//JE_VOIS_CAM_H
