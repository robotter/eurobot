#include <avarix.h>
#include <timer/uptime.h>
#include "jevois_cam.h"
#include "config.h"


void jevois_cam_process_rome(jevois_cam_t *cam, const rome_frame_t *frame) {
  switch(frame->mid) {
    case ROME_MID_JEVOIS_TM_CYLINDER_CAM: {
      cam->received_ts = uptime_us();
      cam->entry_color = frame->jevois_tm_cylinder_cam.entry_color;
      cam->cylinder_color = frame->jevois_tm_cylinder_cam.cylinder_color;
      cam->entry_height = frame->jevois_tm_cylinder_cam.entry_height;
    } break;

    default:
      break;
  }
}

void jevois_cam_init(jevois_cam_t *cam) {
  cam->received_ts = 0;
}

uint8_t jevois_cam_is_valid(jevois_cam_t *cam) {
  return cam->received_ts + JEVOIS_CAM_TIMEOUT_US > uptime_us();
}

uint16_t jevois_cam_get_entry_height(jevois_cam_t *cam) {
  return cam->entry_height;
}

rome_enum_jevois_color_t jevois_cam_get_entry_color(jevois_cam_t *cam) {
  return cam->entry_color;
}

rome_enum_jevois_color_t jevois_cam_get_cylinder_color(jevois_cam_t *cam) {
  return cam->cylinder_color;
}

