#ifndef JE_VOIS_CAM_H
#define JE_VOIS_CAM_H

#include <avarix.h>
#include <rome/rome.h>

typedef struct {
  uint32_t received_ts;
  uint16_t entry_height;
  rome_enum_jevois_color_t entry_color;
  rome_enum_jevois_color_t cylinder_color;

} jevois_cam_t;

#define JEVOIS_CAM_TIMEOUT_US 1000000

void jevois_cam_init(jevois_cam_t*);
void jevois_cam_process_rome(jevois_cam_t*, const rome_frame_t *frame);
uint8_t jevois_cam_is_valid(jevois_cam_t*);

uint16_t jevois_cam_get_entry_height(jevois_cam_t*);
rome_enum_jevois_color_t jevois_cam_get_entry_color(jevois_cam_t*);
rome_enum_jevois_color_t jevois_cam_get_cylinder_color(jevois_cam_t*);

#endif //JE_VOIS_CAM_H
