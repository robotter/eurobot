#ifndef _R3D2_H_
#define _R3D2_H_

void r3d2_init(void);

void r3d2_update(void);

/** @brief Set rotation parameters
 * @param speed mirror rotation speed in turns per minute
 * @param threshold speed valid threshold in percent of rotation speed consign
 */
void r3d2_set_rotation(uint16_t speed, uint8_t threshold);

/** @brief Set R3D2 blind spot, angle is defined positively from begin to end
 */
void r3d2_set_blind_spot(float begin, float end);

#endif//_R3D2_H_
