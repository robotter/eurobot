#ifndef COLOR_DEFS_H
#define COLOR_DEFS_H

typedef enum{
  UNDEFINED_COLOR = 0x00u,                // unknown color
  FRUIT_COLOR,                            // green yellow   - fruit that earn points
  TOXIC_FRUIT_BLACK_LINE_COLOR,           // dark black     - line and toxic fruit color
  TEAM_A_COLOR,                           // trafic yellow  - color of the first team 
  TEAM_B_COLOR,                           // trafic red     - color of the first team 
  MAMMOTH_HEART_OF_FIRE_MOBILE_TORCHES_COLOR,   // security brown - color of mammoth, heart of fire, mobile torches, and trunks
  FRESCO_CAVE_FIXED_TORCHES_COLOR,        // flint grey     - fresco, caves, fixed torches

  /////////////////////////////////////////////////
  // warning : MUST be the last element of the enum
  COLOR_T_ELEMENT_NUMBER,
}Color_t;

#endif //COLOR_DEFS_H
