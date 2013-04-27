#ifndef CAKE_H__
#define CAKE_H__

#include <encoder/aeat/aeat.h>


// Cake radius in millimeters, to compute angle position along the cake
#define CAKE_RADIUS  500
/// Encoder radius, in millimeters
#define CAKE_ENCODER_RADIUS  15


/// Initialize the cake encoder
void cake_init(void);

/// Set position along the cake, in milliradians
void cake_set_angle(int16_t a);
/// Get position along the cake, in milliradians
int16_t cake_get_angle(void);


#endif
