/** @brief Modulo algorithm template
 * @author JD <jdbrossillon@fly-n-sense.com>
 */

#define F___(x) x##_modulo__
#define F__(x) F___(x)

static inline ANGLE_TYPE__ F__(ANGLE_TYPE__)(ANGLE_TYPE__ x, const ANGLE_TYPE__ a, const ANGLE_TYPE__ b) {
  const ANGLE_TYPE__ range = b-a;
  while(x >= b)
    x -= range;
  while(x < a)
    x += range;
  return x;
}

#undef F__
#undef F___
