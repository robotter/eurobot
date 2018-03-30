/** 
 * @brief Fletcher16 checksum for FNS network layer 
 * @author JD <jdbrossillon@fly-n-sense.com>
 */
#ifndef FLETCHER_16_H
#define FLETCHER_16_H

#include <inttypes.h>

typedef struct {
  uint8_t hi;
  uint8_t lo;

}fletcher16_t;

/** @brief Update checkum computing */
static inline void fletcher16_update(fletcher16_t *csum, char c) {
  csum->lo += (uint8_t)c;
  csum->hi += csum->lo;
}

/** @brief Update checksum with a memory block */
static inline void fletcher16_update_block(fletcher16_t *csum, const char *block, uint8_t sz) {
  int i;
  for(i=0;i<sz;i++)
    fletcher16_update(csum, block[i]);
}

/** @brief Reset checksum computing */
static inline void fletcher16_reset(fletcher16_t *csum) {
  csum->lo = 0;
  csum->hi = 0;
}

/** @brief Get 16 bits checksum */
static inline uint16_t fletcher16_get(fletcher16_t *csum) {
  return (((uint16_t)csum->hi<<8) + csum->lo);
}

#endif/*FLETCHER_16_H*/
