/** @brief Define robot motors scales */
#ifndef MOTORS_SCALES_H
#define MOTORS_SCALES_H

#if defined(BUILD_GALIPEUR)
#define M (24.0/32.0)
static const float motors_scales[] = {M, M, M};
static const float encoders_scales[] = {1.0, 1.0, 1.0};
#elif defined(BUILD_GALIPETTE)
static const float motors_scales[] = {2.0, 1.0, 1.0};
static const float encoders_scales[] = {0.5, 1.0, 1.0};
#else
# error "Please define either BUILD_GALIPEUR or BUILD_GALIPETTE"
#endif

#endif//MOTORS_SCALES_H
