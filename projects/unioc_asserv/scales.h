/** @brief Define robot motors scales */
#ifndef MOTORS_SCALES_H
#define MOTORS_SCALES_H

#if defined(GALIPEUR)
#define M (24.0/16.0)
static const float motors_scales[] = {M, M, M};
static const float encoders_scales[] = {1.0, 1.0, 1.0};
#elif defined(GALIPETTE)
#define M (1.0)
#define N (2.0)
static float motors_scales[] = {M*2, M*1.0, M*1.0};
static const float encoders_scales[] = {-0.5*N, 1.0*N, 1.0*N};
#else
# error "Please define either GALIPEUR or GALIPETTE"
#endif

#endif//MOTORS_SCALES_H
