/*
 * @brief calibration parameters files
 * @author Julien Metge <jmetge@gmail.com>
 * automatically generated file 2013-04-28 15:36:09.777205
 */
#ifndef HROBOT_MANAGER_CONFIG_H
#define HROBOT_MANAGER_CONFIG_H


#ifdef GALIPEUR

double hrobot_motors_matrix[9] = {
0.137193775559,-0.227742535811,32.7587578324,
-0.267514745628,0.000225842067981,32.2910980339,
0.138273262887,0.235015679279,32.2670974911,
};

double hrobot_motors_matrix_correct[9] = {
  1.0,      0.0,        0.0,
  0.0,      1.0,        0.0,
  0.0,      0.0,        1.0
};

#elif defined(GALIPETTE)
double hrobot_motors_matrix[9] = {
 0.132510, -0.211187, 21.412129,
 -0.248472, 0.000420, 20.095279,
 0.113424, 0.219686, 20.654236,
};

double hrobot_motors_matrix_correct[9] = {
  1.0,      0.0,        0.0,
  0.0,      1.0,        0.0,
  0.0,      0.0,        1.0
};

#endif

#endif//HROBOT_MANAGER_CONFIG_H
