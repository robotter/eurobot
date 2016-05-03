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
1.37437214e-01,  -2.27742741e-01, 3.27293729e+01,
-2.57911181e-01,  -1.57161354e-02, 3.45842111e+01,
1.38110951e-01,  2.35129630e-01,  3.22620303e+01
};

double hrobot_motors_matrix_correct[9] = {
  1.0,      0.0,        0.0,
  0.0,      1.0,        0.0,
  0.0,      0.0,        1.0
};

#endif

#endif//HROBOT_MANAGER_CONFIG_H
