/*
 * @brief calibration parameters files
 * @author Julien Metge <jmetge@gmail.com>
 * automatically generated file 2013-04-28 15:36:09.777205
 */
#ifndef HROBOT_MANAGER_CONFIG_H
#define HROBOT_MANAGER_CONFIG_H


#ifdef BUILD_GALIPEUR

double hrobot_motors_matrix[9] = {
0.137193775559,-0.227742535811,32.7587578324,
-0.267514745628,0.000225842067981,32.2910980339,
0.138273262887,0.235015679279,32.2670974911,
};

#elif defined(BUILD_GALIPETTE)

double hrobot_motors_matrix[9] = {
0.137437213978,-0.227742741327,32.7293729332,
-0.257911181339,-0.0157161354388,34.5842110822,
0.13815905366,0.235049919704,32.2734855617,
};



#endif

#if 0
double hrobot_motors_matrix_correct[9] = {
  1.0,           -0.000910,        0.0,
  0.07,          1.0,              0.0,
  -0.00015,      0.000350,         1.0};
  #endif

#endif//HROBOT_MANAGER_CONFIG_H
