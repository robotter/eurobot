/*
 * @brief calibration parameters files
 * @author Julien Metge <jmetge@gmail.com>
 * automatically generated file 2013-04-28 15:36:09.777484
 */
#ifndef HPOSITION_MANAGER_CONFIG_H
#define HPOSITION_MANAGER_CONFIG_H

#ifdef GALIPEUR

double hrobot_motors_invmatrix[9] = {
-1.24627114282,2.4735001584,-1.21007913871,
2.15287736186,-0.0169008404017,-2.16876778164,
-0.0103397573436,-0.010476522571,-0.0100097003094,
};

double hrobot_motors_invmatrix_correct[9] = {
  1.0,      0.0,        0.0,
  0.0,      1.0,        0.0,
  0.0,      0.0,        1.0
  };

#elif defined(GALIPETTE)
double hrobot_motors_invmatrix[9] = {
 -1.327571, 2.731647, -1.281436,
 2.233109, 0.092876, -2.405414,
 -0.016462, -0.015989, -0.015794,
};

double hrobot_motors_invmatrix_correct[9] = {
  1.0,      0.0,        0.0,
  0.0,      1.0,        0.0,
  0.0,      0.0,        1.0
  };

#endif

#endif//HPOSITION_MANAGER_CONFIG_H
