#!/usr/bin/python
# @brief calibration du robot pour la localisation et l'asserv
# @author Julien Metge <jmetge@gmail.com>
# Copyright Robotter 2013

import os, datetime
import math, numpy, sympy
import string
from numpy import array, dot, transpose
from numpy.linalg import inv, pinv
from math import pi, cos, sin

def test_simulation():
  # deplacement (x,y,theta)
  X = array([[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1],
             [1, 2, 0]]).transpose()
  # corresponding encoder measurement (enc0,enc1,enc2)
  Y = array([[5, 1, 1],
             [2, 5, 1],
             [0, 3, 5],
             [3, 2, 1]]).transpose()
  
  # A to test and generate theorical Y
  A = array([[2, 1, 0],
             [0, 3, 0],
             [1, 2, 4]]).transpose()
  Y = dot(A,X)
  

  # Model : Y = AX
  A = dot(Y,pinv(X))

  print "X\n", X
  print "Y\n", Y
  print "A\n", A
  print "inv(A)\n", inv(A)
  print "Y2=AX\n", dot(A,X)
  print "X2=inv(A)Y\n", dot(inv(A),Y)

def write_calib_file(filename, matrix_name, matrix):
  print "write ", filename
  now = datetime.datetime.now()

  f = open(filename,'w')
  f.write('/*\n'
          ' * @brief calibration parameters files\n'
          ' * @author Julien Metge <jmetge@gmail.com>\n'
          ' * automatically generated file ' + str(now) + '\n'
          ' */\n'
          '#ifndef ' + os.path.basename(filename).upper().replace('.','_') + '\n'
          '#define ' + os.path.basename(filename).upper().replace('.','_') + '\n'
          '\n')

  f.write('double ' + matrix_name + '[' + str(matrix.size) + '] = {\n')
  for r in matrix:
    for e in r:
      f.write(str(e) + ',')
    f.write('\n')
  f.write('};\n\n')
   

  f.write('#endif//' + os.path.basename(filename.upper()).replace('.','_') + '\n')
  f.close()


if __name__ == '__main__':
  #test_simulation() 

  # deplacement (x,y,theta)
  X = array([[0, 0, 2*2*pi],
             [0, 0, -2*2*pi],
             [0, 0, 5*2*pi],
             [0, 0, -5*2*pi],
             [1000, 0, 0],
             [1000, 0, 0],
             [-1000*cos(pi/3), -1000*sin(pi/3), 0],
             [-1000*cos(pi/3), -1000*sin(pi/3), 0]]).transpose()

  # corresponding encoder measurement (enc0,enc1,enc2)
  Y = array([[-399508, -392954, -393487],
             [400259, 393746, 393726],
             [-998047, -981852, -982792],
             [998038, 986397, 983555],
             [-122679, 250142, -123203],
             [-121900, 249835, -123471],
             [-119967, -124680, 249145],
             [-119962, -124621, 249365]]).transpose()
  
  # Model : Y = AX
  A = dot(Y,pinv(X))

  write_calib_file("../hrobot_manager_config.h", "hrobot_motors_matrix", A)
  write_calib_file("../hposition_manager_config.h", "hrobot_motors_invmatrix", inv(A))

  print "X\n", X
  print "Y\n", Y
  print "A\n", A
  print "Y2=AX\n", dot(A,X)
  print "X2=inv(A)Y\n", dot(inv(A),Y)

