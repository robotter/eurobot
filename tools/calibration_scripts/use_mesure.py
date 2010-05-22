import re
import numpy
import sys
from numpy      import dot as dot
from copy       import deepcopy

f = open(sys.argv[1], 'r')
E = []
S = []
for line in f.readlines():
	m = re.search('([0-9.-]*), *([0-9.-]*), *([0-9.-]*), *([0-9.-]*), *([0-9.-]*), *([0-9.-]*), *([0-9.-]*), *([0-9.-]*), *([0-9.-]*)', line)
	groups = m.groups()
	mesure = map(float, groups)
	E.append(mesure[:3])
	S.append(mesure[3:])

E = numpy.array(E).T
E = E*pow(2, 14)
S = numpy.array(S).T

M =  numpy.linalg.lstsq(S.T, E.T)[0].T

print """
double hrobot_adnsMatrix_123[18] = {
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f
};
""" % tuple(M.reshape((1,18))[0])

nb_mesures = S[0].size

S12 = deepcopy(S)
S12[-1] = numpy.zeros(nb_mesures)
S12[-2] = numpy.zeros(nb_mesures)

M =  numpy.linalg.lstsq(S12.T, E.T)[0].T

print """
double hrobot_adnsMatrix_12[18] = {
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f
};
""" % tuple(M.reshape((1,18))[0])

S13 = deepcopy(S)
S13[-3] = numpy.zeros(nb_mesures)
S13[-4] = numpy.zeros(nb_mesures)

M =  numpy.linalg.lstsq(S13.T, E.T)[0].T

print """
double hrobot_adnsMatrix_13[18] = {
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f
};
""" % tuple(M.reshape((1,18))[0])


S23 = deepcopy(S)
S23[-5] = numpy.zeros(nb_mesures)
S23[-6] = numpy.zeros(nb_mesures)

M =  numpy.linalg.lstsq(S23.T, E.T)[0].T

print """
double hrobot_adnsMatrix_23[18] = {
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f,
%f,%f,%f,%f,%f,%f
};
""" % tuple(M.reshape((1,18))[0])

print dot(M,(S.T)[1].T), (E.T)[1].T

f.close()
