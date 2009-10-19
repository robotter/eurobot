// Variables to configure the project

// Author: RB



#ifndef __CONFIG_H__
#define __CONFIG_H__

#define NB_TRANSDUCTOR 3


/*
  A variable to convert a MIDI note value to a frequency
*/
float note2freq[128]={0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
27.50,
29.13,
30.86,
32.70,
34.64,
36.70,
38.89,
41.20,
43.65,
46.24,
48.99,
51.91,
55.00,
58.27,
61.73,
65.40,
69.29,
73.41,
77.78,
82.40,
87.30,
92.49,
97.99,
103.82,
110.00,
116.54,
123.47,
130.81,
138.59,
146.83,
155.56,
164.81,
174.61,
184.99,
195.99,
207.65,
220.00,
233.08,
246.94,
261.62,
277.18,
293.66,
311.12,
329.62,
349.22,
369.99,
391.99,
415.30,
440.00,
466.16,
493.88,
523.25,
554.36,
587.32,
622.25,
659.25,
698.45,
739.98,
783.99,
830.60,
880.00,
932.32,
987.76,
1046.50,
1108.73,
1174.65,
1244.50,
1318.51,
1396.91,
1479.97,
1567.98,
1661.21,
1760.00,
1864.65,
1975.53,
2093.00,
2217.46,
2349.31,
2489.01,
2637.02,
2793.82,
2959.95,
3135.96,
3322.43,
3520.00,
3729.31,
3951.06,
4186.01,
4434.92,
4698.64,
4978.03,
5274.04,
5587.65,
5919.91,
6271.93,
6644.87,
7040.00,
7458.62,
7902.13,
8372.02,
8869.84,
9397.27,
9956.06,
10548.08,
11175.30,
11839.82,
12543.85};


/*
  convert x into PWM width
  where x can be the second harmonic depending on the instrument or the output power depending on the velocity
  here is the power function
*/
float x2pwm_width[128]={
0.00000,
0.00251,
0.00501,
0.00752,
0.01003,
0.01254,
0.01504,
0.01755,
0.02006,
0.02258,
0.02509,
0.02760,
0.03012,
0.03264,
0.03516,
0.03768,
0.04021,
0.04274,
0.04527,
0.04780,
0.05034,
0.05288,
0.05542,
0.05797,
0.06052,
0.06307,
0.06563,
0.06819,
0.07076,
0.07333,
0.07591,
0.07849,
0.08108,
0.08367,
0.08627,
0.08887,
0.09148,
0.09410,
0.09672,
0.09935,
0.10199,
0.10464,
0.10729,
0.10995,
0.11262,
0.11529,
0.11798,
0.12067,
0.12337,
0.12608,
0.12881,
0.13154,
0.13428,
0.13703,
0.13980,
0.14257,
0.14536,
0.14816,
0.15097,
0.15379,
0.15663,
0.15948,
0.16234,
0.16522,
0.16812,
0.17103,
0.17395,
0.17689,
0.17985,
0.18283,
0.18582,
0.18884,
0.19187,
0.19492,
0.19799,
0.20109,
0.20421,
0.20735,
0.21051,
0.21370,
0.21691,
0.22015,
0.22342,
0.22672,
0.23004,
0.23340,
0.23679,
0.24022,
0.24367,
0.24717,
0.25070,
0.25427,
0.25789,
0.26155,
0.26525,
0.26900,
0.27280,
0.27666,
0.28057,
0.28454,
0.28857,
0.29267,
0.29684,
0.30109,
0.30541,
0.30983,
0.31433,
0.31893,
0.32364,
0.32846,
0.33341,
0.33849,
0.34373,
0.34913,
0.35472,
0.36051,
0.36654,
0.37284,
0.37945,
0.38642,
0.39382,
0.40177,
0.41038,
0.41990,
0.43068,
0.44343,
0.46003,
0.50000};


#endif /* __CONFIG_H__ */
