import numpy as np
from implementation import compute_min_n_by_polygon_shrinkage

velocities = np.array([[-1.94500000e+01,  2.38193802e-15, -1.53060000e+02,  1.87444439e-14], [ 2.72,  0.  , 16.62,  0.  ], [-4.99655894e-16, -2.72000000e+00, -3.05304447e-15, -1.66200000e+01], [1.19096901e-15, 1.94500000e+01, 9.37222195e-15, 1.53060000e+02]])
durations = np.array([np.float64(4.557737869664243), np.float64(41.79324921627044), np.float64(167.21759832767611), np.float64(16.98422507024444)])
x0 = np.array([-120.92317891, 399.53687397, 26.25108255, 344.47456885])
box_lower = -1*np.array([420,350,420,350])
box_upper = np.array([420,350,420, 350])



n = compute_min_n_by_polygon_shrinkage(velocities, durations, x0, box_lower, box_upper)
print(n)