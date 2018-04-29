
PARTICLE_COUNT = 5000       # Total number of particles in your filter

# odometry Gaussian noise model
ODOM_TRANS_SIGMA = 0.02     # translational err in inch (grid unit)
ODOM_HEAD_SIGMA = 2         # rotational err in deg

# marker measurement Gaussian noise model
MARKER_TRANS_SIGMA = 0.5    # translational err in inch (grid unit)
MARKER_ROT_SIGMA = 5        # rotational err in deg

PARTICLE_MAX_SHOW = 500     # Max number of particles to be shown in GUI (for speed up)

ROBOT_CAMERA_FOV_DEG = 45   # Robot camera FOV in degree
