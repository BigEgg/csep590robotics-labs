from grid import *
from particle import Particle
from utils import *
from setting import *


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
    for particle in particles:
        #   Move particle in the exact same way, but add noise to each movement
        (dx, dy, dh) = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        # (dx, dy, dh) = odom
        #   Rotate the *robot frame* location to *world frame*
        (dx, dy) = rotate_point(dx, dy, dh)
        motion_particles.append(Particle(particle.x + dx, particle.y + dy, particle.h + dh))
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
                * Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    weights = []
    has_weight = False
    for particle in particles:
        particle_measured_marker = particle.read_markers(grid)
        likely_hood = 0
        for measurement1 in measured_marker_list:
            for measurement2 in particle_measured_marker:
                likely_hood += measurement_prob(measurement1, measurement2)
        if likely_hood != 0:
            has_weight = True
        weights.append(likely_hood)

    #resample 90% of the particle, and add 10% random particle uni
    resample_rate = 0.9
    if not has_weight:
        #   In some case their had no weight after the calculation
        measured_particles = random.choices(particles, k=(int)(resample_rate * len(particles)))
    else:
        measured_particles = random.choices(particles, weights=weights, k=(int)(resample_rate * len(particles)))
    measured_particles.extend(Particle.create_random((int)((1 - resample_rate) * len(particles)), grid))

    return measured_particles


def measurement_prob(measurement1, measurement2):
    (x0, y0, h0) = measurement1
    (x1, y1, h1) = measurement2
    (delta_x, delta_y, delta_h) = (x1 - x0, y1 - y0, h1 - h0)

    dist = math.sqrt(delta_x ** 2 + delta_y ** 2)
    return gaussian(0, MARKER_TRANS_SIGMA, delta_x) * gaussian(0, MARKER_TRANS_SIGMA, delta_y) * gaussian(0, MARKER_ROT_SIGMA, delta_h)


def gaussian(mu, sigma, x):
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    # Probability density function:
    #                                                         (x - mu) ^ 2
    #                                                    (-) --------------
    #                                   1                     2 *sigma ^ 2
    # f(x | mu, sigma ^ 2) = ------------------------- e
    #                        sqrt(2 * pi * sigma ^ 2)
    return math.exp(- ((mu - x) ** 2) / (2 * sigma ** 2)) / math.sqrt(2.0 * math.pi * (sigma ** 2))

