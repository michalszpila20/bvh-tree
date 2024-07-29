import logging
import numpy as np

def test_sphere_sphere(sphere_A, sphere_B):

    center_A = np.array([sphere_A.centre(0), sphere_A.centre(1), sphere_A.centre(2)])
    center_B = np.array([sphere_B.centre(0), sphere_B.centre(1), sphere_B.centre(2)])

    d = center_A - center_B
    dist2 = np.dot(d, d)

    logging.debug(f"dist2: {dist2}")

    radius_sum = sphere_A.radius + sphere_B.radius

    logging.debug(f"radius_sum: {radius_sum}")

    logging.debug(f"dist2 <= radius_sum * radius_sum: {dist2 <= radius_sum * radius_sum}")

    return dist2 <= radius_sum * radius_sum

def collision_detection_sphere(node_list_A, node_list_B):
    logging.debug("collision_detection_sphere")