import math
import numpy as np
import matplotlib.pyplot as plt
from icp import icp


if __name__ == '__main__':
    # set seed for reproducible results
    np.random.seed(12345)

    # create a set of points to be the reference for ICP
    reference_points = np.array([
                                [2.3, 10.1],
                                [4.2, 8.1],
                                [5.1, 11.3]
                                ])

    # transform the set of reference points to create a new set of
    # points for testing the ICP implementation

    # 1. remove some points
    points_to_be_aligned = np.array([
                                    [2.3, 10.1],
                                    [4.5, 7.8],
                                    [5.7, 11.5]
                                    ])

    # 2. apply rotation to the new point set
    theta = math.radians(12)
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s],
                    [s, c]])
    points_to_be_aligned = np.dot(points_to_be_aligned, rot)

    # run icp
    transformation_history, aligned_points = icp(reference_points, points_to_be_aligned, verbose=True)

    # show results
    plt.plot(reference_points[:, 0], reference_points[:, 1], 'rx', label='reference points')
    plt.plot(points_to_be_aligned[:, 0], points_to_be_aligned[:, 1], 'b1', label='points to be aligned')
    plt.plot(aligned_points[:, 0], aligned_points[:, 1], 'g+', label='aligned points')
    plt.legend()
    plt.show()