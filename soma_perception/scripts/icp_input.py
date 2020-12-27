import math
import numpy as np
import matplotlib.pyplot as plt
from icp import icp


if __name__ == '__main__':
    robot_pose = [0, 0, 0]
    

    # create a set of points to be the reference for ICP
    reference_points = np.array([
                                [2.3, 10.1], #1
                                [4.2, 8.1], #2
                                [5.1, 11.3] #3
                                ])

    # transform the set of reference points to create a new set of
    # points for testing the ICP implementation

    # 1. remove some points
    points_to_be_aligned = np.array([
                                    [-1.97, 5.68], #1
                                    [-0.09, 2.61], #2
                                    [1.25, 5.50] #3
                                    ])

    robot_pose[0] = reference_points[0][0] - points_to_be_aligned[0][0]
    robot_pose[1] = reference_points[0][1] - points_to_be_aligned[0][1]

    for n in range(len(points_to_be_aligned)):
      points_to_be_aligned[n][0] += robot_pose[0]
      points_to_be_aligned[n][1] += robot_pose[1]
    

    print('robot pose : ', robot_pose)


    # run icp
    #transformation_history, aligned_points = icp(reference_points, points_to_be_aligned, verbose=True)
    transformation_history, aligned_points, robot_pose_cor = icp(reference_points, points_to_be_aligned, robot_pose, verbose=True)

    print('robot pose_cor : ', robot_pose_cor)

    # show results
    plt.plot(reference_points[:, 0], reference_points[:, 1], 'rx', label='reference points')
    plt.plot(points_to_be_aligned[:, 0], points_to_be_aligned[:, 1], 'b1', label='points to be aligned')
    plt.plot(aligned_points[:, 0], aligned_points[:, 1], 'g+', label='aligned points')
    plt.plot(robot_pose[0], robot_pose[1], 'b^', label='robot pose')
    plt.plot(robot_pose_cor[0], robot_pose_cor[1], 'g^', label='robot pose ICP')
    plt.legend()
    plt.show()