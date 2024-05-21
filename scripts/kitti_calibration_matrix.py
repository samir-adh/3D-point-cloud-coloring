import numpy as np

camera_index = 2

proj_i_rect_matrix = np.array(
    [
        [7.215377e02, 0.000000e00, 6.095593e02, 4.485728e01],
        [0.000000e00, 7.215377e02, 1.728540e02, 2.163791e-01],
        [0.000000e00, 0.000000e00, 1.000000e00, 2.745884e-03],
    ]
)
rot_0_rect_matrix = np.array(
    [
        [9.999239e-01, 9.837760e-03, -7.445048e-03],
        [-9.869795e-03, 9.999421e-01, -4.278459e-03],
        [7.402527e-03, 4.351614e-03, 9.999631e-01],
    ]
)


R_cam_velo = np.array(
    [
        [7.533745e-03, -9.999714e-01, -6.166020e-04],
        [1.480249e-02, 7.280733e-04, -9.998902e-01],
        [9.998621e-01, 7.523790e-03, 1.480755e-02],
    ]
)


t_velo_cam = np.array([-4.069766e-03, -7.631618e-02, -2.717806e-01]).T
t_velo_cam = np.reshape(t_velo_cam, (3, 1))


# Préparation de rot_0_rect_matrix

# On ajoute une ligne de zeros en bas de rot_0_rect_matrix
rot_0_rect_matrix = np.concatenate(
    [rot_0_rect_matrix, np.zeros(shape=(1, rot_0_rect_matrix.shape[1]))], axis=0
)
# On ajoute une colonne de zeros à droite de rot_0_rect_matrix
rot_0_rect_matrix = np.concatenate(
    [rot_0_rect_matrix, np.zeros(shape=(rot_0_rect_matrix.shape[0], 1))], axis=1
)
rot_0_rect_matrix[3,3] = 1

# Construction de T_cam_velo
T_cam_velo = np.concatenate(
    [
        np.concatenate([R_cam_velo, t_velo_cam], axis=1),
        np.concatenate(
            [
                np.zeros(shape=(1, R_cam_velo.shape[1])),
                np.ones(shape=(1, t_velo_cam.shape[1])),
            ],
            axis=1,
        ),
    ],
    axis=0,
)

calibration_matrix = proj_i_rect_matrix @ rot_0_rect_matrix @ T_cam_velo

if __name__ == "__main__":
    print(calibration_matrix.shape, proj_i_rect_matrix.shape, rot_0_rect_matrix.shape, T_cam_velo.shape)
    print(calibration_matrix)


# def get_rigid_transformation(calib_path):
#     """Obtains rigid transformation matrix in homogeneous coordinates (combination of
#     rotation and translation.
#     Used to obtain:
#         - LiDAR to camera reference transformation matrix
#         - IMU to LiDAR reference transformation matrix
#     """
#     with open(calib_path, "r") as f:
#         calib = f.readlines()

#     R = np.array([float(x) for x in calib[1].strip().split(" ")[1:]]).reshape((3, 3))
#     t = np.array([float(x) for x in calib[2].strip().split(" ")[1:]])[:, None]

#     T = np.vstack((np.hstack((R, t)), np.array([0, 0, 0, 1])))

#     return T


# calib_cam_to_cam = (
#     r"/home/samir/Desktop/PIDR/2011_09_26_calib/2011_09_26/calib_cam_to_cam.txt"
# )

# with open(calib_cam_to_cam, "r") as f:
#     calib = f.readlines()

# # get projection matrices (rectified left camera --> left camera (u,v,z))
# P_rect2_cam2 = np.array([float(x) for x in calib[25].strip().split(" ")[1:]]).reshape(
#     (3, 4)
# )


# # get rectified rotation matrices (left camera --> rectified left camera)
# R_ref0_rect2 = np.array([float(x) for x in calib[24].strip().split(" ")[1:]]).reshape(
#     (
#         3,
#         3,
#     )
# )

# # add (0,0,0) translation and convert to homogeneous coordinates
# R_ref0_rect2 = np.insert(R_ref0_rect2, 3, values=[0, 0, 0], axis=0)
# R_ref0_rect2 = np.insert(R_ref0_rect2, 3, values=[0, 0, 0, 1], axis=1)


# # get rigid transformation from Camera 0 (ref) to Camera 2
# R_2 = np.array([float(x) for x in calib[21].strip().split(" ")[1:]]).reshape((3, 3))
# t_2 = np.array([float(x) for x in calib[22].strip().split(" ")[1:]]).reshape((3, 1))

# # get cam0 to cam2 rigid body transformation in homogeneous coordinates
# T_ref0_ref2 = np.insert(np.hstack((R_2, t_2)), 3, values=[0, 0, 0, 1], axis=0)

# calib_velo_to_cam = (
#     r"/home/samir/Desktop/PIDR/2011_09_26_calib/2011_09_26/calib_velo_to_cam.txt"
# )

# T_velo_ref0 = get_rigid_transformation(calib_velo_to_cam)

# # transform from velo (LiDAR) to left color camera (shape 3x4)
# T_velo_cam2 = P_rect2_cam2 @ R_ref0_rect2 @ T_ref0_ref2 @ T_velo_ref0

# calibration_matrix = T_velo_cam2
