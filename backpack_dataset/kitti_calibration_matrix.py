"""Scripts pour calculer les matrices de calibration des caméras"""
import numpy as np

p_rect_03 = np.array(
    [
        [7.215377e+02, 0.000000e+00, 6.095593e+02, -3.395242e+02],
        [0.000000e+00, 7.215377e+02, 1.728540e+02, 2.199936e+00],
        [0.000000e+00, 0.000000e+00, 1.000000e+00, 2.729905e-03]
    ]
)

p_rect_02 = np.array(
    [
        [7.215377e02, 0.000000e00, 6.095593e02, 4.485728e01],
        [0.000000e00, 7.215377e02, 1.728540e02, 2.163791e-01],
        [0.000000e00, 0.000000e00, 1.000000e00, 2.745884e-03],
    ]
)
r_rect_00 = np.array(
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
r_rect_00 = np.concatenate(
    [r_rect_00, np.zeros(shape=(1, r_rect_00.shape[1]))], axis=0
)
# On ajoute une colonne de zeros à droite de rot_0_rect_matrix
r_rect_00 = np.concatenate(
    [r_rect_00, np.zeros(shape=(r_rect_00.shape[0], 1))], axis=1
)
r_rect_00[3, 3] = 1

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

calibration_matrix_cam2 = p_rect_02 @ r_rect_00 @ T_cam_velo
calibration_matrix_cam3 = p_rect_03 @ r_rect_00 @ T_cam_velo

if __name__ == "__main__":
    # print(calibration_matrix_cam2.shape, p_rect_02.shape,
    #       r_rect_00.shape, T_cam_velo.shape)
    print(calibration_matrix_cam2)
    print(calibration_matrix_cam2)