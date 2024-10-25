import numpy as np
import pyvista as pv
from PIL import Image

import ros2_ws.src.kitti_pointcloud_publisher.kitti_pointcloud_publisher.kitti_calibration_matrix as kitti_calibration_matrix
import ros2_ws.src.kitti_pointcloud_publisher.kitti_pointcloud_publisher.tools as tools


def pcd_with_hue_diff(bin_file, img1_file, img2_file, calib_matrix1, calib_matrix2) -> np.ndarray:
    """Calcul de la différence de teinte à partir des fichiers source"""
    pcd = tools.load_point_cloud(bin_file)
    calib = (calib_matrix1, calib_matrix2)
    imgs = tools.load_image(img1_file), tools.load_image(img2_file)
    pcd_colors = [tools.get_point_cloud_colors(
        pcd, imgs[i], calib[i]) for i in range(2)]
    pcd_hsv = [tools.colors_to_hsv(pcd_colors[i]) for i in range(2)]
    h_diff = tools.hue_diff(pcd_hsv[0], pcd_hsv[1])
    return np.hstack([pcd, h_diff])


if __name__ == "__main__":
    import os
    DATA_PATH = r"/home/samir/Downloads/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync/"
    PCD_PATH = DATA_PATH + "velodyne_points/data/"
    IMG2_PATH = DATA_PATH + "image_02/data/"
    IMG3_PATH = DATA_PATH + "image_03/data/"
    # Change this path to the directory containing KITTI point cloud data
    kitti_pcd_dir = PCD_PATH
    kitti_img2_dir = IMG2_PATH
    kitti_img3_dir = IMG3_PATH

    # List all files in the directory
    pcd_files = os.listdir(kitti_pcd_dir)
    img2_files = os.listdir(kitti_img2_dir)
    img3_files = os.listdir(kitti_img3_dir)

    # Sort the files numerically
    pcd_files.sort()
    pcd_files = [kitti_pcd_dir + f for f in pcd_files]
    img2_files.sort()
    img2_files = [kitti_img2_dir + f for f in img2_files]
    img3_files.sort()
    img3_files = [kitti_img3_dir + f for f in img3_files]

    pcd_hdiff = tools.pcd_with_hue_diff(
        bin_file=pcd_files[0],
        img1_file=img2_files[0],
        img2_file=img3_files[0],
        calib_matrix1=kitti_calibration_matrix.calibration_matrix_cam2,
        calib_matrix2=kitti_calibration_matrix.calibration_matrix_cam3
    )

    pv_pcd = pv.PolyData(pcd_hdiff[:, :3])
    pv_pcd["hue difference"] = pcd_hdiff[:, 3]
    pv_pcd.plot()
