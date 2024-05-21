"""Script pour afficher un nuage de point à partir d'un fichier .bin de KITTI"""

import struct
import numpy as np
import open3d
import kitti_pointcloud_publisher.kitti_pointcloud_publisher.tools as zb
from PIL import Image
from kitti_calibration_matrix import calibration_matrix_cam2


def convert_kitti_bin_to_pcd(binFilePath):
    size_float = 4
    list_pcd = []
    with open(binFilePath, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(np_pcd)
    return pcd


def color(bin_File, img_file):
    # Load image
    image = Image.open(img_file)
    image_array = np.array(image)
    # Load point cloud
    pcd = convert_kitti_bin_to_pcd(bin_File)
    pcd.points = zb.colorize_point_cloud(
        point_cloud=pcd.points,
        image=image_array,
        calibration_matrix=calibration_matrix_cam2,
    )
    return pcd


if __name__ == "__main__":
    # source
    bin_file = "/home/eleve/pidr/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    img_file = "/home/eleve/pidr/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/image_02/data/0000000018.png"

    # convert
    pcd_ = convert_kitti_bin_to_pcd(bin_file)

    # show
    # print(pcd_)
    pcd = color(bin_file,img_file)
    # print(np.asarray(pcd_.points))
    open3d.visualization.draw_geometries([pcd_])

    print(pcd.shape)
