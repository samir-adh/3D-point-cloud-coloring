"""Script pour afficher un nuage de point à partir d'un fichier .bin de KITTI"""

import struct
import numpy as np
from open3d import *


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


# source
bin_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000064.bin"

# convert
pcd_ = convert_kitti_bin_to_pcd(bin_file)

# show
print(pcd_)
print(np.asarray(pcd_.points))
open3d.visualization.draw_geometries([pcd_])

# save
open3d.io.write_point_cloud(
    "1644609772_916356000.pcd",
    pcd_,
    write_ascii=False,
    compressed=False,
    print_progress=False,
)
