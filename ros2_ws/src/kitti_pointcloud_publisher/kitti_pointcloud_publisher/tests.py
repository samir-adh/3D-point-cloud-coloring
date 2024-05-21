import kitti_calibration_matrix
import numpy as np
from PIL import Image
import open3d as o3d
import matplotlib.pyplot as plt

def test_pcd_projection():
    # Chargement du nuage de points
    bin_file = "/home/eleve/pidr/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    point_cloud = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    projection_matrix = kitti_calibration_matrix.calibration_matrix_cam2
    coor = point_cloud_projection(
        projection_matrix=projection_matrix, point_cloud=point_cloud
    )
    print(coor.astype)


def test_pcd_colors():
    # Chargement du nuage de points
    bin_file = "/home/eleve/pidr/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    point_cloud = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    projection_matrix = kitti_calibration_matrix.calibration_matrix_cam2

    # Chargement de l'image
    img_file = "/home/eleve/pidr/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/image_02/data/0000000018.png"

    # Open the image using PIL
    image = Image.open(img_file)

    # Convert the image to a NumPy array
    image_array = np.array(image)

    coordinates = point_cloud_projection(
        projection_matrix=projection_matrix, point_cloud=point_cloud
    )

    colors = point_cloud_colors(image_coordinates=coordinates, image=image_array)


def test_colorize_point_cloud():
    # Chargement du nuage de points
    bin_file = "/home/eleve/pidr/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    projection_matrix = kitti_calibration_matrix.calibration_matrix_cam2

    # Chargement de l'image
    img_file = "/home/eleve/pidr/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/image_02/data/0000000018.png"

    # Open the image using PIL
    image = Image.open(img_file)

    # Convert the image to a NumPy array
    image_array = np.array(image)

    coordinates = point_cloud_projection(
        projection_matrix=projection_matrix, point_cloud=points
    )

    colors = point_cloud_colors_rgb(image_coordinates=coordinates, image=image_array)
    colored_points = np.concatenate([points, colors], axis=1)

    f = open("points.txt", "w")
    for p in points:
        for v in p:
            f.write(f"{v} ")
        f.write("\n")

    f = open("pcd.txt", "w")
    for c in colored_points:
        for v in c:
            f.write(f"{v} ")
        f.write("\n")

    # Create a PointCloud object
    point_cloud = o3d.geometry.PointCloud()

    # Set the points and colors
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors / 255)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])
