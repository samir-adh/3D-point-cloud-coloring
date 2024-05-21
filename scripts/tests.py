from z_buffer import *
import kitti_calibration_matrix
import numpy as np
from PIL import Image
import open3d as o3d
import matplotlib.pyplot as plt

def test_pcd_projection():
    # Chargement du nuage de points
    bin_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    point_cloud = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    projection_matrix = kitti_calibration_matrix.calibration_matrix
    coor = point_cloud_projection(
        projection_matrix=projection_matrix, point_cloud=point_cloud
    )
    print(coor.astype)


def test_pcd_colors():
    # Chargement du nuage de points
    bin_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    point_cloud = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    projection_matrix = kitti_calibration_matrix.calibration_matrix

    # Chargement de l'image
    img_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/image_02/data/0000000018.png"

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
    bin_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    projection_matrix = kitti_calibration_matrix.calibration_matrix

    # Chargement de l'image
    img_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/image_02/data/0000000018.png"

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


def demo_plt():
    bin_file = "/home/samir/Desktop/temp/pidr/toto.txt"
    from mpl_toolkits import mplot3d
    import numpy as np
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = plt.axes(projection="3d")
    points_3d = np.loadtxt(bin_file).reshape(-1, 3)[:, :3]
    ax.set_xlim([-1200, 1200])
    ax.set_xlim([-200, 200])
    ax.scatter3D(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], s=1)
    plt.show()


def demo():

    # Chargement du nuage de points
    bin_file = "/home/samir/Desktop/temp/pidr/toto.txt"
    pcd = o3d.io.read_point_cloud(bin_file, format="xyz")
    o3d.visualization.draw_geometries([pcd])


def toto():
    bin_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    f = open("toto.txt", "w")
    calib = kitti_calibration_matrix.calibration_matrix
    img_coordinates = point_cloud_projection(
        projection_matrix=calib, point_cloud=points
    )
    for c in img_coordinates.T:
        # print(f"P {c}")
        if c[2] > 0:  # and np.abs(c[0]) < 1200 and np.abs(c[1] < 200):
            # f.write(f"{-(c[0]/c[2]-600)} {(185 - c[1]/c[2])} {c[2]}\n")
            f.write(f"{c[0]} {c[1]} {c[2]}\n")


def test_z_buffer():
    # Chargement du nuage de points
    bin_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data/0000000018.bin"
    points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
    projection_matrix = kitti_calibration_matrix.calibration_matrix

    # Chargement de l'image
    img_file = "/home/samir/Desktop/PIDR/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync/image_02/data/0000000018.png"

    # Open the image using PIL
    image = Image.open(img_file)

    # Convert the image to a NumPy array
    image_array = np.array(image)

    coordinates = point_cloud_projection(
        projection_matrix=projection_matrix, point_cloud=points
    )
    z_buff = z_buffer(image=image_array, image_coordinates=coordinates)
    colors = get_zbuffer_colors(point_cloud=points, z_buffer=z_buff,image=image_array)
    colors /= 255
    colored_points = np.concatenate([points, colors], axis=1)
    # point_cloud = o3d.geometry.PointCloud()

    # # Set the points and colors
    # point_cloud.points = o3d.utility.Vector3dVector(points)
    # point_cloud.colors = o3d.utility.Vector3dVector(colors / 255 )

    # o3d.visualization.draw_geometries([point_cloud])
    f = open("pcd.txt", "w")
    for c in colored_points:
        for v in c:
            f.write(f"{v} ")
        f.write("\n")

def o3d_txt():
    # Chargement du nuage de points
    bin_file = "pcd.txt"
    pcd = o3d.io.read_point_cloud(bin_file, format="xyzrgb")
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    test_z_buffer()
    o3d_txt()
