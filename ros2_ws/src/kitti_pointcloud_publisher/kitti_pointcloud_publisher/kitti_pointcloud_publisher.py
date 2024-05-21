"""Publisher ros2 qui publie des nuages de points coloriés"""
import os
import time
import sys

# from progress.bar import ChargingBar
import numpy as np
import rclpy
from kitti_pointcloud_publisher.kitti_calibration_matrix import (
    calibration_matrix_cam2, calibration_matrix_cam3)
from kitti_pointcloud_publisher.tools import *
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# Chemin des données LIDAR du dataset
DATA_PATH = r"/home/samir/Downloads/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync/"
PCD_PATH = DATA_PATH + "velodyne_points/data/"
IMG2_PATH = DATA_PATH + "image_02/data/"
IMG3_PATH = DATA_PATH + "image_03/data/"

calibration_matrix = calibration_matrix_cam3


class PointCloudPublisher(Node):
    """Publisher de nuages de points"""

    def __init__(self):
        super().__init__("point_cloud_publisher")
        self.publisher_ = self.create_publisher(
            PointCloud2, "/velodyne_points", 10)

    def point_cloud_colors(self, point_cloud: np.ndarray, img_array, calib_matrix) -> np.ndarray:
        """Fonction pour calculer les couleurs associées aux points"""
        point_cloud_camera = velo_to_cam_projection(point_cloud, calib_matrix)
        point_cloud_camera_h = cam_to_image_coordinates(point_cloud_camera)
        point_cloud_colors = colors_from_pcd_h(
            point_cloud_camera_h, img_array)
        colors_as_uint32 = colors_to_uint32(point_cloud_colors)
        return colors_as_uint32

    def get_colors_rgb(self, point_cloud, img_array, calib_matrix):
        point_cloud_camera = velo_to_cam_projection(point_cloud, calib_matrix)
        point_cloud_camera_h = cam_to_image_coordinates(point_cloud_camera)
        point_cloud_colors = colors_from_pcd_h(
            point_cloud_camera_h, img_array)
        return point_cloud_colors

    def publish_colored_point_cloud(self, point_cloud, colors):
        """
        Publish point cloud as ROS PointCloud2 message.

        Parameters:
            point_cloud (numpy.ndarray): Nx3 array containing XYZ coordinates of the point cloud.
        """
        header = Header()
        header.frame_id = "velodyne"

        fields = [
            PointField(name="x", offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12,
                       datatype=PointField.UINT32, count=1),
        ]
        nb_pts = len(point_cloud)

        msg = PointCloud2()
        msg.header = header
        msg.fields = fields
        msg.height = 1
        msg.width = nb_pts
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * nb_pts
        msg.is_dense = True
        # data_bytes = point_cloud.astype(np.float32).tobytes()
        data_bytes = bytearray()
        for p, c in zip(point_cloud, colors):
            data_bytes.extend(p.astype(np.float32).tobytes())
            data_bytes.extend(c.astype(np.uint32).tobytes())

        if len(data_bytes) != msg.width * msg.point_step:
            print("Error: Data size does not match expected size. Dropping message.")
            return
        msg.data = data_bytes
        self.publisher_.publish(msg)

    def publish_pcd_colors_difference(self, point_cloud: np.ndarray, difference: np.ndarray):
        """
        Publish point cloud as ROS PointCloud2 message.

        Parameters:
            point_cloud (numpy.ndarray): Nx3 array containing XYZ coordinates of the point cloud.
        """
        header = Header()
        header.frame_id = "velodyne"

        fields = [
            PointField(name="x", offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        nb_pts = len(point_cloud)

        msg = PointCloud2()
        msg.header = header
        msg.fields = fields
        msg.height = 1
        msg.width = nb_pts
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * nb_pts
        msg.is_dense = True
        data_bytes = np.hstack((point_cloud, difference.reshape(
            difference.shape[0], 1))).astype(np.float32).tobytes()
        if len(data_bytes) != msg.width * msg.point_step:
            print("Error: Data size does not match expected size. Dropping message.")
            return
        msg.data = data_bytes
        self.publisher_.publish(msg)


def main(args=None):
    """Fonction principale"""
    publish_colored_point_cloud()


def publish_colored_point_cloud(args=None):
    rclpy.init(args=args)

    point_cloud_publisher = PointCloudPublisher()

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
    img2_files.sort()
    img3_files.sort()

    n_files = -1

    while True:
        for pcd, img2, img3 in zip(pcd_files[:n_files], img2_files[:n_files], img3_files[:n_files]):
            # Load point cloud from the chosen file
            pcd_file = kitti_pcd_dir + pcd
            point_cloud = load_point_cloud(pcd_file)

            # Chargement des images de la cam 2
            img2_file = kitti_img2_dir + img2

            img_array = load_image(img2_file)

            # Calcul des couleurs
            colors = get_point_cloud_colors(point_cloud,img_array,calibration_matrix_cam2)
            colors_as_uint32 = colors_to_uint32(colors)

            # Publish the point cloud
            # point_cloud_publisher.publish_pcd_colors_difference(point_cloud, diff)
            point_cloud_publisher.publish_colored_point_cloud(
                point_cloud, colors_as_uint32)
            # Wait for a few seconds before publishing the next point cloud
            time.sleep(1 / 60)
            # break

    rclpy.spin(point_cloud_publisher)

    point_cloud_publisher.destroy_node()
    rclpy.shutdown()



def publish_diff_rgb(args=None):
    rclpy.init(args=args)

    point_cloud_publisher = PointCloudPublisher()

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
    img2_files.sort()
    img3_files.sort()

    outputs = []
    # bar = ChargingBar('Chargement', max=len(pcd_files))
    n_files = -1

    while True:
        for pcd, img2, img3 in zip(pcd_files[:n_files], img2_files[:n_files], img3_files[:n_files]):
            # Load point cloud from the chosen file
            pcd_file = kitti_pcd_dir + pcd
            point_cloud = load_point_cloud(pcd_file)

            # Chargement des images de la cam 2
            img2_file = kitti_img2_dir + img2
            image2_array = np.array(Image.open(img2_file))

            # Chargement des images de la cam 3
            img3_file = kitti_img3_dir + img3
            image3_array = np.array(Image.open(img3_file))

            # Récupération des couleurs de la cam 2
            colors_cam2 = point_cloud_publisher.get_colors_rgb(
                point_cloud, image2_array, calibration_matrix_cam2)

            # Récupération des couleurs de la cam 3
            colors_cam3 = point_cloud_publisher.get_colors_rgb(
                point_cloud, image3_array, calibration_matrix_cam3)

            # Calcul de la différence
            diff = colors_diff(colors_cam1=colors_cam2,
                               colors_cam2=colors_cam3, norm=norm2)
            # Tableau des couleurs rouge correspondantes
            col_diff = np.hstack(((diff*255).reshape(
                diff.shape[0], 1), np.zeros(shape=(diff.shape[0], 2))))
            col_diff_uint = colors_to_uint32(col_diff)

            # Publish the point cloud
            # point_cloud_publisher.publish_pcd_colors_difference(point_cloud, diff)
            point_cloud_publisher.publish_colored_point_cloud(
                point_cloud, col_diff_uint)
            # Wait for a few seconds before publishing the next point cloud
            time.sleep(1 / 60)
            # break

    rclpy.spin(point_cloud_publisher)

    point_cloud_publisher.destroy_node()
    rclpy.shutdown()


def publish_diff_hsv(args=None):
    rclpy.init(args=args)

    point_cloud_publisher = PointCloudPublisher()

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
    img2_files.sort()
    img3_files.sort()

    n_files = -1

    while True:
        for pcd, img2, img3 in zip(pcd_files[:n_files], img2_files[:n_files], img3_files[:n_files]):
            # Load point cloud from the chosen file
            pcd_file = kitti_pcd_dir + pcd
            point_cloud = load_point_cloud(pcd_file)

            # Chargement des images de la cam 2
            img2_file = kitti_img2_dir + img2

            # Chargement des images de la cam 3
            img3_file = kitti_img3_dir + img3

            # Calcul de la différence
            diff = pcd_with_hue_diff(
                pcd_file, img2_file, img3_file, calibration_matrix_cam2, calibration_matrix_cam3)

            point_cloud = diff[:, :3]
            diff = diff[:, 3]

            col_diff = np.hstack(((diff/360 * 255).reshape(
                diff.shape[0], 1), np.zeros(shape=(diff.shape[0], 2))))
            col_diff_uint = colors_to_uint32(col_diff)
            print(f"max diff = {np.max(col_diff[:,0])}")

            # Publish the point cloud
            # point_cloud_publisher.publish_pcd_colors_difference(point_cloud, diff)
            point_cloud_publisher.publish_colored_point_cloud(
                point_cloud, col_diff_uint)
            # Wait for a few seconds before publishing the next point cloud
            time.sleep(1 / 60)
            # break

    rclpy.spin(point_cloud_publisher)

    point_cloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
