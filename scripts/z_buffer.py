import kitti_calibration_matrix
import numpy as np
from PIL import Image
import open3d as o3d
import matplotlib.pyplot as plt


# TODO Problème lié à un facteur d'échelle camera/lidar ???
# + Voir ce que ça donne en faisant el coloriage avec 2 cameras
def point_cloud_projection(
    projection_matrix: np.ndarray, point_cloud: np.ndarray
) -> np.ndarray:
    point_cloud_padded = np.concatenate(
        [point_cloud, np.ones(shape=(point_cloud.shape[0], 1))], axis=1
    )
    img_coordinates = np.matmul(projection_matrix, point_cloud_padded.T)
    # img_coordinates[:2, :] /= img_coordinates[[-1], :]

    return img_coordinates


def point_cloud_colors_rgb(
    image_coordinates: np.ndarray, image: np.ndarray
) -> np.ndarray:
    colors = np.zeros(shape=(image_coordinates.shape[1], 3))
    for i, coordinate in enumerate(image_coordinates.astype(int).T):
        if coordinate[2] > 0:
            coordinate[0] = int(coordinate[0] / coordinate[2])
            coordinate[1] = int(coordinate[1] / coordinate[2])
            if (
                coordinate[0] < image.shape[1]
                and coordinate[1] < image.shape[0]
                and coordinate[0] > 0
                and coordinate[1] > 0
            ):
                # print(f"coordinate : {coordinate}")
                c = image[coordinate[1], coordinate[0]]
                # print(f"c = {c}")
                colors[i] = c  # / 255
    return colors


def z_buffer(image: np.ndarray, image_coordinates: np.ndarray) -> np.ndarray:
    dims = image.shape[0], image.shape[1]
    buffer = np.zeros(shape=dims, dtype=int)
    for i, coordinate in enumerate(image_coordinates.astype(int).T):
        if coordinate[2] > 0:
            buffer_x = int(coordinate[0] / coordinate[2])
            buffer_y = int(-coordinate[1] / coordinate[2])
            if (
                buffer_y < image.shape[0]
                and buffer_x < image.shape[1]
                and buffer_y > 0
                and buffer_x > 0
            ):
                buffer[buffer_y, buffer_x] = i
    return buffer


def get_zbuffer_colors(
    point_cloud: np.ndarray, z_buffer: np.ndarray, image: np.ndarray
) -> np.ndarray:
    colors = np.zeros(shape=(point_cloud.shape[0], 3))
    for y in range(len(z_buffer)):
        for x in range(len(z_buffer[y])):
            i = z_buffer[y, x]
            colors[i] = image[y, x]
    # for c in colors:
    #     if sum(c) > 0:
    #         print(f"colors : {c}")
    return colors


def colorize_point_cloud(
    point_cloud: np.ndarray, image: np.ndarray, calibration_matrix: np.ndarray
):
    coordinates = point_cloud_projection(
        projection_matrix=calibration_matrix, point_cloud=point_cloud
    )
    colors = point_cloud_colors_rgb(
        coordinates, image
    )  # .reshape((point_cloud.shape[0], 1))
    colored_pcd = np.concatenate([point_cloud, colors], axis=1)
    return colored_pcd


def rgb_to_uint32(rgb: np.ndarray):
    # Make sure RGB values are within range
    r, g, b = rgb
    r = max(0, min(255, r))
    g = max(0, min(255, g))
    b = max(0, min(255, b))

    # Pack RGB values into a single uint32
    return (r << 16) | (g << 8) | b
