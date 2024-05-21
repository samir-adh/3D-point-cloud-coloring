import tools
import json
import open3d as o3d
import numpy as np
import os
import re


def main():
    file = open("./dataset_path.json")
    dataset_path = json.load(file)["dataset2"]

    # Chargement du nuage de points
    pcd_path = dataset_path + "/pointcloud.txt"
    pcd_points = tools.load_point_cloud(pcd_path, "xyz")

    # Chargement de l'image
    img_path = dataset_path + "/screenshot_000.png"
    img_array = tools.load_image(img_path)

    # Chargement de la matrice de calibration
    calib_path = dataset_path + "/screenshot_000.txt"
    calib_matrix = tools.load_calibration_matrix(calib_path)
    pcd_colors = tools.get_point_cloud_colors(
        point_cloud=pcd_points, img_array=img_array, calib_matrix=calib_matrix
    )
    # colors = np.hstack(
    #     [np.full((pcd_points.shape[0], 1), 255), np.zeros((pcd_points.shape[0], 2))]
    # )
    pcd_points_in_image = (
        calib_matrix @ np.hstack([pcd_points, np.ones((pcd_points.shape[0], 1))]).T
    ).T
    pcd_points_in_image[:, :2] = pcd_points_in_image[:, :2] / pcd_points_in_image[
        :, 3
    ].reshape(-1, 1)
    # pcd_points_in_image[:,:2] = pcd_points_in_image[:,:2]/pcd_points_in_image[:,2].reshape(-1,1)
    pcd_points_in_image[:, 1] *= -img_array.shape[0] / 2
    pcd_points_in_image[:, 0] *= img_array.shape[1] / 2
    pcd_points_in_image[:, 1] += img_array.shape[0] / 2
    pcd_points_in_image[:, 0] += img_array.shape[1] / 2

    pcd_points_in_image[:, 2] *= img_array.shape[0] / 2

    pcd_colors = tools.zbuffer_coloring(pcd_points_in_image, img_array)

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pcd_points_in_image[:, :3])
    point_cloud.colors = o3d.utility.Vector3dVector(pcd_colors / 255)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])  # type: ignore


def get_pcd_colors(pcd_points, img_array, calib_matrix) -> np.ndarray:
    pcd_colors = tools.get_point_cloud_colors(
        point_cloud=pcd_points, img_array=img_array, calib_matrix=calib_matrix
    )
    pcd_points_in_image = (
        calib_matrix @ np.hstack([pcd_points, np.ones((pcd_points.shape[0], 1))]).T
    ).T
    pcd_points_in_image[:, :2] = pcd_points_in_image[:, :2] / pcd_points_in_image[
        :, 3
    ].reshape(-1, 1)
    # pcd_points_in_image[:,:2] = pcd_points_in_image[:,:2]/pcd_points_in_image[:,2].reshape(-1,1)
    pcd_points_in_image[:, 1] *= -img_array.shape[0] / 2
    pcd_points_in_image[:, 0] *= img_array.shape[1] / 2
    pcd_points_in_image[:, 1] += img_array.shape[0] / 2
    pcd_points_in_image[:, 0] += img_array.shape[1] / 2

    pcd_points_in_image[:, 2] *= img_array.shape[0] / 2

    pcd_colors = tools.zbuffer_coloring(pcd_points_in_image, img_array)

    return pcd_colors


def load_all_screenshots(dataset_path):
    """
    Load all screenshots from the dataset.
    """
    dataset_screenshots = os.listdir(dataset_path)
    dataset_screenshots.sort()
    dataset_screenshots = [
        tools.load_image(f"{dataset_path}/{f}")
        for f in dataset_screenshots
        if re.match(r"screenshot_\d{3}.png", f)
    ]
    return dataset_screenshots


def load_all_matrices(dataset_path):
    """
    Load all screenshots from the dataset.
    """
    calibration_matrices = os.listdir(dataset_path)
    calibration_matrices.sort()
    calibration_matrices = [
        tools.load_calibration_matrix(f"{dataset_path}/{f}")
        for f in calibration_matrices
        if re.match(r"screenshot_\d{3}.txt", f)
    ]
    return calibration_matrices


def full_scan():
    file = open("./dataset_path.json")
    dataset_path = json.load(file)["dataset1"]

    # Chargement du nuage de points
    pcd_path = dataset_path + "/pointcloud.txt"
    pcd_points = tools.load_point_cloud(pcd_path, "xyz")

    # Chargement de toutes les images
    dataset_screenshots = load_all_screenshots(dataset_path)
    img_h, img_w, _ = dataset_screenshots[0].shape

    # Charage de toutes les matrices de calibration
    calibration_matrices = load_all_matrices(dataset_path)

    projections_per_image = [
        calib_matrix @ np.hstack([pcd_points, np.ones((pcd_points.shape[0], 1))]).T
        for calib_matrix in calibration_matrices
    ]
    projections_per_image = [projection.T for projection in projections_per_image]
    for projection in projections_per_image:
        projection[:, :2] = projection[:, :2] / projection[:, 3].reshape(-1, 1)
        projection[:, 1] *= -img_h / 2
        projection[:, 1] += img_h / 2
        projection[:, 0] *= img_w / 2
        projection[:, 0] += img_w / 2
        projection[:, 2] *= img_h / 2

    pcd_colors = np.zeros((pcd_points.shape[0], 3))

    print(f"n proj per image = {len(projections_per_image)}")

    def min_z_index(index):
        min_z_arg = 0
        for i in range(len(projections_per_image)):
            z = projections_per_image[i][index][2]
            if z < projections_per_image[min_z_arg][index][2]:
                min_z_arg = i
        return min_z_arg

    for index in range(len(pcd_points)):
        min_z_arg = min_z_index(index)
        i = np.floor(projections_per_image[min_z_arg][index][0]).astype(int)
        j = np.floor(projections_per_image[min_z_arg][index][1]).astype(int)
        pcd_colors[index] = dataset_screenshots[min_z_arg][j, i]

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pcd_points[:, :3])
    point_cloud.colors = o3d.utility.Vector3dVector(pcd_colors / 255)

    pcddown = point_cloud.voxel_down_sample(voxel_size=0.005)
    pcddown.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcddown],point_show_normal=True)  # type: ignore

    # Attention : on a des problèmes pour le dataset 2 par exemple parce que 
    # la cam qui fait face au visage est plus proche que la cam qui fait face au dos,
    # ce problème peut être réglé par le zbuffer => à faire


if __name__ == "__main__":
    full_scan()
    # main()
