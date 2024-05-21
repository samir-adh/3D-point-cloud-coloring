import os
from tools import load_point_cloud,load_image, get_point_cloud_colors,colors_to_uint32
from kitti_calibration_matrix import calibration_matrix_cam2
import pyvista as pv

DATA_PATH = r"/home/samir/Downloads/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync/"
PCD_PATH = DATA_PATH + "velodyne_points/data/"
IMG2_PATH = DATA_PATH + "image_02/data/"
IMG3_PATH = DATA_PATH + "image_03/data/"

def display_colored_pcd():
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

    n_files = 1

    for pcd, img2, img3 in zip(pcd_files[:n_files], img2_files[:n_files], img3_files[:n_files]):
        # Load point cloud from the chosen file
        pcd_file = kitti_pcd_dir + pcd
        pcd_points = load_point_cloud(pcd_file)

        # Chargement des images de la cam 2
        img2_file = kitti_img2_dir + img2

        img_array = load_image(img2_file)

        # Calcul des couleurs
        pcd_colors = get_point_cloud_colors(pcd_points,img_array,calibration_matrix_cam2)
        
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(pcd_points[:, :3])
        # point_cloud.colors = o3d.utility.Vector3dVector(pcd_colors / 255)
        # o3d.visualization.draw_geometries([point_cloud])
        # # Wait for a few seconds before publishing the next point cloud
        # time.sleep(1 / 60)
        # # break
        point_cloud = pv.PolyData(pcd_points)
        point_cloud['RGB'] = pcd_colors

        # Create a plotter object
        plotter = pv.Plotter()

        # Add the point cloud to the plotter
        plotter.add_points(point_cloud, scalars='RGB', rgb=True)
        plotter.set_position([0,0,20])
        # Show the plot
        plotter.show()

if __name__ == '__main__':
    display_colored_pcd()