"""Fonction pour manipuler et colorier des nuages de points au format numpy"""

import numpy as np
from PIL import Image


def velo_to_cam_projection(
    point_cloud: np.ndarray, velo_to_cam_matrix: np.ndarray
) -> np.ndarray:
    """Projection des coordonées des points dans le référentiel de la caméra"""
    point_cloud_h = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))
    point_cloud_camera = velo_to_cam_matrix @ point_cloud_h.T
    return point_cloud_camera


def cam_to_image_coordinates(point_cloud_camera: np.ndarray) -> np.ndarray:
    """Projection des points dans les coordonées de l'image"""
    point_cloud_camera_h = point_cloud_camera
    point_cloud_camera_h[:2] /= point_cloud_camera_h[2]
    point_cloud_camera_h = point_cloud_camera_h.T
    return point_cloud_camera_h


def colors_from_pcd_h(
    point_cloud_camera_h: np.ndarray, image_array: np.ndarray
) -> np.ndarray:
    """Récupération de la couleur des points dans l'image"""
    img_h, img_w, _ = image_array.shape
    point_cloud_colors = np.zeros(shape=(point_cloud_camera_h.shape[0], 3), dtype=int)

    x = np.round(point_cloud_camera_h[:, 0]).astype(int)
    y = np.round(point_cloud_camera_h[:, 1]).astype(int)
    z = point_cloud_camera_h[:, 2]

    valid_indices = (x >= 0) & (x < img_w) & (y >= 0) & (y < img_h) & (z > 0)

    x_valid = x[valid_indices]
    y_valid = y[valid_indices]

    point_cloud_colors[valid_indices] = image_array[y_valid, x_valid]

    return point_cloud_colors


def colors_diff(colors_cam1, colors_cam2, norm) -> np.ndarray:
    """Calcul de la différence de couleur avec la norme passée en argument"""
    # On récupère les uniquement indices des points coloriés deux fois
    colored_indexes = np.logical_and(
        (np.sum(colors_cam1, axis=1) > 0), (np.sum(colors_cam2, axis=1) > 0)
    )
    diff_by_color = np.zeros((colors_cam1.shape))
    diff_by_color[colored_indexes] = (colors_cam1 - colors_cam2)[
        colored_indexes
    ] / 255.0
    diff = norm(diff_by_color)
    diff_normalized = diff / np.sqrt(3)  # np.max(diff)
    # for d in diff_normalized:
    #     print(d)
    return diff_normalized  # np.sqrt(diff_normalized)


def colors_to_hsv(colors: np.ndarray):
    """Convertit un tableau de couleurs RGB en tableau couleurs hsv"""
    colors_hsv = np.zeros(shape=(colors.shape[0], 3))
    for i, c in enumerate(colors):
        colors_hsv[i] = rgb_to_hsv(c)
    return colors_hsv


def hue_diff(hsv_cam1: np.ndarray, hsv_cam2: np.ndarray) -> np.ndarray:
    """Calcul de la différence de teinte"""
    colored_indexes = np.logical_and(
        (np.sum(hsv_cam1, axis=1) > 0), (np.sum(hsv_cam2, axis=1) > 0)
    )
    diff = np.absolute(hsv_cam1[:, 0] - hsv_cam2[:, 0]).reshape(-1, 1)
    # A tester
    # diff = np.absolute(hsv_cam1[colored_indexes, 0] - hsv_cam2[colored_indexes, 0]).reshape(-1, 1)
    return diff


def norm1(vec_array: np.ndarray) -> np.ndarray:
    """Norme 1 : Somme_sur_i(abs(xi))"""
    return np.sum(np.absolute(vec_array), axis=1)


def norm2(vec_array: np.ndarray) -> np.ndarray:
    """Norme 2 : racine_carre(Somme_sur_i(carré(xi)))"""
    return np.sqrt(np.sum(np.square(vec_array), axis=1))


def colors_to_uint32(colors: np.ndarray) -> np.ndarray:
    """Convertit un tableau de couleurs RGB en tableau couleurs uint32"""
    colors_uint32 = np.zeros(shape=(colors.shape[0], 1), dtype=np.uint32)
    for i, c in enumerate(colors):
        colors_uint32[i] = rgb_to_uint32(c)
    return colors_uint32


def rgb_to_hsv(rgb: np.ndarray):
    """Conversion RVB -> HSV"""
    r, g, b = rgb / 255
    # assert 0 <= r <= 1 and 0 <= g <= 1 and 0 <= b <= 1
    c_max = max(rgb)
    c_min = min(rgb)
    v = c_max
    delta = c_max - c_min
    if delta == 0:
        return (0, 0, v)
    h = hue(delta, r, g, b, c_max)
    s = delta / v
    return (h, s, v)


def hue(delta, r, g, b, c_max):
    """Calcul de la teinte"""
    result = 0
    if r == c_max:
        result = 60 * ((g - b) / delta)
    elif g == c_max:
        result = 60 * ((b - r) / delta + 2)
    else:  # b == Cmax
        result = 60 * ((r - g) / delta + 4)
    return result


def rgb_to_uint32(rgb: np.ndarray):
    """Convertit des RGB en uint32"""
    r, g, b = rgb
    r = max(0, min(255, r))
    g = max(0, min(255, g))
    b = max(0, min(255, b))
    return np.uint32((np.uint8(b) << 16) | (np.uint8(g) << 8) | np.uint8(r))


def get_point_cloud_colors(
    point_cloud: np.ndarray, img_array: np.ndarray, calib_matrix: np.ndarray
) -> np.ndarray:
    """Fonction pour calculer les couleurs associées aux points"""
    point_cloud_camera = velo_to_cam_projection(point_cloud, calib_matrix)
    point_cloud_camera_h = cam_to_image_coordinates(point_cloud_camera)
    point_cloud_colors = colors_from_pcd_h(point_cloud_camera_h, img_array)
    return point_cloud_colors


def load_image(png_file):
    """
    Charge l'image depuis un fichier .png et renvoie un tableau numpy.
    """
    return np.array(Image.open(png_file))


def load_calibration_matrix(calib_file) -> np.ndarray:
    """
    Charge la matrice de calibration depuis le fichier screenshot_XXX.txt
    """
    file = open(calib_file)
    lines = file.readlines()
    file.close()

    def parse_matrix(start_line) -> np.ndarray:
        matrix = np.zeros((4, 4))
        for i in range(start_line, start_line + 4):
            line = lines[i]
            matrix[i - start_line] = [float(x) for x in line.split(" ")[1:-1]]
        return matrix

    model = parse_matrix(1)
    view = parse_matrix(6)
    persp = parse_matrix(11)
    calib_matrix = persp.T @ view.T @ model.T
    # print("model :")
    # print(model)
    # print("view :")
    # print(view)
    # print("persp :")
    # print(persp)
    # print("calib :")
    # print(calib_matrix)
    return calib_matrix


def pcd_with_hue_diff(
    bin_file, img1_file, img2_file, calib_matrix1, calib_matrix2
) -> np.ndarray:
    """Calcul de la différence de teinte à partir des fichiers source"""
    pcd = load_point_cloud(bin_file)
    calib = (calib_matrix1, calib_matrix2)
    imgs = load_image(img1_file), load_image(img2_file)
    pcd_colors = [get_point_cloud_colors(pcd, imgs[i], calib[i]) for i in range(2)]
    pcd_hsv = [colors_to_hsv(pcd_colors[i]) for i in range(2)]
    h_diff = hue_diff(pcd_hsv[0], pcd_hsv[1])
    return np.hstack([pcd, h_diff])


def load_point_cloud(bin_file, file_type=None) -> np.ndarray:
    """
    Load point cloud from KITTI .bin file.

    Parameters:
        bin_file (str): Path to the .bin file.

    Returns:
        numpy.ndarray: Nx3 array containing XYZ coordinates of the point cloud.
    """
    if file_type == "xyz":
        file = open(bin_file)
        lines = file.readlines()
        file.close()
        pcd = np.zeros((len(lines), 3))
        for index, line in enumerate(lines):
            coordinates = [float(c) for c in line.split(" ")]
            pcd[index] = coordinates
        return pcd
    else:
        point_cloud = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)[:, :3]
        return point_cloud


def create_zbuffer(
    point_cloud_camera_h: np.ndarray, image_array: np.ndarray
) -> np.ndarray:
    """Fonction pour générer un z-buffer"""
    img_h, img_w, _ = image_array.shape
    z_buffer = np.full(shape=(img_h, img_w), fill_value=-1)

    x = np.round(point_cloud_camera_h[:, 0]).astype(int)
    y = np.round(point_cloud_camera_h[:, 1]).astype(int)
    z = point_cloud_camera_h[:, 2]


    for index in range(len(x)):
        current_point_index = z_buffer[y[index], x[index]]
        if current_point_index == -1 or z[current_point_index] > z[index]:
            z_buffer[y[index], x[index]] = index
        elif z[current_point_index] < z[index]:
            print(f"point {index} is behind")

    return z_buffer


def extract_colors_from_zbuffer(
    num_points: int, img_array: np.ndarray, z_buffer: np.ndarray
) -> np.ndarray:
    colors = np.zeros((num_points, 3))
    for i in range(img_array.shape[0]):
        for j in range(img_array.shape[1]):
            colors[z_buffer[i, j]] = img_array[i, j]
    return colors


def zbuffer_coloring(
    point_cloud_camera_h: np.ndarray, image_array: np.ndarray
) -> np.ndarray:
    z_buffer = create_zbuffer(
        image_array=image_array, point_cloud_camera_h=point_cloud_camera_h
    )
    colors = extract_colors_from_zbuffer(
        num_points=point_cloud_camera_h.shape[0],
        img_array=image_array,
        z_buffer=z_buffer,
    )
    return colors
