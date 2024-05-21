import open3d as o3d

def demo():
    # Chargement du nuage de points
    bin_file = "/home/eleve/pidr/pidr_scan_2024/titi"
    pcd = o3d.io.read_point_cloud(bin_file,format='xyz')
    o3d.visualization.draw_geometries([pcd])

demo()