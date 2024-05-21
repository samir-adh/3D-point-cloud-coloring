# Kitti Point Cloud Publisher
### Module Ros2 pour publier les points du dataset Kitti dans le topic '/velodyne_points'

## Installation
### 1. Copier le module
Copier ou cloner le module dans `ros2_ws/src` (avec `ros2_ws` le nom du workspace ros2)
### 2. Changer le chemin vers le dataset
Dans [kitti_pointcloud_publisher.py](src/kitti_pointcloud_publisher/kitti_pointcloud_publisher/kitti_pointcloud_publisher.py), changer le chemin vers le dataset kitti par exemple :
`/chemin/du/dataset/2011_10_03/2011_10_03_drive_0047_sync/velodyne_points/data/`
### 3. Build le module
À la racine du répertoire (dans `ros2_ws/`) faire :
```bash
colcon build
```
Puis :
```bash
source install/setup.sh
```

## Visualiser le dataset
### Etape 1
Dans un terminal exécuter :
```bash
ros2 run kitti_pointcloud_publisher kitti_pointcloud_publisher
```
### Etape 2
Lancer rviz2 :
```bash
rviz2
```
### Etape 3
Ajouter PointCloud2 et changer le champ de `Fixed Frame` par `velodyne`


## TO-DO List

- [X] Visualiser le point cloud avec un autre outils que rviz2 pour s'assurer que les couleurs passées sont correctes
- [X] Colorier le nuage de point avec la deuxième caméra (cam3)
- [X] Colorier avec la difference de couleur entre les caméras
- [ ] Déterminer une "distance" maximale acceptable entre les deux coloriages pour chaque point 
- [ ] ZBuffer
- [ ] Rappport
- [X] HSL diff avec H
- [ ] Mesure de confiance pour diff hsl
