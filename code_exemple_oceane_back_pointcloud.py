import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
import cv2
import time
import numpy as np

from bosdyn.api import image_pb2
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.frame_helpers import *
from bosdyn.client.image import ImageClient, depth_image_to_pointcloud, build_image_request
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.util import timestamp_to_nsec
import bosdyn.client.math_helpers


class BackPointCloud(Node):


    def __init__(self):
    
        sdk = bosdyn.client.create_standard_sdk('RobotCommandMaster') 
        self.robot = sdk.create_robot("192.168.80.3") 
        self.robot.authenticate("admin", "u4zdowc8kr8p")  # ri2bdhh6n776 #u4zdowc8kr8p # "uxxi4si2xtg7"
        self.robot.time_sync.wait_for_sync() 
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.sources_left = ['back_depth', 'back_fisheye_image'] #_in_visual_frame
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'back', 10)
        timer_period = 0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        #total = time.time()
        pixel_format = None
        msg = PointCloud2()  
        dtype = np.float32
        ros_dtype = sensor_msgs.PointField.FLOAT32
        itemsize = np.dtype(dtype).itemsize

        #récupération du nuage de points à partir de la caméra
        #temps_im = time.time()
        image_request = [
            build_image_request(source, pixel_format=pixel_format)
            for source in self.sources_left
        ]
        image_responses = self.image_client.get_image(image_request)
        image = image_responses[0]

        pointcloud = depth_image_to_pointcloud(image)
        #print("temps image : ", time.time() - temps_im)
        
        #récupération image
        #temps_color = time.time()
        im = cv2.imdecode(np.frombuffer(image_responses[1].shot.image.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        im_rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        
        cv_depth = np.frombuffer(image_responses[0].shot.image.data, dtype=np.uint16)
        cv_depth = cv_depth.reshape(image_responses[0].shot.image.rows, image_responses[0].shot.image.cols)

        imnp  = np.array(im_rgb)
        iminv = np.array(imnp[:,:,[0,0,0,0]]).copy() 
        iminv[:,:,3] = 0
        imfl  = np.frombuffer(iminv, dtype=np.float32).reshape(im_rgb.shape[0], im_rgb.shape[1])        
        
        color_visual = []
        for x in range(len(cv_depth)):
            for y in range(len(cv_depth[0])):
                if cv_depth[x][y] != 0 :                    
                    color_visual.append(imfl[x][y])
                    
        #print("temps récupération couleur : ", time.time() - temps_color)
        
        #création du fields
        fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]
        fields += [sensor_msgs.PointField( name='rgb', offset=12, datatype=sensor_msgs.PointField.UINT32, count=1 )]

        #Création de data        
        #temps_data = time.time()
        rot = image.shot.transforms_snapshot.child_to_parent_edge_map["back"].parent_tform_child
        se3  = math_helpers.SE3Pose(rot.position.x, rot.position.y, rot.position.z, rot.rotation)
        
        pointcloud = se3.transform_cloud(pointcloud)
        points = np.concatenate((pointcloud, color_visual), axis=1)

        #print("temps création data : ", time.time() - temps_data)
        
        #Création du message
        msg.header = std_msgs.Header(frame_id="map")
        msg.height = 1
        msg.width = len(pointcloud)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = itemsize*4
        msg.row_step = msg.width * msg.point_step
        data = points.astype(dtype).tobytes()
        msg.data = data
        msg.is_dense = False

        #publication du message
        self.publisher_.publish(msg)
        
        #print("temps total : " + str(time.time() - total))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = BackPointCloud()
    rclpy.spin(minimal_publisher)
    
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
