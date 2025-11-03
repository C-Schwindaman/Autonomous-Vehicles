#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class Obstacles(Node):
    def __init__(self, delta_r=0.25):
        super().__init__('bus_monitor')             
        self.delta_r = delta_r

        self.publisher = self.create_publisher(MarkerArray, 'obstacles', 1)    
        self.subscription = self.create_subscription(LaserScan, 'scan', self.lidar_centroids, 1)
        self.subscription

    def lidar_centroids(self, msg):       
        ranges = np.array(msg.ranges)  # Convert to Numpy array for vector operations
        if len(ranges)==0:
            return
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min # Angle of each ray
        x = ranges * np.cos(angles) # vector arithmatic is much faster than iterating
        y = ranges * np.sin(angles)
        good = ranges < np.inf  # get all finite returns

        # Create your list of obstacles here:

        # ...

        # let's say xcen and ycen are np arrays of obstacle centroids,
        # then convert them to a list of points like this:
        points = np.column_stack( (xcen, ycen, np.zeros_like(xcen)) ).tolist()

        # Create unique IDs for each point:
        ids = np.arange(len(points)).astype(int)

        # publish the centroids:
        self.pub_centroids(points, ids, msg.header)

      
    def pub_centroids(self, points, ids, header):

        ma = MarkerArray()

        for id, p in zip(ids, points):
            mark = Marker()            
            mark.header = header
            mark.id = id.item()
            mark.type = Marker.SPHERE
            mark.pose = Pose(position=Point(x=p[0],y=p[1],z=p[2]), orientation=Quaternion(x=0.,y=0.,z=0.,w=1.))
            mark.scale.x = 0.25
            mark.scale.y = 0.25
            mark.scale.z = 0.25
            mark.color.a = 0.75
            mark.color.r = 0.25
            mark.color.g = 1.
            mark.color.b = 0.25
            mark.lifetime = Duration(seconds=0.4).to_msg()
            ma.markers.append(mark)

        self.publisher.publish( ma )

def main(args=None):

    rclpy.init(args=args)

    node = Obstacles()
    rclpy.spin(node) 

    node.destroy_node()
    rclpy.shutdown()