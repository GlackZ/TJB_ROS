# from PIL import Image
# import pandas as pd
import numpy as np
from cv_bridge import CvBridge
import time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points 

class point_cloud_generator_depth():
 
    def __init__(self, ros_depth_img, focal_length, scalingfactor):

        self.width = ros_depth_img.width
        self.height = ros_depth_img.height

        self.cv_bridge = CvBridge()
        self.depth_img = self.cv_bridge.imgmsg_to_cv2(ros_depth_img, desired_encoding='passthrough')
        
        self.focal_length = focal_length
        self.scalingfactor = scalingfactor
        # self.rgb = Image.open(rgb_file)
        # self.depth = Image.open(depth_file).convert('I')
        
    def calculate(self):
        t1=time.time()
        depth = np.asarray(self.depth_img).T
        self.Z = depth / self.scalingfactor
        X = np.zeros((self.width, self.height))
        Y = np.zeros((self.width, self.height))
        for i in range(self.width):
            X[i, :] = np.full(X.shape[1], i)
 
        self.X = ((X - self.width / 2) * self.Z) / self.focal_length
        for i in range(self.height):
            Y[:, i] = np.full(Y.shape[0], i)
        self.Y = ((Y - self.height / 2) * self.Z) / self.focal_length
 
        df=np.zeros((6,self.width*self.height))
        df[0] = self.X.T.reshape(-1)
        df[1] = -self.Y.T.reshape(-1)
        df[2] = -self.Z.T.reshape(-1)

        # df[3] = np.full(df[3].shape, 255)
        # df[4] = np.full(df[4].shape, 255)
        # df[5] = np.full(df[5].shape, 255)
        self.df=df
        t2=time.time()
        print('calcualte 3d point cloud Done.',t2-t1)
        
        points =[]
        for i in self.df.T:
            points.append((float(i[0]), 
                           float(i[1]), 
                           float(i[2])))
 
        return points
    
class point_cloud_generator_pointcloudmsg():
    def __init__(self, point_cloud_msg):
        self.point_cloud_msg = point_cloud_msg
        self.points = []
    
    def calculate(self):
        for point in read_points(self.point_cloud_msg, field_names = ("x", "y", "z"), skip_nans=True):
            self.points.append((float(point[0]),
                                float(point[1]),
                                float(point[2])))
        return self.points