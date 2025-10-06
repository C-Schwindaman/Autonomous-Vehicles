'''
GroundSpot Node
  Subscribes to camera images and camera_info
  Uses a trained logistic regression model to find a landmark in the image
  Uses camera intrinsics and extrinsics to convert the landmark pixel location
    to a ground point in base_footprint coordinates
  Publishes the ground point as a PointStamped message
'''
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node


class GroundSpot(Node):
    def __init__(self, path_params):
        super().__init__('ground_spot')

        # Create listener for transforms:

        # Create publisher for ground spot:

        # Load logistic regression model parameters

        # Initialize variables for intrinsics and extrinsics:

        # Use a mutually exclusive callback group to prevent simultaneous execution of callbacks

        # Subscription to camera_info to get intrinsics:

        # Subscription to the image topic:
                 

    def copy_cam_info(self, msg):
        ''' Callback to read in camera intrinsics from camera_info 
        '''
        # If intrinsics not yet initialized, 
        # then copy D and K matrices and convert numpy arrays, where K should be 3x3

    def initialize_extrinsics(self, stamp):
        ''' Initialize camera extrinsics from tf2 
            stamp: the current timestamp for finding the transformation
        '''
        # If extrinsics not yet initialized,
        # then lookup the transform from base_footprint to the camera optical frame
        # Extract the rotation and translation

    def find_landmark_in_image(self, min_area=100):
        ''' Find landmark in image and return pixel coordinates
            min_area: minimum number of pixels to consider a valid landmark
        '''
        # Apply logistic regression and find largest target to find the landmark:

    def calc_ground_spot(self, msg):
        ''' Callback to find the ground_spot from the image
            msg: the incoming Image message 
            Will publish a PointStamped message if landmark found
        '''
        # If intrinsics not yet initialized, return

        # Initialize camera extrinsics:

        # Extract image from the message:

        # Find landmark in image and plot it:

        # If no landmark found, return

        # Initialize the header for the output PointStamped message:

        # Use math from the provided problem to find the ground point:

        # Create a PointStamped containing this point along with the header: out_header
        
        # Publish the ground spot


    # ----------------
    # The below are helper functions for logistic regression and plotting
    def apply(self, img):
        ''' Application of trained logisitic regression to an image
            img:         [MxNx3] input 3-channel color image
            score:       [MxN] logistic regression score for each pixel (between -inf and inf)
        '''
        score = (img.astype(float) * self.cvec).sum(axis=2) + self.intercept
        return score

    def prob_target(self, score):
        ''' Transforms score to probability of target using sigmoid '''
        return expit( score )

    def find_largest_target(self, prob_target, threshold=0.5, minpix=20):
        ''' Finds largest contiguous target region
            prob_target: [MxN] input probability of target
            centroid:    [x,y] coordinates of the largest centroid
            area:        number of pixels in target
            target_mask: [MxN] binary mask with 1's at target
        '''
        binary_target = (prob_target>threshold).astype(np.uint8)
        cc = cv.connectedComponentsWithStats(binary_target)
        inds = np.argsort( cc[2][:,4] )  # Sort on number of pixels in each continguous region
        target_mask = np.zeros_like(binary_target)
        centroid = []
        area = []
        for i in inds[::-1]:            
            if binary_target[cc[1]==i].astype(float).mean() > 0.99 and cc[2][i,4] >= minpix:
                target_mask += (cc[1]==i).astype(np.uint8)
                centroid = cc[3][i,:] 
                area = cc[2][i,4]
                break
        return centroid, area, target_mask
    
    def plotTargets(self, target_mask, centroid):

        ''' Plot detected target_mask and output to file
            target_mask: (NxM) numpy array, or else empty list
            centroids: list of [x,y] centroids
        ''' 
        if isinstance(target_mask,list):
            target_mask = np.array(target_mask)  # Needs to be a numpy array
        if target_mask.size:  # Check if not empty numpy array:
            # Then highlight the detected pixels in the original image
            green = np.zeros_like(self.img)
            green[:,:,1] = 128
            mask = target_mask[:,:,None].repeat(3,axis=2)
            outim = self.img.copy() * (1-mask) + (self.img.copy()//2 + green) * mask
        else:
            outim = self.img.copy()
        if not centroid is None:
            loc = tuple( np.array(centroid).astype(int) )  # location needs to be int tuple
            cv.circle(outim, loc, 12, (0,0,255), -1 )
        cv.imshow("Target", cv.resize(outim,(800,400)))

        return outim


def main(args=None):
    rclpy.init(args=args)

    # Set up argument parser.
   
    # Initialize the GroundSpot node and spin


