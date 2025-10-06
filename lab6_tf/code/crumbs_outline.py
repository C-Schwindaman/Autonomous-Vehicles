#!/usr/bin/env python
''' crumbs.py

    Outline of solution to Lab 6, Exercise 2

'''
import rclpy
from rclpy.node import Node

class Crumbs(Node):
    def __init__(self):
        super().__init__('crumbs')        

        # Initialize variables

        # Initialize the static transform broadcaster

        # Initialize the tf2 buffer and listener

        # Create a list to hold the transforms

        # Create a timer for periodic checking of motion


    def check_motion(self):

        # define source and target frames

        # Find the transform from source to target frame

        # Check the distance

        # if distance is greater than threshold or first time through:
        # create a new crumb frame and append the transform to the list
        # publish the list of transforms

def main(args=None):
    rclpy.init(args=args)

    # Initialize the node and spin
