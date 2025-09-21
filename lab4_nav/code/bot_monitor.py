#!/usr/bin/env python
''' bot_monitor.py

    To save plot to bot_plot.png do:
     ros2 param set /bot_monitor bot_plot save
    To clear plot do:
     ros2 param set /bot_monitor bot_plot clear

    <...> Complete missing portions

    Copyright: Daniel Morris, 2020, 2025
'''
import time
import matplotlib.pyplot as plt
import rclpy, os
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

NAMESPACE = os.environ.get("ROBOT_NAMESPACE","")

class PlotOdom(Node):
    ''' Reads and plots odometry '''

    def __init__(self):
        super().__init__('bot_monitor')        

        self.deltaPos = 0.02  #Only add a point if moved at least this far from prevously plotted point
        # <...> initialize some parameters to store previously plotted points


        self.isinit = False
        self.declare_parameter('bot_plot','')

        # Prepare quality of service for subscription:
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # <...> Create a subscription to /odom with the appropriate namespace

        # <...> Create log that says what topic has been subscribed:



    def callback(self, msg):
        ''' A callback on an Odometry message '''

        # <...> extract the robot position

        # <...> set doPlot to be true if moved at least self.deltaPos from previously plotted position

        if not self.isinit: 
            # Do plot initialization in callback to keep all plotting 
            # in the same thread            
            doPlot = True  # Always plot if doing initialization
            self.isinit = True 
            figsel = 'Odom'
            fig = plt.figure(figsel,figsize=(4,4))
            fig.clf()
            plt.subplots(1,1,num=figsel)    
            plt.grid(True, linestyle='--')
            plt.gca().set_aspect('equal', 'box')
            plt.gcf().canvas.flush_events()
            plt.show(block=False)
            plt.show(block=False) # For some reason calling this twice is necessary
            
        if doPlot:
            # <...> store new position as previously plotting position

            # <...> plot current position

            plt.gcf().canvas.flush_events()
            plt.show(block=False)
            plt.show(block=False)
            time.sleep(0.01)
        
        # Handle saving plot to png and clearing plot
        val = self.get_parameter('bot_plot').get_parameter_value().string_value
        if val:   # if not empty parameter:
            if val=="save":
                plt.savefig("bot_plot.png")
                self.get_logger().info("Saving plot to bot_plot.png")
            elif val=="clear":
                plt.cla()
                plt.grid(True, linestyle='--')
                plt.gcf().canvas.flush_events()
                plt.show(block=False)
                plt.show(block=False) 
                self.get_logger().info("Clearing bot_monitor plot")
            else:
                self.get_logger().info(f"Unrecognized parameter value: {val}")
            # Set parameter to empty value:
            new_param_val = Parameter('bot_plot',rclpy.Parameter.Type.STRING,'')
            self.set_parameters([new_param_val])

def main(args=None):
    rclpy.init(args=args)

    # <...> Initialize a PlotOdom class and call spin
