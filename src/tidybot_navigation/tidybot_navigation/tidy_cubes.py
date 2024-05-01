#!/usr/bin/env python

import rclpy
from rclpy.duration import Duration # It's unfortunate that we have to use this class a lot, so import the object we need
from rclpy.node import Node

import tf2_ros
import geometry_msgs.msg
import tf_transformations as tft # Alias to tft because that's a big name
import nav2_simple_commander.robot_navigator as nav2sc # Alias because it's a long name

from tidybot_interfaces.msg import CubeContext


import message_filters as mf
import numpy as np
import typing
import enum


class State(enum.Enum):
    # This state is when we can no longer find any cubes to push to the end
    TIDYING_COMPLETE  = 0;
    # The state when we are actively pushing a cube
    # PUSHING_CUBE -> RETURNING_HOME when target reached
    # (alternatively) PUSHING_CUBE -> RETURNING_HOME if the robot gets lost (this sometimes happens)
    PUSHING_CUBE      = 1;
    PUSHING_CUBE_ONE  = 2;
    PUSHING_CUBE_WALL = 3;
    # When we are looking for a cube to push
    # SEARCHING_CUBE -> ALIGNING_CUBE when cube found
    # SEARCHING_CUBE -> TIDYING_COMPLETE when no cube found
    # We have a state for turning left and one for turning right
    SEARCHING_CUBE    = 4;
    # When we are preparing to push a cube
    # ALIGNING_CUBE -> PUSHING_CUBE
    # (alternatively) ALIGNING_CUBE -> SEARCHING_CUBE if the cube gets lost (this should never happen)
    ALIGNING_CUBE     = 5;
    # When we've finished pushing a cube and want to return home
    # RETURNING_HOME -> SEARCHING_CUBE
    RETURNING_HOME    = 6;
    # The state to begin in
    # START_STATE -> RETURNING_HOME
    START_STATE       = 7;
    # An illegal state for debugging
    ILLEGAL           = 999;


class TidyCubes(Node):
    """
    A finite-state machine to tidy cubes
    """
    
    def __init__(self):
        # Init our ros2 node with a name
        super().__init__("tidy_cubes");

        # We use the nav2 simple commander as it gives a programmatic way of dealing with nav2
        # https://navigation.ros.org/commander_api/index.html
        self.navigator = nav2sc.BasicNavigator();


    def send_goal(self, pos: typing.List[float], rotation: typing.List[float], world_space: bool = True):
        """
        Send a target pose to nav2

        Arguments:
            pos      -- The target position in Cartesian coordinates (x,y)
            rotation -- The target rotation in euler roll/pitch/yaw
        """

        # Halt the navigator since we want to do this task now
        self.navigator.cancelTask();
    
        # Send our goal (after getting it)
        self.navigator.goToPose( self.get_goal(pos, rotation, world_space) );
    

            


def main(args=None):
    print('Starting tidy_cubes.py');

    rclpy.init(args=args);

    controller = TidyCubes();

    rclpy.spin(controller);

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node();
    rclpy.shutdown();


# If this script is being imported don't execute main()
# If it's being directly executed, do run main()
if __name__ == '__main__':
    main();
