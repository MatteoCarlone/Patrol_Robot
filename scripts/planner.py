#! /usr/bin/env python

"""
.. module:: Planner
    :platform: Unix
    :synopsis: Python code that plan a set via_points from a room to another

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>


Action:

    /armor_client
    motion/planner


This Node implement the planning action of creating a set of via points between a room to another. 
Those via points will be then passed to the Controlling node to perform the actual movement.

"""
#---Libraries---#

import rospy
from actionlib import SimpleActionServer
from patrol_robot.msg import Point, PlanAction, PlanFeedback, PlanResult
from armor_api.armor_client import ArmorClient
from patrol_robot import environment as env
import numpy as np
import re
from std_msgs.msg import Bool


#--------------#

class PlaningAction(object):

    """

    Class representing the Planning state of the Smach-State-Machine, which creates a set of via points between the robot position
    and a target room to be pointed decided by the Reasoner State and passed to this ros node via a motion/planner action-client request
    in the fsm script.

    Methods
    ----------

    __init__(self)

        Initialization of parameters:

            client:ros_action_client
                Armor-Client to set-up the Ontology
            as:ros_action_server
                the server of the motion/planner action 
            _environment_size:list[]
                ROS parameter containing the coordinate limits of the environment 

    execute_callback(self,goal)

        Server Callback of the action motion/planner requested from the fsm module to start up 
        the via points generation action.

        This Callback-Server simulate the robot's planning by generating a set of n poits equally spacied from a room to another. 

    """

    def __init__(self):

        self._environment_size = rospy.get_param(env.ENV_SIZE)

        self.client = ArmorClient('armor_client', 'reference')

		# Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(env.ACTION_PLANNER, 
                                      PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)

        # Initialize the starting room
        self.start_room = env.START_LOC
        # Define boolean variable for the battery state 
        self._battery_low = False
        # # ROS message subscriber on the topic /battery_low 
        self.sub_battery = rospy.Subscriber(env.TOPIC_BATTERY_LOW, Bool, self._battery_cb)

        # start plan action server
        self._as.start()

    def _battery_cb(self,battery_value):

        """
        Callback function for /battery_low topic. Stores the battery state.

        """

        # store battery state from /battery_low topic message
        self._battery_low = battery_value.data

    def execute_callback(self, goal):

        """
        Callback function for the action server.

        """

        print('\n###############\nPLANNING EXECUTION')

        # Get list of room coordinates from ROS parameters
        loc_coordinates = rospy.get_param('ids')

        if self._battery_low:

            # Wait until robot is in a room during the low battery routine 
            print('BATTERY_LOW')

            # Ontology Reasoning
            self.client.utils.apply_buffered_changes()
            self.client.utils.sync_buffered_reasoner()
            # Query ontology for current location of robot
            current_start_room = self.client.query.objectprop_b2_ind('isIn','Robot1')
            # Format room name from ontology response
            current_start_room = re.search('#(.+?)>',current_start_room[0]).group(1)

            # Loop until robot is no longer in the previous target room
            r = rospy.Rate(1)
            while(current_start_room == self.start_room):

                # Ontology Reasoning
                self.client.utils.apply_buffered_changes()
                self.client.utils.sync_buffered_reasoner()
                # Query ontology for current location of robot
                current_start_room = self.client.query.objectprop_b2_ind('isIn','Robot1')
                # Format room name from ontology response
                current_start_room = re.search('#(.+?)>',current_start_room[0]).group(1)
                r.sleep()
                
            # Check if robot is already in target room
            if current_start_room == goal.target:
                self._as.set_aborted()
                return

        # get the current robot location
        self.start_room = self.client.query.objectprop_b2_ind('isIn','Robot1')
        # format information
        self.start_room = re.search('#(.+?)>',self.start_room[0]).group(1)

        # mapping start_room location into coordinates
        start_point = loc_coordinates[self.start_room]


    	# mapping the target location into coordinates
        target_point = loc_coordinates[goal.target]

        print('Start Room: '+self.start_room+'\n')
        print('Target Room: '+goal.target+'\n')

        log_msg = (f'Starting-Room [{start_point[0]}, {start_point[1]}] , Target-Room [{target_point[0]}, '
                       f'{target_point[1]}] ')
        print(log_msg)

        # check if locations are None
        if start_point is None or target_point is None:

            log_msg = 'Cannot have `None` start point nor target_point. This service will be aborted!.'
            print(log_msg)
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return

        # check if locations' coordinates are in the environment limit
        if not(self._is_valid(start_point) and self._is_valid(target_point)):
            log_msg = (f'Start point ({start_point[0]}, {start_point[1]}) or target point ({target_point[0]}, '
                       f'{target_point[1]}) point out of the environment. This service will be aborted!.')
            print(log_msg)
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return

        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []

        # number of via points
        n_points = 10

        # generate a set of n equally spaced via points
        points_x = np.linspace(start_point[0],target_point[0],num = n_points)
        points_y = np.linspace(start_point[1],target_point[1],num = n_points)

        # format information
        points = [[a , b] for a, b in zip(points_x, points_y)]

        print('GENERATING VIA POINTS...')

        # loop to simulated time in generating via points
        for i in range(n_points):

            if self._as.is_preempt_requested():
                print('Server has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()  
                return

            # create a Point struct with current location's coordinates
            new_point = Point()
            new_point.x = points[i][0]
            new_point.y = points[i][1]

            print('[' + str("%.2f"% new_point.x)+','+str("%.2f"% new_point.y)+']')

            # update feedback
            feedback.via_points.append(new_point)
            # publish feedback
            self._as.publish_feedback(feedback)

            rospy.sleep(0.1)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points

        self._as.set_succeeded(result)


    def _is_valid(self, point):
        
        # return bool variable, true coordinates are in the env limits, false viceversa
        return self._environment_size[0] <= point[0] <= self._environment_size[1] and self._environment_size[2] <= point[1] <= self._environment_size[3]


if __name__ == '__main__':

    # Initialise the node, its action server, and wait.    
    rospy.init_node(env.NODE_PLANNER, log_level=rospy.INFO)
    # Instantiate the node manager class and wait.
    server = PlaningAction()
    rospy.spin()