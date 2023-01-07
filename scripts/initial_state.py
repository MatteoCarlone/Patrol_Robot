#!/usr/bin/env python

"""
.. module:: initial_state
    :platform: Unix
    :synopsis: Python code to initialize and load the topology

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>

Service:
    /start 
Action:
    /armor_client

This Node start the whole program by

    - Looking for the aruco marker
        - orient the camera using the robot arm
    - decode information about the environment
    - initializing and loading the topology.

"""

#---Libraries---#

import sys
import roslib
import rospy
import actionlib
import time
from os.path import dirname, realpath

from std_srvs.srv import Empty , EmptyResponse
from armor_api.armor_client import ArmorClient

from patrol_robot import environment as env

from patrol_robot.srv import MarkerRoutine
from patrol_robot.srv import RoomInformation
from std_msgs.msg import Int32


#--------------#

class InitialState:

    """
    Class representing the Initial-State of the Smach-State-Machine, which load the 
    Ontology initialized with the environment description, and starting point of the Robot.
    ...

    Methods
    ----------

    __init__(self)

        Initialization of parameters:

            client:ros_action_client
                Armor-Client to set-up the Ontology
            path:str
                The Ontology folder-path to load 
            server:ros_server
                the server of the Empty service /start with Callback execute()

    execute(self,request)

        Server Callback of the /start service requested from the fsm module to start up 
        the whole program.

        This Callback-Server load the ontology, set all the room of the environment with the respective doors,
        the starting position of the robot, already initialized in the starting ontology, the disjoint function
        that let the reasoner understanding all the individuals and I assumed all the rooms visited at time zero 
        in order to yet start the counter for the URGENT rooms.

        All the informations are retrived from aruco markers detection. Therefore the camera mounted on the robot's arm 
        scan the surroundings by achiving different orientations. The movement of the arm is performed by a MoveIt control node.

    """

    def __init__(self):

        # armor - client
        self.client = ArmorClient("armor_client", "reference")

        # absolute ontology path
        self.path = dirname(realpath(__file__))
        self.path = self.path + "/../topology/"

        # /start Empty Server , execute callback
        self.server = rospy.Service(env.SERVER_START , Empty , self.execute)

        # ROS message subscriber on the topic /aruco_detector/id 
        rospy.Subscriber("/aruco_detector/id", Int32, self.id_callback)

        # define empty lists and dictionaries
        self.id_list = []
        self.loc = []
        self.loc_dict = {}
        self.loc_coordinates = {}
        self.coordinates_loc = {}

    def execute(self,request):

        time.sleep(5)

        print('\n###############\nTOPOLOGY LOADING EXECUTION')
        
        # get the current time instant 
        curr_time = int(time.time())

        # wait for move_arm srv to be online
        rospy.wait_for_service('move_arm')
        # Create a client for the 'move_arm' service
        MR_client = rospy.ServiceProxy('move_arm',MarkerRoutine)

        # Call the 'MarkerRoutine' method of the 'move_arm' service with an argument of 1 (MR_left position)
        resp = MR_client(1)
        print(resp.message)
        time.sleep(5)
        # Call the 'MarkerRoutine' method of the 'move_arm' service with an argument of 2 (home position)
        resp = MR_client(2)
        print(resp.message)
        time.sleep(5)
        # Call the 'MarkerRoutine' method of the 'move_arm' service with an argument of 1 (MR_right position)
        resp = MR_client(3)
        print(resp.message)
        time.sleep(5)
        # Call the 'MarkerRoutine' method of the 'move_arm' service with an argument of 1 (MR_back position)
        resp = MR_client(4)
        print(resp.message)
        time.sleep(5)

        # wait for room_info srv to be online
        rospy.wait_for_service('/room_info')
        # Create a client for the 'room_info' service
        MS_client = rospy.ServiceProxy('/room_info',RoomInformation)

        # construct lists and dictionaries from infos retrived from the room_info srv
        for i in self.id_list:
            resp = MS_client(i)
            self.loc.append(resp.room)
            self.loc_dict[resp.room] = resp.connections
            self.loc_coordinates[resp.room] = [resp.x,resp.y]
            self.coordinates_loc[str(resp.x) + ',' + str(resp.y)] = resp.room

            time.sleep(0.5)

        # load dictionaries on the ROS parameter server
        rospy.set_param('ids',self.loc_coordinates)
        rospy.set_param('coord',self.coordinates_loc)

        # Call the 'MarkerRoutine' method of the 'move_arm' service with an argument of 5 (stop position)
        resp = MR_client(5)
        print(resp.message)
        time.sleep(3)


        # load ontology from the absolute path 
        self.client.utils.load_ref_from_file(self.path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23",
                                            True, "PELLET", True, False)  
        
         # -------  Set up all the rooms with respective doors ------- #                      
        self.client.utils.mount_on_ref()
        self.client.utils.set_log_to_terminal(True)

        for i in self.loc:
            connections = self.loc_dict[i]
            print(i)
            for j in connections:
                print(j.through_door)
                self.client.manipulation.add_objectprop_to_ind("hasDoor", i, j.through_door)
        # ----------------------------------------------------------- #

        # Robot starting room
        self.client.manipulation.add_objectprop_to_ind("isIn", 'Robot1', 'E')

        # Set all rooms visited at curr_time time instant
        for room in self.loc:
            self.client.manipulation.add_dataprop_to_ind('visitedAt',room, 'Long', str(curr_time))

        # Disjoint for Individuals understanding
        self.client.call('DISJOINT','IND','',self.loc)

        # First Reasoning
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()

        print('LOADING COMPLETED')

        # returning an empty response to notify the completed load of the ontology
        return EmptyResponse()

    def id_callback(self,msg):
        """
        Callback of the ROS message on the topic /aruco_detector/id
        to retrive codes from aruco markers and stuck them in a list
        """
        if(msg.data not in self.id_list):
            self.id_list.append(msg.data)
            

if __name__ == "__main__":

    # Initialize the ROS-Node
    rospy.init_node(env.NODE_INIT_STATE, log_level=rospy.INFO)

    # Instantiate the node manager class and wait.
    InitialState()
    
    rospy.spin()