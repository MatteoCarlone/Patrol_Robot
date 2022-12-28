#!/usr/bin/env python

"""
.. module:: Environment 
    :platform: Unix
    :synopsis: Python to initialize some variables shared by the all program

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>

This helper module contains many static variables that are used and shared by all the nodes in the script section. 

"""
#-----------------------------------------#

"""
Intialization of enivornmental variables regarding the rooms present in the scenario associated with a set of coordinates [x,y] .

"""

#-----------------------------------------#

# starting location
START_LOC = 'E'

# List of locations.
Loc = ['E','C1','C2','R1','R2','R3','R4']

Map_R = {
    'E': [0,0],

    'C1': [1,2],

    'C2': [-1,2],

    'R1': [2,1],

    'R2': [2,3],

    'R3': [-2,1],

    'R4': [-2,3]}

Map_C = {
    '0.0,0.0': 'E',

    '1.0,2.0': 'C1',

    '-1.0,2.0':'C2',

    '2.0,1.0': 'R1',

    '2.0,3.0': 'R2',

    '-2.0,1.0': 'R3',

    '-2.0,3.0': 'R4'}

#-----------------------------------------#

"""
ROS parameters for the environment size and time interval for the random battery notifier function in the battery.py script.

"""

#-----------------------------------------#

ENV_SIZE = 'config/environment_size'
RND_BATTERY_TIME = 'config/random_battery_time'

#-----------------------------------------#

"""
Declaration of ROS-Node names, ROS-Service names and ROS-Action names

"""

#-----------------------------------------#

# The name of the FSM node.
NODE_FSM = 'fsm'

# The name of the node that loads the topology.
NODE_INIT_STATE = 'initial_state'

# The name of the server to start the topology loading.
SERVER_START = 'start'

# The name of the reasoner node.
NODE_REASONER = 'reasoner'

# The name of the server to start reasoning.
SERVER_REASON = 'reason'

# The name of the battery node.
NODE_BATTERY = 'battery'

# The name of the server to start recharging the battery.
SERVER_RECHARGE = 'recharge'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# The name of the controller node.
NODE_CONTROLLER = 'controller'

# The name of the action server solving the motion control problem.
ACTION_CONTROLLER = 'motion/controller'

# The name of the planner node.
NODE_PLANNER = 'planner'

# The name of the action server solving the motion planning problem.
ACTION_PLANNER = 'motion/planner'

#-----------------------------------------#



