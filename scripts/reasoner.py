#! /usr/bin/env python

"""
.. module:: Reasoner
    :platform: Unix
    :synopsis: Python to code compute the next room to visit according to a certain protocol

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>

Service: 

    /reason

Action:

    /armor_client


This Node implement the reasoner capable of decide the next-room to be visited by the robot.
It firt retrive the position of the robot and the room it can reach, then it compute the next room giving priority to corridors,
but if there's one or more URGENT rooms it will point randomly one of them.

"""
#---Libraries---#

import rospy
import roslib
import rospy
import actionlib
import random
import re

from patrol_robot import environment as env
from armor_api.armor_client import ArmorClient
from patrol_robot.srv import Reason , ReasonResponse
from patrol_robot.helper import InterfaceHelper

#--------------#

class Reasoner:

   """
    Class representing the Reasoning-State of the Smach-State-Machine, which decide the 
    next room to point whenever a client-request on the custom-service /reason is perfomed by the 
    finite state machine.

    Methods
    ----------

    __init__(self)

        Initialization of parameters:

            client:ros_action_client 
               Armor-Client to set-up the Ontology
            _helper:InterfaceHelper (object)
               Object define in the intefacehelper script in utilities folder
            reachable_list:list[]
               list of reachable rooms 
            urgent_list:list[]
               list of urgent rooms
            corridors:list[]
               list of corridors

    execute(self,request)

        Server Callback of the /reason service requested from the fsm module to reason 
        on the next room to point.

        This Callback-Server updates the position of the robot, the room that can reach, the urgent rooms (if any) and 
        the corridors. Then it calls the private method _next_room(self).

    _next_room(self)

        This private method computes the new room according to the following protocol:

            corridor has higher prioprity than normal rooms
            if there's one or more urgent rooms and the robot can reach them it will prioritize them randomly

    """

   def __init__(self):

      rospy.Service(env.SERVER_REASON , Reason , self.execute)

      self.client = ArmorClient("armor_client", "reference")

      interfacehelper = InterfaceHelper()
      self._helper = interfacehelper

      self.reachable_list = []
      self.urgent_list = []
      self.corridors = []

   def execute(self,request):

      print('\n###############\nREASONING EXECUTION')

      # reason on the ontology
      self._helper.reason()

      # get the current robot location
      isin = self.client.query.objectprop_b2_ind('isIn','Robot1')
      # get the current robot time instant   
      now = self.client.query.dataprop_b2_ind('now','Robot1')

      print('Robot isIn: '+ re.search('#(.+?)>',isin[0]).group(1) + ' at time: ' + re.search('"(.+?)"',str(now)).group(1))

      # get the robot's reachable rooms
      can_reach = self.client.query.objectprop_b2_ind('canReach','Robot1')
      # get the list of current urgent rooms
      urgent_list = self.client.query.ind_b2_class('URGENT')
      # get the list of Corridors
      corridors = self.client.query.ind_b2_class('CORRIDOR')

      # format informations
      self.reachable_list = self._helper.list_formatter(can_reach,'#','>')
      self.urgent_list = self._helper.list_formatter(urgent_list,'#','>')
      self.corridors = self._helper.list_formatter(corridors,'#','>')

      print('Robot canReach Rooms: ',self.reachable_list)

      # compute the next room to go according with the protocol
      room_to_go = self._next_room()

      # get the last time instant in which thw robot has seen the targeted room
      visited_at = self.client.query.dataprop_b2_ind('visitedAt',room_to_go)
      # format information
      visited_at = re.search('"(.+?)"',str(visited_at)).group(1)

      print('Next Room: '+ room_to_go + ' lastly visited at time: ' + str(visited_at))

      # erase the list of reachable rooms
      self.reachable_list = []

      # returning the target room
      return ReasonResponse(room_to_go)

   def _next_room(self):

      # Implementation of the protocol giving priority to urgent rooms, 
      #if there's no urgent the highest prio goes to corridors.
      # I do random choice if more than one room satisfies the protocol.

      RU_room = [i for i in self.reachable_list if i in self.urgent_list]
      
      if not RU_room:

         RC_room = [i for i in self.reachable_list if i in self.corridors]
         

         if not RC_room:
            to_point = random.choice(self.reachable_list) 

         else:
            to_point = random.choice(RC_room)
      else:
        
         to_point = random.choice(RU_room)

      # returning the target room
      return to_point


def main():

   # Initialize the ROS-Node  
   rospy.init_node(env.NODE_REASONER, log_level=rospy.INFO)
   # Instantiate the node manager class and wait.
   Reasoner()
   rospy.spin()

if __name__ == '__main__':

   main()