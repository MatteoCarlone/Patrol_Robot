#! /usr/bin/env python

"""
.. module:: Controller
    :platform: Unix
    :synopsis: Python code that control the robot trough the via points generated by the planner

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>


Action:

    /armor_client
    motion/controller


This Node implement the controlling action that mainly consists in retrive the via points 
from the planner node and then simulate the movement by losing time while print the via points.
Once reached the final destination the current time and location associated with the robot is updated as the visited time of the room 
for the URGENT property estimation.

"""
#---Libraries---#

import rospy
from actionlib import SimpleActionServer
from patrol_robot.msg import ControlAction, ControlFeedback, ControlResult
from armor_api.armor_client import ArmorClient
from patrol_robot import environment as env
import re
import time 
from std_msgs.msg import Bool


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from patrol_robot.srv import MarkerRoutine

#--------------#

class ControllingAction(object):

    """

    Class representing the Controlling state of the Smach-State-Machine, which manage the robot's motion trough
    via poits sent by request from the action-client motion/controller in the fsm script.

    Methods
    ----------

    __init__(self)

        Initialization of parameters:

            client:ros_action_client
                Armor-Client to set-up the Ontology
            as:ros_action_server
                the server of the motion/controller action  

    execute_callback(self,goal)

        Server Callback of the action motion/controller requested from the fsm module to start up 
        the controlling action towards a target room.

        This Callback-Server simulate the robot's movement by printing its position in time. Once the robot reach 
        the target room some Ontology paramenters are updated: 

        * the robot location (isIn)
        * the time related to the robot (now)
        * the time in which the new target room is visited (visitedAt)

    """

    def __init__(self):

        self.client = ArmorClient('armor_client', 'reference')

        # MoveBase Action message
        self.Goal_msg=MoveBaseGoal() 

        self._battery_low = False

        # MoceBase Action client
        self.mb_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) 

        self._as = SimpleActionServer(env.ACTION_CONTROLLER,
            ControlAction,
            execute_cb=self.execute_callback,
            auto_start=False)

        self._as.start()

        self.achieved = False

        self.sub_battery = rospy.Subscriber(env.TOPIC_BATTERY_LOW, Bool, self._battery_cb)

        self.velocity_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        self.vel_msg = Twist()


        #self.C_interfacehelper = InterfaceHelper()

        # InterfaceHelper Object declaration
        #self._helper = helper   

    def _battery_cb(self,battery_value):

        # store battery state from /battery_low topic message
        self._battery_low = battery_value.data




    def action_client(self):

        self.mb_client.wait_for_server()    #Waiting until the connection to the ActionServer is established

        # Setting some goal's fields
        self.Goal_msg.target_pose.header.frame_id = 'map'            
        self.Goal_msg.target_pose.header.stamp = rospy.Time.now()    
        self.Goal_msg.target_pose.pose.orientation.w = 1

    def done_cb(self,status,result):


        if(status==3):
            print('\nGOAL_REACHED\n')
            self.achieved = True
        else:
            print('Not Reached')
            self.achieved = True

        
    def active_cb(self):
        #No-parameter callback that gets called on transitions to Active.
        #This function is called before the goal is processed
        print("Goald processed...")

    #def feedback_cb(self,feedback):
        #Callback that gets called whenever feedback for this goal is received. Takes one parameter: the feedback.
        
        #rospy.loginfo(")\tFeedback Active, the Modality is running...")

    def set_goal(self,x, y):
        # Creates a goal and sends it to the action server. 
    
        self.Goal_msg.target_pose.pose.position.x = x
        self.Goal_msg.target_pose.pose.position.y = y
        self.mb_client.send_goal(self.Goal_msg, self.done_cb, self.active_cb)

    def execute_callback(self, goal):
    
        print('\n###############\nCONTROLLING EXECUTION')

        loc_coordinates = rospy.get_param('ids')
        coordinates_loc = rospy.get_param('coord')

        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.point_set is None or len(goal.point_set) == 0:
            print('No via points provided! This service will be aborted!')
            self._as.set_aborted()
            return

        result = ControlResult()
        result.reached_point = goal.point_set[-1]

        # map coordinates into locations
        starting_room = coordinates_loc[str(goal.point_set[0].x) + ',' + str(goal.point_set[0].y)]
        reached_room = coordinates_loc[str(result.reached_point.x) + ',' + str(result.reached_point.y)]

        #Setting goal parameters for the action
        self.action_client()

        #Setting a new goal_position
        print(str(goal.point_set[-1].x)+',' +str(goal.point_set[-1].y))
        self.set_goal(goal.point_set[-1].x, goal.point_set[-1].y)
        

        # Initialise the `feedback`
        feedback = ControlFeedback()

        r = rospy.Rate(0.5)
        while(not self.achieved):
            print('moving')
            r.sleep()
            
        rospy.wait_for_service('move_arm')

        MR_client = rospy.ServiceProxy('move_arm',MarkerRoutine)

        resp = MR_client(2)
        print(resp.message)
        rospy.sleep(5)

        self.vel_msg.angular.z = 1

        self.velocity_pub.publish(self.vel_msg)

        rospy.sleep(4)

        self.vel_msg.angular.z = 0

        self.velocity_pub.publish(self.vel_msg)

        resp = MR_client(5)
        print(resp.message)
        rospy.sleep(5)

        # loop to simulate robot moving in time 
        '''
        for point in goal.point_set:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                print('Service has been cancelled by the client!')
                # Actually cancel this service.
                self._as.set_preempted()
                return

            print('  ['+ str("%.2f"%point.x) +','+ str("%.2f"%point.y)+']', end = '\r')

            # update feedback
            feedback.reached_point = point
            self._as.publish_feedback(feedback)

            rospy.sleep(0.5)
        '''
        self.mb_client.cancel_all_goals()

        # replace current robot location
        self.client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', reached_room, starting_room])

        # get current time instant 
        curr_time = int(time.time())

        # get time instant asscociated with the robot
        now = self.client.query.dataprop_b2_ind('now','Robot1')
        # format information
        now = re.search('"(.+?)"',str(now)).group(1)

        # replace robot time intant with the current one 
        self.client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long' , str(curr_time)  , str(now) ])

        # get last time instant the robot visited the reached room
        visited_at = self.client.query.dataprop_b2_ind('visitedAt',reached_room)
        # format information
        visited_at = re.search('"(.+?)"',str(visited_at)).group(1)

        # replace the time instant the robot visited the reached room with the current one
        self.client.call('REPLACE','DATAPROP','IND',['visitedAt', reached_room, 'Long' , str(curr_time)  , str(visited_at) ])

        print('Reached Room: '+reached_room+ ' Coordinate: '+str(result.reached_point.x) + ' , ' + str(result.reached_point.y))
        print('Started from Room: '+ starting_room +' Coordinate: ' + str(goal.point_set[0].x) + ' , ' + str(goal.point_set[0].y))

        self.achieved = False

        self._as.set_succeeded(result)


        return  # Succeeded.

if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(env.NODE_CONTROLLER, log_level=rospy.INFO)
    # Instantiate the node manager class and wait.
    server = ControllingAction()
    rospy.spin()