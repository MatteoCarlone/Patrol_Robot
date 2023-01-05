#!/usr/bin/env python

"""
.. module:: Finite_State_Machine
    :platform: Unix
    :synopsis: Python code that manage the SMACH finite state machine 

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>

Service:
    /start 
    /reason
    /recharge
Action:
    /armor_client
    motion/controller
    motion/planner

This Node implement the SMACH Finite-State-Machine that guide all the surveillance routine.
The communication with other nodes is managed using ros-services and actions, the actual structure of the service and 
action client is implemented in the script helper.py in the utilities folder.
two sub state machines are implemented to exploit the planning and control states while performing the movement and the recharge routine
composed by a movement to the DOC-Station (Starting Room) and the actual recharging of the battery.

"""

#---Libraries---#

import roslib
import rospy
import smach
import smach_ros
import time
import string

from patrol_robot.helper import InterfaceHelper
from patrol_robot import environment as env

#--------------#
 
#global variable describing the battery state
low = False

class Initial(smach.State):

    """
    Class representing the initial state directly connected with the script initial_state.py .
    When called from the SMACH state machine a client makes a request to the server /start, and then it
    busy waits the response of the finished loading of the Ontology from the server.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible, ROS automatically protect access to variables with mutexes.

    First State runned when main state machine is called.

    * Returns:

    start:(str)
        transition flag for the SMACH fsm to the reason state.

    """


    def __init__(self,interfacehelper):

        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['start'],
                             input_keys=['initial_counter_in'],
                             output_keys=['initial_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper

        print('init_fsm')

    def execute(self, userdata):

        """
        Initial State execution function, called by the smach state machine

        """


        # /start service client request
        self._helper.start_client() 

        print('req srv')

        # looping until transition
        while not rospy.is_shutdown():

            print('initialization')

            # acquire mutex
            self._helper.mutex.acquire()

            try:

    			# look for transition flags:

                # transition to reasoning state
                if self._helper.should_reasoning_start():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with checked flag
                    return 'start'

            finally:

    			# release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)


class Reasoning(smach.State):

    """
    Class representing the reasoning state directly connected with the script reason.py .
    When called from the SMACH state machine a client makes a request to the server /reason, and then it
    busy waits the response of the finished decision of which room will be pointed by the robot.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible, ROS automatically protect access to variables with mutexes.

    * Input Transitions:

    start:(str)
        transition flag for the SMACH fsm from the initial state.

    * Returns:
    

    reasoned:(str)
        transition flag for the SMACH fsm to the moving sub-state machine.
    battery_low:(str)
        transition flag for the SMACH fsm to the recharge sub-state machine.

    """

    def __init__(self,interfacehelper):
        
        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['start','reasoned','battery_low'],
                             input_keys=['reasoning_counter_in'],
                             output_keys=['reasoning_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper



    def execute(self, userdata):
        """
        Reasoning State execution function, called by the smach state machine.

        """
        # recall the low global variable
        global low

        # /reason service client request
        self._helper.reason_client()

        # looping until transition
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags:

                # transition to recharge sub-state machine
                if self._helper.is_battery_low():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # change global battery state 
                    low = True

                    # returning the transition associated with checked flag
                    return 'battery_low'

                # transition to moving sub-state machine
                if self._helper.should_pointing_start():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with checked flag
                    return 'reasoned'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)


class Pointing(smach.State):
    
    """
    Class representing the pointing state directly connected with the script planner.py .
    When the sub-state machine moving is called from the SMACH state machine 
    a action-client makes a request to the action-server motion/planner, and then it
    busy waits the response of the finished planning of the via-poits the robot will follow in the Controlling State.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible, ROS automatically protect access to variables with mutexes.

    First State runned when the Moving sub-state machine is called.

    * Returns:
    
    pointed:(str)
        transition flag for the SMACH fsm to the controlling state of the moving sub-state machine.
    battery_low:(str)
        transition flag for the SMACH fsm to the recharge sub-state machine.

    """

    def __init__(self,interfacehelper):
        
        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['pointed','battery_low'],
                             input_keys=['pointing_counter_in'],
                             output_keys=['pointing_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper

    def execute(self, userdata):
        """
        Pointing State execution function, called by the smach state machine.

        """
        # recall the low global variable
        global low

        # motion/planner action client request, low variable is passed to force the robot 
        # to the DOC location
        self._helper.send_planner_goal(low)

        # looping until transition
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags:

                # transition to recharge sub-state machine
                if self._helper.is_battery_low():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # canceling the goal, the transition may arrive while the action is on going 
                    self._helper.planner_client.cancel_goals()

                    # change global battery state 
                    low = True

                    # returning the transition associated with the checked flag
                    return 'battery_low'

                # transition to the controlling state inside the moving sub state machine 
                if self._helper.planner_client.is_done():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with the checked flag
                    return 'pointed'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)

class Controlling(smach.State):

    """
    Class representing the controlling state directly connected with the script controller.py .
    When called from the SMACH state machine a action-client makes a request to the action-server motion/controller,
    and then it busy waits the response of the finished movement of the robot trough the via points passed by the client 
    and taken from the Pointing State.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible, ROS automatically protect access to variables with mutexes.

    * Input Transitions:

    pointed:(str)
        transition flag for the SMACH fsm from the Pointing state.

    * Returns:
    
    reached:(str)
        transition flag for the SMACH fsm that ends the Moving sub-state machine 
    battery_low:(str)
        transition flag for the SMACH fsm to the recharge sub-state machine.

    """

    def __init__(self,interfacehelper):
        
        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['reached','battery_low'],
                             input_keys=['controlling_counter_in'],
                             output_keys=['controlling_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper

    def execute(self, userdata):
        """
        Controlling State execution function, called by the smach state machine 

        """
        # recall the low global variable
        global low

        # motion/controller action client request
        self._helper.send_controller_goal()

        # looping until transition 
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags:

                # transition to recharge sub-state machine
                if self._helper.is_battery_low():

                    # resetting states when transition occur 
                    self._helper.reset_states()

                    # canceling the goal, the transition mt arrive while the action in on going 
                    self._helper.controller_client.cancel_goals()

                    # change global battery state 
                    low = True

                    # returning the transition associated with checked flag 
                    return 'battery_low'

                # transition that ends the moving sub-state machine 
                if self._helper.controller_client.is_done():    

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with the checked flag 
                    return 'reached'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)

class Recharge(smach.State):

    """
    Class representing the Recharge state directly connected with the script battery.py .
    When the sub-state machine recharge is called from the SMACH state machine 
    a client makes a request to the server /recharge, and then it
    busy waits the response of the finished recharging of the robot's battery. Since the robot can recharge the
    battery only in the DOC-Statetion, if the robot is not there the fsm redirect the the state to the moving sub-state machine 
    nested in the recharge sub-state machine in order to force the robot to DOC before recharging.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible, ROS automatically protect access to variables with mutexes.

    First State runned when the Recharge sub-state machine is called.

    * Returns:
    
    battery_full:(str)
        transition flag for the SMACH fsm that ends the recharge sub-state machine.
    not_at_dock:(str)
        transition flag for the SMACH fsm to Moving sub-state machine in order to get to the DOC-station.

    """

    def __init__(self,interfacehelper):

        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['battery_full','not_at_dock'],
                             input_keys=['recharge_counter_in'],
                             output_keys=['recharge_counter_out'])

        # Interface Object declaration
        self._helper = interfacehelper

    def execute(self, userdata):    
        """
        Recharging State execution function, called by the smach state machine 
        """

        # recall the low global variable        
        global low

        # reasoning on the ontology
        self._helper.reason()

        # getting the actual robot's location 
        isin = self._helper.client.query.objectprop_b2_ind('isIn','Robot1')
        isin = self._helper.list_formatter(isin,'#','>')

        # check weather the robot is not in the starting location
        if env.START_LOC not in isin:

            print('\nROBOT in: ',isin)
            print('Not in START LOCATION, Cannot DOC\n')

            # retunrning transition to the moving sub-state machine nested in the recharge sub-state machine
            return 'not_at_dock'

        # /recharge service client request
        self._helper.recharge_client()

        # looping until transition
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags:

                # transition that ends the recharge sub-state machine 
                if self._helper.is_battery_full():

                    # resetting states when transition occur 
                    self._helper.reset_states()

                    # change global battery state
                    low = False

                    # returning the transition associated with the checked flag
                    return 'battery_full'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)


def main():

    print('fsm')
    # Initialize ros-node
    rospy.init_node(env.NODE_FSM)
    print('fsm2')
    # InterfaceHelper Object Initialization
    interfacehelper = InterfaceHelper()

    
	# Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['fsm'])
    sm_top.userdata.sm_counter = 0

    # ------ Main State Machine ------ #
    with sm_top:
        

        smach.StateMachine.add('INITIAL', Initial(interfacehelper), 
                               transitions={'start':'REASONING'},
                               remapping={'initial_counter_in':'sm_counter', 
                                          'initial_counter_out':'sm_counter'})

        smach.StateMachine.add('REASONING', Reasoning(interfacehelper), 
                               transitions={'start':'REASONING',
                                            'reasoned':'MOVING',
                                            'battery_low':'RECHARGE'},
                               remapping={'reasoning_counter_in':'sm_counter', 
                                          'reasoning_counter_out':'sm_counter'})

        # Create moving sub-state machine
        sm_sub = smach.StateMachine(outcomes=['exit','low'])
        sm_sub.userdata.sm_counter = 0

        # ------ Moving Sub-State Machine ------ #

        with sm_sub:
    
            smach.StateMachine.add('POINTING', Pointing(interfacehelper), 
    					    	   transitions={'pointed':'CONTROLLING',
                                                'battery_low':'low'},

    					    	   remapping={'pointing_counter_in':'sm_counter', 
                                              'pointing_counter_out':'sm_counter'})
        
            smach.StateMachine.add('CONTROLLING', Controlling(interfacehelper), 
    					    	   transitions={'reached':'exit',
                                                'battery_low':'low'},

    					    	   remapping={'controlling_counter_in':'sm_counter', 
    					    				  'controlling_counter_out':'sm_counter'})

        # --------------------------------------- #

        smach.StateMachine.add('MOVING', sm_sub, 
                               transitions={'exit':'REASONING',
                                            'low':'RECHARGE'},

                               remapping={'moving_counter_in':'sm_counter', 
                                          'moving_counter_out':'sm_counter'})   

        # Create recharge sub-state machine 
        sm_recharge = smach.StateMachine(outcomes=['full'])
        sm_recharge.userdata.sm_counter = 0

        # ------ Recharge Sub-State Machine ------ #

        with sm_recharge:

            smach.StateMachine.add('RECH_BAR', Recharge(interfacehelper), 
                                   transitions={'battery_full':'full',
                                                'not_at_dock':'MOVE_TO_DOCK'},

                                   remapping={'bar_counter_in':'sm_counter', 
                                              'bar_counter_out':'sm_counter'})

            smach.StateMachine.add('MOVE_TO_DOCK', sm_sub, 
                                   transitions={'exit':'RECH_BAR',
                                                'low':'MOVE_TO_DOCK'},

                                   remapping={'moving_counter_in':'sm_counter', 
                                              'moving_counter_out':'sm_counter'})

        # --------------------------------------- # 

        smach.StateMachine.add('RECHARGE', sm_recharge, 
                               transitions={'full':'REASONING'},

                               remapping={'recharge_counter_in':'sm_counter', 
                                          'recharge_counter_out':'sm_counter'})

    # --------------------------------------- #
    
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()