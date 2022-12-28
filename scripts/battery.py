#!/usr/bin/env python

"""

.. module:: Battery
    :platform: Unix
    :synopsis: Python code that simulate the robot's battery recharging at the doc-station

.. moduleauthor:: Matteo Carlone <matteo.carlone99@gmail.com>

Service: 

    /reason

Action:

    /armor_client


This Node implement the recharging bar visible at screen that starts whenever the robot battery is low and it 
is in the DOC-Station (Starting Room). In addition to this, in a new thread, a random notifier simulate the decrease of the robot's
battery while moving in the environment.


"""

#---Libraries---#

import sys
import roslib
import rospy
import threading
import random

from std_msgs.msg import Bool
from patrol_robot.helper import InterfaceHelper
from std_srvs.srv import Empty , EmptyResponse
from patrol_robot import environment as env

#--------------#

class Battery(object):

    """
    Class representing the Robot's battery, it mainly has two functions a random alarm that notifies that the battery is low 
    and need a recharge, a loading bar visible on screen that simulates the recharging action at the DOC-station. 

    Methods
    ----------

    __init__(self)

        Initialization of parameters:

            server:ros_server
                the server of the Empty service /recharge with Callback execute()
            _helper:InterfaceHelper (object)
                Object define in the intefacehelper script in utilities folder
            _battery_low:Bool
                variable that represent the state of the battery
            _random_battery_time:list[]
                ROS parameter containing the interval of seconds in which the random notifier works
            th:thread
                the thread in which the random notifier works

    execute(self,request)

        Server Callback of the /recharge service requested from the fsm module when the battery has to be rehcarged.

        This Callback-Server calls the private method _printProgressBar(...) that simulates the recharging action.

    _printProgressBar(self,iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '¦', printEnd = "backslash r")
        
        Call in a loop to create terminal progress bar
        
        Parameters:

            iteration - Required  : current iteration (Int) \n
            total - Required  : total iterations (Int) \n
            prefix - Optional  : prefix string (Str) \n
            suffix - Optional  : suffix string (Str) \n
            decimals - Optional  : positive number of decimals in percent complete (Int) \n
            length - Optional  : character length of bar (Int) \n
            fill - Optional  : bar fill character (Str) \n
            printEnd - Optional  : end character (e.g. "backslash r", "backslash n") (Str)

    _random_notifier(self)

        This method executes in another thread and basically simulate the randomness dreasing of the robot's battery.
        It publishes the state of the battery on the state/battery_low topic (Bool ROS message)

    """
	
    def __init__(self):

        self.server = rospy.Service(env.SERVER_RECHARGE, Empty , self.execute)

        interfacehelper = InterfaceHelper()
        self._helper = interfacehelper

        self._battery_low = False

        self._random_battery_time = rospy.get_param(env.RND_BATTERY_TIME)
        
        th = threading.Thread(target=self._random_notifier)
        th.start()

    def execute(self,request):

        print('\n###############\nRECHARGE EXECUTION')

        # get robot current position from ontology
        isin = self._helper.client.query.objectprop_b2_ind('isIn','Robot1')
        isin = self._helper.list_formatter(isin,'#','>')

        # prtint to see that the robot docked for recharge in the starting room
        print('The robot docked for recharge in: ' + isin[0] + '\n')

        # A List of Items
        items = list(range(0, 57))
        l = len(items)

        # Initial call to print 0% progress
        self._printProgressBar(0, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

        # animate the progress bar
        for i, item in enumerate(items):

        	rospy.sleep(0.1)
        	self._printProgressBar(i + 1, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

        print('BATTERY FULL')

        # return an empty response to notify the completed recharge
        return EmptyResponse()


    def _random_notifier(self):

    	# ROS message publisher on the topic /battery_low 
    	publisher = rospy.Publisher(env.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)

    	while not rospy.is_shutdown():

    	    # Wait for simulate battery usage.
    	    delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
    	    rospy.sleep(delay)
    	    # Change battery state.
    	    self._battery_low = True
    	    # Publish battery level.
    	    publisher.publish(Bool(self._battery_low))

    def _printProgressBar (self,iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '¦', printEnd = "\r"):

        percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
        filledLength = int(length * iteration // total)
        bar = fill * filledLength + '-' * (length - filledLength)
        print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
        # Print New Line on Complete
        if iteration == total: 
            print()


if __name__ == '__main__':

	# Initialize the ROS-Node
	rospy.init_node(env.NODE_BATTERY, log_level=rospy.INFO)
	# Instantiate the node manager class and wait.
	Battery()
	rospy.spin()

