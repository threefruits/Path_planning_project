#!/usr/bin/env python

"""
Starter code for EECS C106B Spring 2020 Project 2.
Author: Amay Saxena
"""
import numpy as np
import sys
import math
import matplotlib.pyplot as plt
import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
from proj2_pkg.msg import BicycleCommandMsg, BicycleStateMsg
from proj2.planners import SinusoidPlanner, RRTPlanner, BicycleConfigurationSpace
# tune_set=[[0.15,0],[-0.15,0],[0,0.1],[0,-0.1]]
tune_set=[[0.15,0],[-0.15,0],[0,0.15],[0,-0.15],[0.1,0.1],[0.1,-0.1],[-0.1,0.1],[-0.1,-0.1],[0,0]]

class BicycleModelController(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe)
        self.state = BicycleStateMsg()
        rospy.on_shutdown(self.shutdown)

    def execute_plan(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj: Plan. See configuration_space.Plan
        """
        if len(plan) == 0:
            return
        rate = rospy.Rate(int(1 / plan.dt))
        start_t = rospy.Time.now()
        desired_state_x, desired_state_y, real_state_x, real_state_y = [],[],[],[]
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_t).to_sec()
            if t > plan.times[-1]:
                break
            state, cmd = plan.get(t)
            next_state,next_cmd = plan.get(t+0.01)
            self.step_control(state,next_state , cmd)
            rate.sleep()
            desired_state_x.append(state[0])
            desired_state_y.append(state[1])
            real_state_x.append(self.state[0])
            real_state_y.append(self.state[1])
        plt.plot(desired_state_x, desired_state_y)
        plt.plot(real_state_x, real_state_y)
        plt.show()
        self.cmd([0, 0])
    def distance(self, c1, c2):
        """
        c1 and c2 should be numpy.ndarrays of size (4,)
        """
        theta_d= math.pi - abs((c1[2]-c2[2])%(2*math.pi)-math.pi)
        distance = np.sqrt((c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])+0.15*theta_d*theta_d)
        return distance

    def dynamic_model(self,p1,action):
        next_p3= p1[3]+0.01*action[1]
        next_p2= p1[2]+0.01*3.33*np.tan(next_p3)*action[0]
        next_p=np.array([p1[0]+ 0.01*action[0]*np.cos(next_p2), p1[1]+ 0.01*action[0]*np.sin(next_p2),next_p2 ,next_p3])
        return next_p,action
    
    def cost(self, possible_tune, goal):
        return np.array([self.distance(goal,p[0]) for p in possible_tune])
        
    
    def step_control(self, target_position, next_target_position, open_loop_input):
        """Specify a control law. For the grad/EC portion, you may want
        to edit this part to write your own closed loop controller.
        Note that this class constantly subscribes to the state of the robot,
        so the current configuratin of the robot is always stored in the 
        variable self.state. You can use this as your state measurement
        when writing your closed loop controller.

        Parameters
        ----------
            target_position : target position at the current step in
                              [x, y, theta, phi] configuration space.
            open_loop_input : the prescribed open loop input at the current
                              step, as a [u1, u2] pair.
        Returns:
            None. It simply sends the computed command to the robot.
        """
        
        pre_close_loop_input=(np.zeros([1,2]))[0]
        pre_close_loop_input[0] = open_loop_input[0]  #- np.linalg.norm((target_position-self.state)[:2])
        pre_close_loop_input[1] =   open_loop_input[1] + 0.5*(target_position-self.state)[2]
        possible_tune = [self.dynamic_model(self.state,np.array([pre_close_loop_input[0]+tune[0],pre_close_loop_input[1]+tune[1]])) for tune in tune_set]
        next_p,close_loop_input= possible_tune[np.argmin(self.cost(possible_tune,next_target_position))]

        self.cmd(close_loop_input)
        # self.cmd(open_loop_input)

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : numpy.ndarray
        """
        self.pub.publish(BicycleCommandMsg(*msg))

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...
        
        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        self.state = np.array([msg.x, msg.y, msg.theta, msg.phi])

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd((0, 0))
