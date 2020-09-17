#! /usr/bin/env python
"""
Quadrotor tethered with object
"""
import numpy as np
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState, GetModelState, ApplyBodyWrench
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
from hector_uav_msgs.msg import EnableMotors


class SoloCarrier:

    def __init__(self):
        # env properties
        self.env_type = 'continuous'
        self.name = 'solo_carrier'
        rospy.init_node(self.name, anonymous=True, log_level=rospy.DEBUG)
        self.rate = rospy.Rate(1000) # gazebo world is running at 1000 hz
        self.max_episode_steps = 1000
        self.observation_space_shape = (2,6) # x, y, x_d, y_d, th, th_d
        self.action_space_shape = (6,)
        # robot properties
        self.model_states = ModelStates()
        self.status = 'deactivated'
        # services
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.unpause_physics_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_physics_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.apply_wrench_proxy = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.enable_motors_proxy = rospy.ServiceProxy('/enable_motors', EnableMotors)
        # topic publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # subscriber
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback) # model states are under monitoring

    def reset(self):
        pass

    def step(self, action):
        pass
