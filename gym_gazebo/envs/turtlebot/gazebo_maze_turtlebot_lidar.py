import gym
import rospy
import roslaunch
import time
import numpy as np
import math
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from gym.utils import seeding
import json
class GazeboMazeTurtlebotLidarEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "GazeboMazeTurtlebotLidar_v0.launch")
        self.vel_pub = rospy.Publisher('/turtlebot1/mobile_base/commands/velocity', Twist, queue_size=5)#2ayyyyyyy
        self.vel_pub1 = rospy.Publisher('/turtlebot2/mobile_base/commands/velocity', Twist, queue_size=5)#2ayyyyyyy
        
        self.start = [0,0] 
        self.start1 = [0,-1] 
        self.odom = [0,0]
        self.odom1 = [0,0]
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)
        self._seed()

    def odometryCb(self,msg):
        self.odom[0]= msg.pose.pose.position.x
        self.odom[1]= msg.pose.pose.position.y

    def get_OdomSubscriber(self):
        rospy.Subscriber('/turtlebot1/odom',Odometry,self.odometryCb)
    
    def odometryCb1(self,msg):
        self.odom1[0]= msg.pose.pose.position.x
        self.odom1[1]= msg.pose.pose.position.y

    def get_OdomSubscriber1(self):
        rospy.Subscriber('/turtlebot2/odom',Odometry,self.odometryCb1)
        
    def check_dist(self):
        self.get_OdomSubscriber()
        self.get_OdomSubscriber1()

        pos = [0,0]
        pos1 = [0,0]

        pos[0] = self.start[0] + self.odom[0]
        pos[1] = self.start[1] + self.odom[1]
        pos1[0] = self.start1[0] + self.odom1[0]
        pos1[1] = self.start1[1] + self.odom1[1]

        d = math.sqrt( (pos[0] - pos1[0])*(pos[0] - pos1[0]) + (pos[1] - pos1[1])*(pos[1] - pos1[1]))
        print "DIST ",d
        if d <= 2.0:
            return False
        return True

    def discretize_observation(self,data,new_ranges = 100):
        discretized_ranges = []
        min_range = 0.2
        done = False
        mod = (len(data.ranges)/new_ranges)+1
        print data.ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf'):
                    discretized_ranges.append(100)
                elif np.isnan(data.ranges[i]):
                    # print "nan" 
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(data.ranges[i])
            # if data.ranges[i] == float ('Inf') or data.ranges[i] == float ('-Inf'):
            #     print "Range= ",data.ranges[i]    
            if (min_range > data.ranges[i] > 0):#
                done = True
                print "25yran"   
        for i in range (0,4):
            discretized_ranges.append(0)

        return discretized_ranges,done

    def calculate_observation(self,data):
        _ranges = []
        min_range = 0.2
        done = False
        for i, item in enumerate(data.ranges):
            if data.ranges[i] == float ('Inf'):
                _ranges.append(100)
            elif np.isnan(data.ranges[i]):
                _ranges.append(0)
            else:
                _ranges.append(data.ranges[i])
          
            if (min_range > data.ranges[i] > 0):
                done = True

        #print 'DDDDDDDDDDDDDDDATA.Ranges',data
        return _ranges,done
    
    def Take_Action(self,action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        factor = 1
        
        vel_cmd = Twist()
        if action == 0: #FORWARD
            vel_cmd.linear.x = 0.5*factor
            vel_cmd.angular.z = 0.0
            
        elif action == 1: #LEFT
            vel_cmd.linear.x = 0.05*factor
            vel_cmd.angular.z = 0.6
           
        elif action == 2: #RIGHT
            vel_cmd.linear.x = 0.05*factor
            vel_cmd.angular.z = -0.6
        self.vel_pub.publish(vel_cmd)    

    def Take_Action1(self,action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        factor = 1
        
        vel_cmd1 = Twist()
        if action == 0: #FORWARD
            vel_cmd1.linear.x = 0.5*factor
            vel_cmd1.angular.z = 0.0
            
        elif action == 1: #LEFT
            vel_cmd1.linear.x = 0.05*factor
            vel_cmd1.angular.z = 0.6
           
        elif action == 2: #RIGHT
            vel_cmd1.linear.x = 0.05*factor
            vel_cmd1.angular.z = -0.6
        self.vel_pub1.publish(vel_cmd1) 
            
    def reverse_Action(self,action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        factor = 2
        vel_cmd = Twist()
        if action == 0: #FORWARD then go back
            vel_cmd.linear.x = -3.0*factor
            vel_cmd.angular.z = -1.0
            
        elif action == 1: #LEFT then go right
            vel_cmd.linear.x = -3.0*factor
            vel_cmd.angular.z = -1.0
           
        elif action == 2: #RIGHT then 
            vel_cmd.linear.x = -3.0*factor
            vel_cmd.angular.z = 1.0
        
        self.vel_pub.publish(vel_cmd)    

    def reverse_Action1(self,action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        factor = 2
        vel_cmd = Twist()
        if action == 0: #FORWARD then go back
            vel_cmd.linear.x = -3.0*factor
            vel_cmd.angular.z = -1.0
            
        elif action == 1: #LEFT then go right
            vel_cmd.linear.x = -3.0*factor
            vel_cmd.angular.z = -1.0
           
        elif action == 2: #RIGHT then 
            vel_cmd.linear.x = -3.0*factor
            vel_cmd.angular.z = 1.0
        
        self.vel_pub1.publish(vel_cmd)    

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
       
        use2 = self.check_dist() 
        if use2:
            action1 = self.Get_Action(1)
            self.Take_Action1(action1)
        else :
            print "STOOOOOOOOOOP"
        self.Take_Action(action)
        

        # first agent
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data.header.frame_id !=  "robot1_tf/sonar2_link":
                    data = None
            except:
                pass

               
        #second agent
        data1 = None
        while data1 is None:
            try:
                data1 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data1.header.frame_id !=  "robot2_tf/sonar2_link":
                    data1 = None
            except:
                pass

             
        
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state,done = self.calculate_observation(data)
      
        state1,done1 = self.calculate_observation(data1)

        terminated = False
        if not done:
            if action == 0:
                reward = 5
            else:
                reward = 1
        else:
            reward = -200
            terminated = True
            time.sleep(1)
            print "Revered000 !! "
            self.reverse_Action(action)
            time.sleep(1)
        if use2:
            terminated1 = False
            if not done1:
                if action1 == 0:
                    reward1 = 5
                else:
                    reward1 = 1
            else:
                reward1 = -200
                time.sleep(1)
                terminated1 = True
                print "Revered111 !! "
                self.reverse_Action1(action)
                time.sleep(1)
            self.Write_All(state1, reward1, done1, [terminated1], 1)
        else :
            self.Write_All(state1, 0, False, [False], 1)
        return state, reward, False, [terminated]

    def _reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data.header.frame_id !=  "robot1_tf/sonar2_link":
                    data = None
            except:
                pass

        
        data1 = None
        while data1 is None:
            try:
                data1 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data1.header.frame_id !=  "robot2_tf/sonar2_link":
                    data1 = None
            except:
                pass

        
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.calculate_observation(data)
        state1 = self.calculate_observation(data1)

        self.Write_Observ(state1, 1)
        return state

    def Write_Observ(self, observarion, Turtle_Num, TurtleBot_path = '/home/mostafa/GP_Training/TurtleBot/'):
        re = 0
        do = 0
        info = 0
        Action = 0
        parameter_keys = ['obervation','reward','done','info','action']
        parameter_values = [observarion, re, do, info, Action] 
        parameter_dictionary = dict(zip(parameter_keys, parameter_values))
        fileName =  str(TurtleBot_path) +  str(Turtle_Num) + '.json'

        with open(fileName, 'w') as outfile:
            json.dump(parameter_dictionary, outfile)

    def Write_All(self, state, reward, done, info , Turtle_Num, TurtleBot_path = '/home/mostafa/GP_Training/TurtleBot/'):
        Action = 0
        parameter_keys = ['obervation','reward','done','info','action']
        parameter_values = [state, reward, done, info, Action] 
        parameter_dictionary = dict(zip(parameter_keys, parameter_values))
        fileName =  str(TurtleBot_path) + str(Turtle_Num) + '.json'
        with open(fileName, 'w') as outfile:
            json.dump(parameter_dictionary, outfile)


    def Get_Action(self,Turtle_Num, TurtleBot_path = '/home/mostafa/GP_Training/TurtleBot/'):
        params_json = str(TurtleBot_path) + str(Turtle_Num) + '.json'

        with open(params_json) as outfile:
            d = json.load(outfile)
            Action = d.get('action')

        return Action

