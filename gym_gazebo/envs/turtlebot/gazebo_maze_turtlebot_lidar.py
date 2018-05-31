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
        self.vel_pub1 = rospy.Publisher('/turtlebot1/mobile_base/commands/velocity', Twist, queue_size=5)#2ayyyyyyy
        self.vel_pub2 = rospy.Publisher('/turtlebot2/mobile_base/commands/velocity', Twist, queue_size=5)#2ayyyyyyy
        self.vel_pub3 = rospy.Publisher('/turtlebot3/mobile_base/commands/velocity', Twist, queue_size=5)#2ayyyyyyy
        
        self.start = [[0,0],[4,-4],[0,-4]]
        self.odom = [[0,0],[0,0],[0,0]]
        
        self.get_OdomSubscriber1()
        self.get_OdomSubscriber2()
        self.get_OdomSubscriber3()
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)
        self._seed()

    def odometryCb1(self,msg):
        self.odom[0][0]= msg.pose.pose.position.x
        self.odom[0][1]= msg.pose.pose.position.y

    def get_OdomSubscriber1(self):
        rospy.Subscriber('/turtlebot1/odom',Odometry,self.odometryCb1)
    
    def odometryCb2(self,msg):
        self.odom[1][0]= msg.pose.pose.position.x
        self.odom[1][1]= msg.pose.pose.position.y

    def get_OdomSubscriber2(self):
        rospy.Subscriber('/turtlebot2/odom',Odometry,self.odometryCb2)

    def odometryCb3(self,msg):
        self.odom[2][0]= msg.pose.pose.position.x
        self.odom[2][1]= msg.pose.pose.position.y

    def get_OdomSubscriber3(self):
        rospy.Subscriber('/turtlebot3/odom',Odometry,self.odometryCb3)
        
    def check_dist(self):
       
        pos = [[0,0],[0,0],[0,0]]
        cnt_robot = 3
    
        for i in range(0,cnt_robot):
            pos[i][0] =  self.start[i][0] + self.odom[i][0]
            pos[i][1] =  self.start[i][1] + self.odom[i][1]
            
        useRobot = [1,1,1]
        for i in range(0,cnt_robot):
            for j in range(i+1,cnt_robot):
                d = math.sqrt( (pos[i][0] - pos[j][0])*(pos[i][0] - pos[j][0]) + (pos[i][1] - pos[j][1])*(pos[i][1] - pos[j][1]))
                print "Dist between ",i+1,j+1," is: ",d
                if d <= 2.0:
                    useRobot[j]=0
        
        return useRobot

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
    
    def Take_Action(self,action, robot_num):
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

        if robot_num == 1:
            self.vel_pub1.publish(vel_cmd)    
        elif robot_num == 2:
            self.vel_pub2.publish(vel_cmd)    
        elif robot_num == 3:
            self.vel_pub3.publish(vel_cmd)    
             
    def reverse_Action(self,action, robot_num):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        vel_cmd = Twist()
        if action == 0: #FORWARD then go back
            vel_cmd.linear.x = -1.0
            vel_cmd.angular.z = 0.0
            
        elif action == 1: #LEFT then go right
            vel_cmd.linear.x = -1.0
            vel_cmd.angular.z = 0.0
           
        elif action == 2: #RIGHT then 
            vel_cmd.linear.x = -1.0
            vel_cmd.angular.z = 0.0
        
        if robot_num == 1:
            self.vel_pub1.publish(vel_cmd)    
        elif robot_num == 2:
            self.vel_pub2.publish(vel_cmd)    
        elif robot_num == 3:
            self.vel_pub3.publish(vel_cmd)   

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action1):
       
        useRobot = self.check_dist() 
        if useRobot[1] == 1: # run robot num 2 normally
            action2 = self.Get_Action(2)
            self.Take_Action(action2,2)
        else :
            print "STOOOOOOOOOOP num 2"
        
        if useRobot[2] == 1: # run robot num 3 normally
            action3 = self.Get_Action(3)
            self.Take_Action(action3,3)
        else :
            print "STOOOOOOOOOOP num 3"
        self.Take_Action(action1,1)
        

        # first agent
        data1 = None
        while data1 is None:
            try:
                data1 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data1.header.frame_id !=  "robot1_tf/sonar2_link":
                    data1 = None
            except:
                pass

               
        #second agent
        data2 = None
        while data2 is None:
            try:
                data2 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data2.header.frame_id !=  "robot2_tf/sonar2_link":
                    data2 = None
            except:
                pass

        data3 = None
        while data3 is None:
            try:
                data3 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data3.header.frame_id !=  "robot3_tf/sonar2_link":
                    data3 = None
            except:
                pass
        
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state1,done1 = self.calculate_observation(data1)
      
        state2,done2 = self.calculate_observation(data2)

        state3,done3 = self.calculate_observation(data3)

        terminated1 = False
        if not done1:
            if action1 == 0:
                reward1 = 5
            else:
                reward1 = 1
        else:
            reward1 = -200
            terminated1 = True
            time.sleep(1)
            print "Revered111 !! "
            self.reverse_Action(action1,1)
            time.sleep(1)


        if useRobot[1] == 1:
            terminated2 = False
            if not done2:
                if action2 == 0:
                    reward2 = 5
                else:
                    reward2 = 1
            else:
                reward2 = -200
                time.sleep(1)
                terminated2 = True
                print "Revered222 !! "
                self.reverse_Action(action2,2)
                time.sleep(1)
            self.Write_All(state2, reward2, done2, [terminated2], 2)
        else :
            self.Write_All(state2, 0, False, [False], 2)
        
        if useRobot[2] == 1:
            terminated3 = False
            if not done3:
                if action3 == 0:
                    reward3 = 5
                else:
                    reward3 = 1
            else:
                reward3 = -200
                time.sleep(1)
                terminated3 = True
                print "Revered333 !! "
                self.reverse_Action(action3,3)
                time.sleep(1)
            self.Write_All(state3, reward3, done3, [terminated3], 3)
        else :
            self.Write_All(state3, 0, False, [False], 3)
        
        return state1, reward1, False, [terminated1]

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
        data1 = None
        while data1 is None:
            try:
                data1 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data1.header.frame_id !=  "robot1_tf/sonar2_link":
                    data1 = None
            except:
                pass

        
        data2 = None
        while data2 is None:
            try:
                data2 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data2.header.frame_id !=  "robot2_tf/sonar2_link":
                    data2 = None
            except:
                pass

        data3 = None
        while data3 is None:
            try:
                data3 = rospy.wait_for_message('/scan', LaserScan, timeout=5)
                if data3.header.frame_id !=  "robot3_tf/sonar2_link":
                    data3 = None
            except:
                pass

        
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state1 = self.calculate_observation(data1)
        state2 = self.calculate_observation(data2)
        state3 = self.calculate_observation(data3)

        self.Write_Observ(state2, 2)
        self.Write_Observ(state3, 3)
        return state1

    def Write_Observ(self, observarion, Turtle_Num, TurtleBot_path = '/home/mostafa/GP_Training/TurtleBot/'):

        parameter_keys = ['obervation','reward','done','info','action']
        parameter_values = [observarion, 0, 0, 0, 0] 
        parameter_dictionary = dict(zip(parameter_keys, parameter_values))
        fileName =  str(TurtleBot_path) +  str(Turtle_Num) + '.json'

        with open(fileName, 'w') as outfile:
            json.dump(parameter_dictionary, outfile)

    def Write_All(self, state, reward, done, info , Turtle_Num, TurtleBot_path = '/home/mostafa/GP_Training/TurtleBot/'):

        parameter_keys = ['obervation','reward','done','info','action']
        parameter_values = [state, reward, done, info, 0] 
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

