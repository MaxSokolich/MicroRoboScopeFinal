import cv2
import numpy as np
#import torch
from classes.Control.mlp.toMax.implementation import generate_reference_path
from classes.Control.mlp.toMax.xy2pxl import CoordinateMapper

from stable_baselines3 import PPO
import time
import math

class multi_agent_algorithm:
    def __init__(self, pix2metric, video_width, video_height):
        """ Multi-agent control algorithm for 2 hetereogenous microrobots using MPC with a trained MLP model
        width of frame: 2448 pixels
        height of frame: 2048 pixels
        
        (0,0) --------------------------------
        |                                    |
        |                                    |
        |                                    |
        |                                    |
        |               * (p1x, p1y)          |
        |                                    |
        |                                    |
        |                                    |
        |                                    |
        |                                    |          
        ----------------------------------(2448,2048)
        """
        self.counter = 0
        self.node = 0 
        self.start = time.time()

        self.pix2metric = pix2metric
        self.width = video_width#um
        self.height = video_height#um

        self.model =  PPO.load(r'C:\Users\Das_Lab_Admin\Desktop\REPOS\MicroRoboScopeFinal\classes\Control\mlp\toMax\best_model.zip',device="cpu")

        self.prev_action = np.zeros(2, dtype=np.float32)

        
        self.myMapper = CoordinateMapper(self.width, self.height)


        self.path, self.actions_chopped = None, None

        self.bot1_threshold = 50
        self.num_alpha_bins = 4   # angles discretized into 4 bins




        



    def initilize_controller(self, p1,p2,t1,t2):
        """ robot_list: list of robot objects that are clicked on the GUI and to be controlled
            
            To be called once at the start of the multi-agent control session
        """


        #self.my_mlp_controller = MLP_controller()
        #self.my_mlp_controller.N = 1


        #Get pixel coordinate information in opencv frame and convrt to ums
        start_robot1_x = int(p1[0] * self.pix2metric)
        start_robot1_y = int(p1[1] * self.pix2metric) 

        target_robot1_x = int(t1[0] * self.pix2metric)
        target_robot1_y = int(t1[1] * self.pix2metric)

        start_robot2_x = int(p2[0] * self.pix2metric)
        start_robot2_y = int(p2[1] * self.pix2metric) 
        
        target_robot2_x = int(t2[0] * self.pix2metric)
        target_robot2_y = int(t2[1] * self.pix2metric) 

        print("robot1 position (opencv, um):", (start_robot1_x, start_robot1_y))

        #convert opencv um coordinate information to mehdi coordinate system
        start_robot1_x, start_robot1_y = self.myMapper.pixel_to_xy(start_robot1_x, start_robot1_y)
        start_robot2_x, start_robot2_y = self.myMapper.pixel_to_xy(start_robot2_x, start_robot2_y)

        target_robot1_x, target_robot1_y = self.myMapper.pixel_to_xy(target_robot1_x, target_robot1_y)
        target_robot2_x, target_robot2_y = self.myMapper.pixel_to_xy(target_robot2_x, target_robot2_y)

        print("robot1 position (mehdis, um):", (start_robot1_x, start_robot1_y))
        



        #mehdis coordinate in um
        initial_configuration = np.array([start_robot1_x, start_robot1_y, start_robot2_x,  start_robot2_y])
        final_configuration = np.array([target_robot1_x, target_robot1_y, target_robot2_x, target_robot2_y])


        #initial_configuration = np.array([1.1, 1.1 , -10.1 , -1.1])
        #final_configuration = np.array([10 ,-10.1 , 40.1 , 35.1])

        #mehdis um frame
        # self.path, self.actions_chopped = generate_reference_path(initial_configuration,final_configuration, plot = False)  #mehdis coordnate system, um
        trajs = np.load('classes/Control/traj_fixed_dis10.npy', allow_pickle=True)
        li = [(i,len(t)) for i, t in enumerate(trajs)]
        ##sort by length
        li.sort(key=lambda x: x[1])

        traj_idx = li[150][0]  # or any index you want
        selected_traj = np.array(trajs[traj_idx]) 
        self.path = selected_traj - selected_traj[0]+initial_configuration
        print("path: ", self.path)


        #mehdis um frame ---> opencv um frame
        self.opencv_um_path_robot1 = self.myMapper.xy_to_pixel_batch(self.path[:,0:2])
        self.opencv_um_path_robot2 = self.myMapper.xy_to_pixel_batch(self.path[:,2:4])

        print("do we get here", self.opencv_um_path_robot1)






                                   
    def discrete_to_continuous_action(self, discrete_action):
        """
        Convert discrete action indices to continuous values.
        
        Args:
            discrete_action: Array-like of shape (2,) with [f_index, alpha_index]
            
        Returns:
            Tuple of (f, alpha) continuous values
        """
        f_index, alpha_index = discrete_action[0], discrete_action[1]
        
        # Convert f_index (0-24) directly to frequency value
        f = float(f_index)
        
        # Convert alpha_index to continuous angle value
        # Map alpha_index (0 to num_alpha_bins-1) to angle range (-pi to pi)
        alpha_min = -np.pi
        alpha_max = np.pi
        alpha_step = (alpha_max - alpha_min) / self.num_alpha_bins
        alpha = alpha_min + alpha_index * alpha_step + alpha_step / 2  # center of bin
        
        return f, alpha


    def run(self, robot_list, frame):  
        """ robot_list: list of robot objects 
        

            To be run at each frame of the camera feed thread/loop. Aproxx 24Hz
        """
        
        if self.counter == 0:
            p1 = robot_list[0].position_list[-1]  #robot1 start [x1, y1]
            p2 = robot_list[1].position_list[-1]  #robot2 start

            t1 = robot_list[0].trajectory[0]      #robot1 target
            t2 = robot_list[1].trajectory[0] 
            self.initilize_controller(p1,p2,t1,t2)



        self.counter +=1
       
        
       
        for i in range(len(self.opencv_um_path_robot1)):  
            cv2.circle(frame,(int(self.opencv_um_path_robot1[i,0] / self.pix2metric), int(self.opencv_um_path_robot1[i,1]/ self.pix2metric)),2,(255,0,0), -1,)

        for j in range(len(self.opencv_um_path_robot2)):  
            cv2.circle(frame,(int(self.opencv_um_path_robot2[j,0] / self.pix2metric), int(self.opencv_um_path_robot2[j,1]/ self.pix2metric)),2,(0,165,255), -1,)




        #robot data in opencv/um
        robot1_x = robot_list[0].position_list[-1][0] * self.pix2metric #um
        robot1_y = robot_list[0].position_list[-1][1] * self.pix2metric #um

        robot2_x = robot_list[1].position_list[-1][0] * self.pix2metric #um
        robot2_y = robot_list[1].position_list[-1][1] * self.pix2metric #um

        # robot data in mehdi/um
        robot1_x, robot1_y = self.myMapper.pixel_to_xy(robot1_x, robot1_y)
        robot2_x, robot2_y = self.myMapper.pixel_to_xy(robot2_x, robot2_y)



        #mehids um coordinate system
        bot1_target_x = self.path[self.node,0] 
        bot1_target_y = self.path[self.node,1]
        
        # plotting target position node for robot 2
        bot1_target_x_opencv, bot1_target_y_opencv = self.myMapper.xy_to_pixel(bot1_target_x, bot1_target_y)
        cv2.circle(frame,(int(bot1_target_x_opencv / self.pix2metric), int(bot1_target_y_opencv / self.pix2metric)),10,(255,0,0), -1,)


        bot2_target_x = self.path[self.node,2]
        bot2_target_y = self.path[self.node,3]

        # plotting target position node for robot 2
        bot2_target_x_opencv, bot2_target_y_opencv = self.myMapper.xy_to_pixel(bot2_target_x, bot2_target_y)
        cv2.circle(frame,(int(bot2_target_x_opencv / self.pix2metric), int(bot2_target_y_opencv / self.pix2metric)),10,(255,0,0), -1,)


         #calculate error between node and robot in mehdi/um
        error_bot1 = np.array([(robot1_x - bot1_target_x) , (robot1_y - bot1_target_y)], dtype=np.float32)
        error_bot2 = np.array([(robot2_x - bot2_target_x) , (robot2_y - bot2_target_y)], dtype=np.float32)
        observation_error = np.zeros((2, 4), dtype=np.float32)
        observation_error[0, :2] = error_bot1
        observation_error[1, :2] = error_bot2
        observation_error[:, 2:] = self.prev_action
        action, _ = self.model.predict(observation_error, deterministic=True)
        freq,alpha = self.discrete_to_continuous_action(action)
        # print("action =" , action)
        print("freq = %.3f, alpha = %.3f" % (freq, alpha))
        print("node = ", self.node)

    


        dist_error = np.sqrt(error_bot1[0] ** 2 + error_bot1[1] ** 2)


        if (dist_error < self.bot1_threshold) or (self.counter > 30):
            self.counter = 0 
            self.node += 1

        
        self.prev_action = np.asarray(action, dtype=np.float32)


        alpha = action[1] 
        gamma = np.pi/2   #disregard
        freq = math.ceil(action[0])


        return frame, alpha, gamma, freq
