import cv2
import numpy as np
#import torch


class multi_agent_algorithm:
    def __init__(self):
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
        pass
        



    def initilize_controller(self, robot_list):
        """ robot_list: list of robot objects that are clicked on the GUI and to be controlled
            
            To be called once at the start of the multi-agent control session
        """
        self.p1x = robot_list[0].position_list[-1][0] #robot1 x position start. in pixels
        self.p1y = robot_list[0].position_list[-1][1] #robot1 y position start. in pixels
        
        self.p2x = robot_list[1].position_list[-1][0] #robot2 x position start. in pixels
        self.p2y = robot_list[1].position_list[-1][1] #robot2 y position start. in pixels

        self.t1x = robot_list[0].trajectory[0][0]      #robot1 x position target. in pixels
        self.t1y = robot_list[0].trajectory[0][1]      #robot1 y position target. in pixels

        self.t2x = robot_list[1].trajectory[0][0]      #robot2 x position target. in pixels
        self.t2y = robot_list[1].trajectory[0][1]      #robot2 y position target. in pixels

        print("Initializing multi-agent controller with:")
        print("Robot1 start pos:  ({},{})".format(self.p1x, self.p1y))
        print("Robot1 target pos: ({},{})".format(self.t1x, self.t1y))   
        print("Robot2 start pos:  ({},{})".format(self.p2x, self.p2y))
        print("Robot2 target pos: ({},{})".format(self.t2x, self.t2y))






                                   



    def run(self, frame, robot_list, arrivalthresh, pixel2um):  
        """ robot_list: list of robot objects 
            arrivalthresh: threshold distance to target to consider arrived (in pix)
            pixel2um: conversion factor from pixels to micrometers

            To be run at each frame of the camera feed thread/loop. Aproxx 24Hz
        """


        alpha = 0
        gamma = 0
        freq = 0

        return frame, alpha, gamma, freq
