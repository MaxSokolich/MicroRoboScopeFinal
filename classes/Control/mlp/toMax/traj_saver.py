import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from copy import deepcopy

import gymnasium as gym
from gymnasium.spaces import Box, Discrete
import gurobipy as gp
from gurobipy import GRB
import torch
import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

import json
import os
from collections import deque
#from chop import Chop
from matplotlib.animation import FuncAnimation

def load_data(data, ds = 10, plot = False):
    initial_configuration = np.array(data['initial_configuration'])  
    all_commands = np.array(data['all_commands'])
    T_values = np.array((data['T_values'])) 
    # n_repetitions = []
    # for t in T_values:
    #     n_repetitions.append(int(t/dt))
    # n_repetitions = np.array(n_repetitions)
    # all_commands = np.repeat(all_commands, n_repetitions, axis=0)
    # T_values = np.repeat(T_values, n_repetitions, axis=0) 
     
    actions = []
    for vec in all_commands:
        actions.append(vec)
    
    chop_c = Chop(all_commands, T_values, initial_configuration)
    chop_step = chop_c.get_chop_step()
    path, actions_chopped = chop_c.get_chopped_trajectory(chop_step, ds = ds)
    obs_ls = []
    goal = np.array(path[-1])
    N =2
    for i in range(len(path)):
        agents_array = np.asarray(path[i]).flatten()
        # goals_array = np.asarray([10, -10.1, 40.1, 35.1]).flatten()
        obs_ls.append(agents_array)
        # obs = {"agents": np.array(path[i]).reshape(2, 2), "goals": goal}
        # obs_ls.append(obs)
    path1 = path[:,0:2]
    path2 = path[:,2:4]
    # actions_chopped = actions*chop_step
    # actions_chopped = np.array(actions_chopped)

    if plot:
        fig, ax = plt.subplots()
        ax.plot(path1[:,0], path1[:,1], 'o-', color = 'blue')
        ax.plot(path2[:,0], path2[:,1], 'o-', color = 'red')
        ax.plot(initial_configuration[0], initial_configuration[1], 'go')
        ax.plot(initial_configuration[2], initial_configuration[3], 'go')
        ax.plot(path1[-1][0], path1[-1][1], 'o', color = 'black')
        ax.plot(path2[-1][0], path2[-1][1], 'o', color = 'black')
        ###plot goal with star purple
        ax.plot(goal[0], goal[1], marker='*', color='purple')
        ax.plot(goal[2], goal[3], marker='*', color='purple')
        # ax.set_aspect('equal')
        ax.legend(['robot1', 'robot2', 'initial1', 'initial2','final1', 'final2', 'goal1', 'goal2'])
        # plt.show()
        plt.savefig('test.png')
        plt.close()


    
    # Wrap in DictObs

    obs_ls = np.array(obs_ls)
    return path, actions_chopped



def plot_raw_traj(id_trj):
    folder = "/home/mker/ubot_RL/Multi-Layer-Perceptron-Experiments/RL_approach/imitation_data"
    fname_ls = os.listdir(folder)
    size_data = 100
    traj = []
    fname = fname_ls[id_trj]


    with open(os.path.join(folder, fname), "r") as f:
        data = json.load(f)
    initial_configuration = np.array(data['initial_configuration'])  
    all_commands = np.array(data['all_commands'])
    T_values = np.array((data['T_values'])) 
    # n_repetitions = []
    # for t in T_values:
    #     n_repetitions.append(int(t/dt))
    # n_repetitions = np.array(n_repetitions)
    # all_commands = np.repeat(all_commands, n_repetitions, axis=0)
    # T_values = np.repeat(T_values, n_repetitions, axis=0) 
     
    actions = []
    for vec in all_commands:
        actions.append(vec)
    
    chop_c = Chop(all_commands, T_values, initial_configuration)
    chop_step = chop_c.get_chop_step()
    print(f'chop step: {chop_step}')
    path, actions_chopped = chop_c.get_chopped_trajectory(chop_step, ds = 10)
    obs_ls = []
    goal = np.array(path[-1])
    N =2
    for i in range(len(path)):
        agents_array = np.asarray(path[i]).flatten()
        # goals_array = np.asarray([10, -10.1, 40.1, 35.1]).flatten()
        obs_ls.append(agents_array)
        # obs = {"agents": np.array(path[i]).reshape(2, 2), "goals": goal}
        # obs_ls.append(obs)
    path1 = path[:,0:2]
    path2 = path[:,2:4]
    # actions_chopped = actions*chop_step
    # actions_chopped = np.array(actions_chopped)

    fig, ax = plt.subplots()

    # Static elements
    ax.plot(goal[0], goal[1], marker='*', color='purple')
    ax.plot(goal[2], goal[3], marker='*', color='purple')
    ax.plot(path1[-1][0], path1[-1][1], 'o', color='black')
    ax.plot(path2[-1][0], path2[-1][1], 'o', color='black')
    ax.plot(initial_configuration[0], initial_configuration[1], 'go')
    ax.plot(initial_configuration[2], initial_configuration[3], 'go')
    ax.legend(['goal1', 'goal2', 'final1', 'final2', 'initial1', 'initial2'])

    # Axes setup
    ax.set_xlim(min(path1[:,0].min(), path2[:,0].min()) - 1, max(path1[:,0].max(), path2[:,0].max()) + 1)
    ax.set_ylim(min(path1[:,1].min(), path2[:,1].min()) - 1, max(path1[:,1].max(), path2[:,1].max()) + 1)
    ax.set_aspect('equal')

    # Lines for animation
    robot1_line, = ax.plot([], [], 'o-', color='blue')
    robot2_line, = ax.plot([], [], 'o-', color='red')

    # === Animation settings ===
    pause_between_frames = 5  # how many times to repeat each frame
    frame_indices = []
    for i in range(len(path1)):
        frame_indices.extend([i] * pause_between_frames)  # repeat each frame

    # Update function
    def update(frame):
        idx = frame
        robot1_line.set_data(path1[:idx+1, 0], path1[:idx+1, 1])
        robot2_line.set_data(path2[:idx+1, 0], path2[:idx+1, 1])
        return robot1_line, robot2_line

    # Create and show animation
    ani = FuncAnimation(fig, update, frames=frame_indices, interval=100, blit=True)
    plt.show()


    
    # Wrap in DictObs




def gen_dataset(size_data):
    folder = "/home/mker/ubot_RL/Multi-Layer-Perceptron-Experiments/RL_approach/imitation_data"
    fname_ls = os.listdir(folder)
    
    traj = []
    for i in range(size_data):
        fname = fname_ls[i]
    

        with open(os.path.join(folder, fname), "r") as f:
            data = json.load(f)
        states_trj, acts_trj = load_data(data, plot = False, ds=3)
        traj.append(states_trj)

    print(len(traj))
    # save the trajectory data
    np.save('traj.npy', np.array(traj, dtype=object))

if __name__ == "__main__":
    gen_dataset(size_data = 400)
    # load_data(data, dt = 0.1, plot = True)
    # data = np.load('traj.npy', allow_pickle=True)
    # print(data[0])
    # plot_raw_traj(20)
    pass