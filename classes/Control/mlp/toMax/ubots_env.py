import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from copy import deepcopy

import gymnasium as gym
from gymnasium.spaces import Box, Discrete

import torch
import sys
# sys.path.append('~/imitation_Learning_ubots')
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
import json
import os
from collections import deque
#from chop import Chop

def load_data(data, dt = 0.1, plot = False):
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
    path, actions_chopped = chop_c.get_chopped_trajectory(chop_step, dt = dt)
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





class uBotsGym(gym.Env):
    """
    Class for creating uBots Gym(nasium) environment.
    Can be trained with Stable-Baselines3.
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}
    
    LOOKUP_TABLE = [[
                2.72, 4.06, 5.80, 6.81, 9.07, 9.46, 11.32, 11.95, 14.11, 14.49,
                16.15, 16.49, 17.30, 17.09, 18.35, 19.68, 19.45, 21.39, 22.50,
                23.65
            ],
            [
                16.62, 27.30, 37.71, 48.13, 58.72, 66.67, 78.15,
                84.48, 96.43, 108.05, 119.22, 120.53, 127.00,
                133.90, 131.50, 151.17, 153.06, 161.49, 170.00,
                170.95
            ]]

    def __init__(self,
                 N, # number of uBots
                 XMIN=-10, # min x-coord
                 XMAX=10, # max x-coord
                 YMIN=-10, # min y-coord
                 YMAX=10, # max y-coord
                 dt=0.1, # sampling time
                 horizon=100, # task/episode horizon
                 continuous_task=True, # whether to terminate after reaching goal or time elapsed
                 render_mode=None,
                 evaltraj=False):
        self.N = N
        self.XMIN = XMIN
        self.XMAX = XMAX
        self.YMIN = YMIN
        self.YMAX = YMAX
        self.dt = dt
        self.evaltraj = evaltraj
        print(f"Evaltraj: {self.evaltraj}")
        self.horizon = horizon
        self.continuous_task = continuous_task
        self.render_mode = render_mode
        # self.all_commands = np.load("all_commands_sparse.npy")
        # Set observation and action spaces
        self.observation_space = Box(
            low=np.array([[XMIN, YMIN], [XMIN, YMIN]]),  # Lower bounds for (x, y) of each robot
            high=np.array([[XMAX, YMAX], [XMAX, YMAX]]),  # Upper bounds for (x, y) of each robot
            shape=(N, 2),
            dtype=np.float32)


        obs_dim = 2 * self.N  # 2 bots (x,y) + 2 goals (x,y)
        self.folder_path="demo_frames"

        self.action_space = Box(low=np.array([0, -np.pi]),
                                high=np.array([24, np.pi]))
        
        self.max_steps_per_waypoint = 5  # max number of steps to reach each waypoint
        # self.box_size = 50.0  # Size of bounding box
        # self.dir_trajs = "/home/mker/ubot_RL/Multi-Layer-Perceptron-Experiments/RL_approach/imitation_data"
        # self.fname_ls = os.listdir(self.dir_trajs)
        self.trajs = np.load('traj.npy', allow_pickle=True)
        # Create matplotlib figure if rendering
        if render_mode == "human":
            self.fig, self.ax = plt.subplots()
    
    def clear_path(self):
        """
        clears all the frames in the folder
        """
        folder_path = self.folder_path
        if os.path.exists(folder_path):
            for filename in os.listdir(folder_path):
                file_path = os.path.join(folder_path, filename)
                try:
                    if os.path.isfile(file_path):
                        os.remove(file_path)
                except Exception as e:
                    print(f"Error deleting {file_path}: {e}")
        else:
            print(f"{folder_path} does not exist. No files to delete.")

    def reset(self, seed=None,  options=None):
        # Set random seed
    
        self.observation_space.seed(seed)

        # Generate goal location at start of every episode
        

        self._steps_elapsed = 0 # for checking horizon

        self.rob0_togo_prev = None
        self.rob1_togo_prev = None
        self.clear_path()
        # create initial robot locations
       
        
       
       
        if self.evaltraj == False:
            idx = np.random.randint(0, len(self.trajs))
        
            id_weighpoint = np.random.randint(0, len(self.trajs[idx]))
        
            self.current_goal = self.trajs[idx][id_weighpoint]
            # print(f'idx: {idx}, id_weighpoint: {id_weighpoint}')
            self.goal0_pos = self.current_goal[0:2]
            self.goal1_pos = self.current_goal[2:4]
            self.current_goal = np.array([self.current_goal[0:2], self.current_goal[2:4]])
            
            
            # obs = deepcopy(self.positions)
            self.positions = np.array([self.trajs[idx][id_weighpoint-1][0:2], self.trajs[idx][id_weighpoint-1][2:4]])
            self.box_size = np.linalg.norm(self.current_goal.flatten() - self.positions.flatten())+10
        
        # self.positions = self.rel_obs()

        # obs = {"agents": deepcopy(self.positions), "goals": self._get_goal()}

        info = {'horizon': self.horizon, 'is_success': False}

        if self.render_mode == "human":
            # setup the display/render
            self.ax.cla()
            # self.fig, self.ax = plt.subplots()
            self.ax.set_xlim(self.XMIN, self.XMAX)
            self.ax.set_ylim(self.YMIN, self.YMAX)

            # show the goal positions
            self.scat = self.ax.scatter(self.goal0_pos[0],
                                        self.goal0_pos[1],
                                        c='r')
            self.scat = self.ax.scatter(self.goal1_pos[0],
                                        self.goal1_pos[1],
                                        c='g')

            # show the robot positions
            
            positions = np.vstack(self.positions+self.current_goal)
        
            self.scat = self.ax.scatter(positions[:, 0],
                                        positions[:, 1],
                                        c='b')

        return self.rel_obs(), info
    


    def check_in_bounds(self):
        #### check if the robots are within the bounds with some margin
        margin = 0.5
        out = np.all(self.positions[:, 0] > self.XMIN + margin) and np.all(self.positions[:, 0] < self.XMAX - margin) and np.all(self.positions[:, 1] > self.YMIN + margin) and np.all(self.positions[:, 1] < self.YMAX - margin)
        return out
    

    # def step(self, action):
    #     f, alpha = action
    #     new_positions = []
    #     speeds = self.v_i(f)
    #     for i, pos in enumerate(self.positions):
    #         dx = speeds[i] * self.dt * np.cos(alpha)
    #         dy = speeds[i] * self.dt * np.sin(alpha)
    #         new_pos = pos + np.array([dx, dy])
    #         new_pos[0] = np.clip(new_pos[0], self.XMIN, self.XMAX)
    #         new_pos[1] = np.clip(new_pos[1], self.YMIN, self.YMAX)
    #         new_positions.append(new_pos)
    #     self.positions = np.array(new_positions)

    #     self._steps_elapsed += 1

    #     # obs = deepcopy(self.positions)
    #     obs = self._flatten_obs()

    #     # Get reward and number of robots successfully reached their goals
    #     if not self.check_in_bounds():
    #         reward = -90.0
    #         terminated = True
    #         truncated = True if (self._steps_elapsed >= self.horizon) else False
    #         info = {'is_success': False, 'n_successes': 0}
    #         successes = 0
    #     else:
    #         #Get reward and number of robots successfully reached their goals
    #         reward, successes = self._get_reward(obs)
    #     # print(reward)
    #     # if self.continuous_task:
    #     #     terminated = False
    #     # else:
    #     terminated = successes >= 2
        
    #     info = {'is_success': successes >= 2, 'n_successes': successes}
        
    #     truncated = True if (self._steps_elapsed >= self.horizon) else False

    #     return self._flatten_obs(), reward, terminated, truncated, info


    def step(self, action, add_noise=True, noise_magnitude=0.1):
        f, alpha = action
        new_positions = []
        speeds = self.v_i(f)
        for i, pos in enumerate(self.positions):
            if add_noise:
                noise = np.random.normal(0, noise_magnitude, size=pos.shape)
                speed = speeds[i]+noise[0]
            else:
                speed = speeds[i]   
            dx = speed * self.dt * np.cos(alpha)
            dy = speed * self.dt * np.sin(alpha)
            new_pos = pos + np.array([dx, dy])
            new_positions.append(new_pos)
        self.positions = np.array(new_positions)

        self._steps_elapsed += 1
        obs = self.rel_obs()
    
        dist_to_goal = np.linalg.norm(obs)

        # Reward: encourage moving closer to goal
        reward = -dist_to_goal

        # Termination conditions
        terminated = False
        truncated = False
       
        
        if self.evaltraj== False:
            if np.any(np.abs(obs) > self.box_size):
                reward -= 50.0  # penalty for leaving bounding box
                terminated = True
            if dist_to_goal < 4:
                reward += 50.0  # bonus for reaching next waypoint
                terminated = True

        if self._steps_elapsed >= self.horizon:
            truncated = True
        # print(f"dist_to_goal: {dist_to_goal}, reward: {reward}, terminated: {terminated}, truncated: {truncated}")
        
        return obs, reward, terminated, truncated, {}

    def render(self, save_frames=True, show=False):
        self.ax.cla()
        self.ax.set_xlim(self.XMIN, self.XMAX)
        self.ax.set_ylim(self.YMIN, self.YMAX)

        # Plot reference trajectory if it exists
        if hasattr(self, 'reference_traj'):
            traj = np.array(self.reference_traj)
            path1 = traj[:, :2]
            path2 = traj[:, 2:]
            self.ax.plot(path1[:, 0], path1[:, 1], 'o--', color='blue', alpha=0.3, label='Trajectory R1')
            self.ax.plot(path2[:, 0], path2[:, 1], 'o--', color='red', alpha=0.3, label='Trajectory R2')

            if hasattr(self, 'current_wp_idx') and self.current_wp_idx < len(traj):
                wp = traj[self.current_wp_idx]
                self.ax.scatter(wp[0], wp[1], marker="v", color='purple', s=50, label='WP R1')
                self.ax.scatter(wp[2], wp[3], marker="v", color='purple', s=50, label='WP R2')

        if hasattr(self, 'final_goal'):
            self.ax.scatter(self.final_goal[0], self.final_goal[1], marker='*', color='purple', s=50, label='Final Goal R1')
            self.ax.scatter(self.final_goal[2], self.final_goal[3], marker='*', color='purple', s=50, label='Final Goal R2')

        self.ax.scatter(self.positions[:, 0], self.positions[:, 1], s=90, c='black', label='Robots')
        self.ax.legend()
        if show:
            plt.pause(0.1)  # Only show if explicitly asked


        if save_frames:
            os.makedirs(self.folder_path, exist_ok=True)
            self.fig.canvas.draw()  # Draw the figure to the canvas
            self.fig.savefig(f"{self.folder_path}/frame_{self.frame_idx:04d}.png")
            self.frame_idx += 1
            print(f"Saved frame {self.frame_idx} to {self.folder_path}/frame_{self.frame_idx:04d}.png")

        
    # def render(self):
    #     self.scat.set_offsets(self.positions+self.current_goal)
    #     plt.show(block=False)
    #     # Necessary to view frames before they are unrendered
    #     plt.pause(0.1)

    def save_render(self, filename,  folder_path='demo_frames'):
        self.scat.set_offsets(self.positions)
       
        # Necessary to view frames before they are unrendered
        save_path = f"{folder_path}/{filename}.png"
        plt.savefig(save_path)
        print(f"Saved render to {save_path}")
    

    def clean_folder(self, folder_path='demo_frames'):
        import os
        """
        Deletes all files in the given folder.
        """
    
        if not os.path.isdir(folder_path):
            raise ValueError(f"{folder_path} is not a valid directory.")

        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            try:
                if os.path.isfile(file_path):
                    os.remove(file_path)
            except Exception as e:
                print(f"Error deleting {file_path}: {e}")
   
   
    def close(self):
        plt.close()

    

    def min_distance_to_boundary(self):
        min_distance = np.inf
        for pos in self.positions:
            min_distance = min(min_distance, pos[0] - self.XMIN, self.XMAX - pos[0], pos[1] - self.YMIN, self.YMAX - pos[1])
        return min_distance

    def _get_reward(self, obs, eps=4.0):
        
        obs = obs["agents"]
        rob0_pos = obs[0]
        rob1_pos = obs[1]

        # Calculate dist(robot, goal) for each robot
        d0 = np.linalg.norm(rob0_pos - self.goal0_pos)
        d1 = np.linalg.norm(rob1_pos - self.goal1_pos)

        # Check goal-reach condition
        successes = sum(np.array([d0, d1]) <= eps)

        ### Option 1: Single reward function
        # reward = -10.0 * (d0 + d1) + successes
        # reward = 10.0 * (np.exp(-d0) + np.exp(-d1))
        # reward = -1.0 * (np.exp(d0) + np.exp(d1))
        # reward = (1.0 - np.tanh(d0)) + (1.0 - np.tanh(d1))

        ### Option 2: Reward function decomposition (rob0 = Robot 0, rob1 = Robot 1)
        # Distance to goal
        rob0_dist = d0
        rob1_dist = d1
        # OR: Inverse distance to goal (USE ONE ONLY!)
        # small epsilon distance added to numerical stability (divide-by-zero issues)
        # rob0_dist = 1 / (d0 + 1e-4)**2
        # rob1_dist = 1 / (d1 + 1e-4)**2

        # Size of step towards goal (energy cost)
        if self.rob0_togo_prev is None:
            rob0_progress = 0
        else:
            rob0_progress = (self.rob0_togo_prev - d0)
            self.rob0_togo_prev = d0

        if self.rob1_togo_prev is None:
            rob1_progress = 0
        else:
            rob1_progress = (self.rob1_togo_prev - d1)
            self.rob1_togo_prev = d1

        # Check if the robots overshoot their goals
        if (self.rob0_togo_prev or self.rob1_togo_prev) is None:
            all_overshoots = 0
        else:
            rob0_goal_overshoot = 1 if d0 > self.rob0_togo_prev else 0
            rob1_goal_overshoot = 1 if d0 > self.rob0_togo_prev else 0
            all_overshoots = rob0_goal_overshoot + rob1_goal_overshoot

        # Robots should reach their goals at approximately same times
        synchronous_reaching = abs(d0 - d1)

        # Penalty for touching/exiting the boundaries
        rob0_out_of_bounds = (rob0_pos[0] >= self.XMAX) or (rob0_pos[0] <= self.XMIN) or (rob0_pos[1] >= self.YMAX) or (rob0_pos[1] >= self.YMIN)
        rob1_out_of_bounds = (rob1_pos[0] >= self.XMAX) or (rob1_pos[0] <= self.XMIN) or (rob1_pos[1] >= self.YMAX) or (rob1_pos[1] >= self.YMIN)
        rew_out_of_bounds = rob0_out_of_bounds or rob1_out_of_bounds

        # Compose the final reward
        alpha = -1
        beta = -1
        gamma = 1
        delta = 1
        lambda_ = -5
        mu = -1
        sigma = -100
        reward = (alpha * rob0_dist) + \
                 (beta * rob1_dist) + \
                 (gamma * rob0_progress) + \
                 (delta * rob1_progress) + \
                 (lambda_ * all_overshoots) + \
                 (mu * synchronous_reaching) + \
                 (sigma * rew_out_of_bounds)

        return reward, successes


        
        # minimum_distance_to_boundary = self.min_distance_to_boundary()
        # penalty_to_boundry = -2*np.exp(-0.1*minimum_distance_to_boundary)

        # end_goal = successes.all()
        # state = np.array([rob0_pos, rob1_pos]).flatten()
        # cost_to_go = 1

        # reward = -0.01*cost_to_go+penalty_to_boundry
        # # print(reward)

        # return reward, successes
    
    # def _get_goal(self):
        
            
    #         # print(f'obs shape: {obs.shape}')

    #         # return goal0, goal1
    
    def _get_init_robot_pos(self):
        # Random goal
        rob0_pos, rob1_pos = np.random.uniform([self.XMIN, self.YMIN], [self.XMAX, self.YMAX], (self.N, 2))
        
        # # Fixed positions
        # rob0_pos = np.array([  9.183674, -13.265306])
        # rob1_pos = np.array([-5.102041,  39.795918])


        return rob0_pos, rob1_pos

    def v_i(self, f):
        if self.N > 2:
            print("Warning: Number of bots is greater than 2. Replicating the lookup table for the first 2 bots.")
            self.LOOKUP_TABLE = self.LOOKUP_TABLE * (self.N // 2 + 1)
        return np.array([np.interp(f, range(1, 21), self.LOOKUP_TABLE[i]) for i in range(self.N)])
    
    def __str__(self):
        print("Observation space: ", self.observation_space)
        print("Action space: ", self.action_space)
        return ""
    
    def rel_obs(self):
        relative_positions = self.positions- self.current_goal 
        return relative_positions.astype(np.float32)