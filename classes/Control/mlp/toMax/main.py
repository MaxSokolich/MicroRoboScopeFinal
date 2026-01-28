from pathlib import Path
import argparse

import numpy as np

from ubots_env import *

from stable_baselines3 import PPO, SAC, DQN,  HerReplayBuffer
from stable_baselines3.common.logger import configure
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, CallbackList, CheckpointCallback
import os

def run_one_episode():
    env = uBotsGym(N=2)  #, render_mode="human")
    print("Observation space: ", env.observation_space)
    print("Action space: ", env.action_space)
    obs, info = env.reset()
    for i in range(100):
        # action = env.action_space.sample()
        action = (0.1, np.pi / 4)
        obs, reward, terminated, truncated, info = env.step(action)
        # env.render()
    env.close()

def make_single_env(env_kwargs):

    def _init():
        
        env = uBotsGym(N=2, **env_kwargs)
        # else:
        #     env = uBotsGym(N=2, **env_kwargs)
        return env

    return _init

def train(alg='ppo', env_kwargs=None):
    '''RL training function'''

    # Create environment. Multiple paral`lel/vectorized environments for faster training
    env = make_vec_env(make_single_env(env_kwargs), n_envs=8)

    if alg == 'ppo':
        # PPO: on-policy RL
        policy_kwargs = dict(net_arch=dict(pi=[32, 32], vf=[128, 128]))
        # policy_kwargs = dict(net_arch=[64, 64, 64, 64])
        model = PPO("MlpPolicy", 
                    env, 
                    policy_kwargs=policy_kwargs, 
                    batch_size=256,
                    n_epochs=10,
                    n_steps=2048,
                    verbose=1)

    elif alg == 'sac':
        # off-policy RL
        # policy_kwargs = dict(net_arch=dict(pi=[256, 256], qf=[256, 256]))
        policy_kwargs = dict(net_arch=[64, 64, 64, 64])
        model = SAC(
            "MlpPolicy",
            env,
            policy_kwargs=policy_kwargs,
            # use_sde=True,
            # sde_sample_freq=8,
            learning_rate=0.0003,
            learning_starts=1000,
            batch_size=512,
            tau=0.05,
            gamma=0.95,
            # gradient_steps=1,
            verbose=1,
        )
    

    # log the training params
    logfile = f"logs/{alg}_ubots"
    tb_logger = configure(logfile, ["stdout", "csv", "tensorboard"])
    model.set_logger(tb_logger)



    # Add the callback for logging weights
    weight_log_dir = os.path.join(logfile, "weights")
    # callback = LogWeightsCallback(log_dir=weight_log_dir)
    checkpoint_callback = CheckpointCallback(
                save_freq=1000,
                save_path = weight_log_dir,
                name_prefix="rl_model",
                save_replay_buffer=True,
                save_vecnormalize=True,
                )
    eval_env = uBotsGym(N=2, render_mode="human", **env_kwargs)
    best_model_save_path = os.path.join(weight_log_dir, "best_model")
    eval_callback = EvalCallback(eval_env, best_model_save_path=best_model_save_path,
                             log_path=weight_log_dir, eval_freq=10000,
                             deterministic=True, render=False)
    callback = CallbackList([checkpoint_callback, eval_callback])


    # if alg == 'ppo':
    #     ALG = PPO
    # elif alg == 'sac':
    #     ALG = SAC
    # else:
    #     ALG = DQN
    # dir = 'logs/ppo_ubots_discrete/weights/best_model/best_model.zip'
    # # load trained RL model

    # model = ALG.load(dir, env=env)



    # train the model
    model.learn(5*10**6, progress_bar=True, callback=callback)

    model.save(models_dir / f"{alg}_ubots")
    del model
    env.close()


def evaluate(alg, env_kwargs, n_trials=1):
    '''Evaluate the trained RL model'''

    # create single environment for evaluation
    env = uBotsGym(N=2, render_mode="human", **env_kwargs)
    if alg == 'ppo':
        ALG = PPO
    elif alg == 'sac':
        ALG = SAC
    else:
        ALG = DQN
    # dir = '/home/mker/ubot_RL/Multi-Layer-Perceptron-Experiments/RL_approach/discrete/models/ppo_ubotsdiscrete.zip'
    dir = 'logs/ppo_ubots/weights/best_model/best_model.zip'
    # load trained RL model
    # model = ALG.load(models_dir / f"{alg}_ubots{ENV_TYPE}", env=env)
    model = ALG.load(dir, env=env)

    # run some episodes (trials)
    for trial in range(n_trials):
        obs, info = env.reset()
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            # print(obs)
            done = terminated or truncated
            env.render(show= True)
        # print(f"Trial: {trial}, Success: {info['is_success']}, # Successes = {info['n_successes']}")
    env.close()

import cProfile
import pstats

# def evaluate_on_trajectory(model, env, trajectory, max_steps_per_waypoint=50, render=True):
#     """
#     Evaluate a trained model on a specific trajectory.

#     Args:
#         model: Trained SB3 model.
#         env: Instance of uBotsGym with render_mode='human'.
#         trajectory: (T, 4) numpy array where each row is [x1, y1, x2, y2] for two robots.
#         max_steps_per_waypoint: Max steps allowed to reach each waypoint.
#         render: Whether to render the environment.
#     """
#     assert trajectory.shape[1] == 4, "Each waypoint should have shape (4,): [x1, y1, x2, y2]"
    
#     initial_position = trajectory[0]
#     goal_trajectory = trajectory[1:]

#     # Set initial position and first goal
#     env.positions = np.array([initial_position[:2], initial_position[2:]])
#     env.current_goal = np.array([goal_trajectory[0][:2], goal_trajectory[0][2:]])
   
#     obs = env.rel_obs()
#     env.reference_traj = trajectory  # store full reference trajectory
#     env.current_wp_idx = 0           # initialize waypoint index

#     env.reset()

#     for i, waypoint in enumerate(goal_trajectory):
#         env.current_wp_idx = i + 1
#         env.current_goal = np.array([waypoint[:2], waypoint[2:]])
        
#         # env._steps_elapsed = 0
#         obs = env.rel_obs()

#         for step in range(max_steps_per_waypoint):
#             action, _ = model.predict(obs, deterministic=True)
#             obs, reward, terminated, truncated, info = env.step(action)
#             if render:
#                 env.render()
#             if terminated or truncated:
#                 break  # move to next waypoint early

#     env.close()

def evaluate_on_trajectory(model, env, trajectory, max_steps_per_waypoint=50, render=True):
    """
    Evaluate a trained model on a specific trajectory.

    Args:
        model: Trained SB3 model.
        env: Instance of uBotsGym with render_mode='human'.
        trajectory: (T, 4) numpy array where each row is [x1, y1, x2, y2] for two robots.
        max_steps_per_waypoint: Max steps allowed to reach each waypoint.
        render: Whether to render the environment.
    """
    assert trajectory.shape[1] == 4, "Each waypoint should have shape (4,): [x1, y1, x2, y2]"
    # trajectory = trajectory[10:,:]
    # Save trajectory for rendering
    env.reference_traj = trajectory

    # Set initial position
    initial_position = trajectory[0]
    env.positions = np.array([initial_position[:2], initial_position[2:]])
    env._steps_elapsed = 0
    env.final_goal = trajectory[-1]
    env.frame_idx = 0
    
    # Iterate through each waypoint
    for i in range(1, len(trajectory)):
        env.current_wp_idx = i
        waypoint = trajectory[i]
        
        env.current_goal = np.array([waypoint[:2], waypoint[2:]])
        obs = env.rel_obs()
        env._steps_elapsed = 0


        for _ in range(max_steps_per_waypoint):
            action, _ = model.predict(obs, deterministic=True)


            obs, reward, terminated, truncated, info = env.step(action, add_noise=False)

            
            # obs, reward, terminated, truncated, info = env.step(action)
            if render:
                env.render(show= True)
            if terminated or truncated:
                break

    env.close()


    
if __name__ == '__main__':
    # profiler = cProfile.Profile()
    # profiler.enable()


    parser = argparse.ArgumentParser()
    parser.add_argument("--eval",
                        action="store_true", 
                        default=False, 
                        help="Runs the evaluation of a trained model. Default: False (runs RL training by default)")
    args = parser.parse_args()

    # create directory for saving RL models
    models_dir = Path("models")
    models_dir.mkdir(exist_ok=True)

    # create directory for logs
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)

    # ENV_TYPE = 'discrete' # 'continuous'

    # set environment params
    env_kwargs = dict(XMIN=-420,
                 XMAX=420,
                 YMIN=-350,
                 YMAX=350,
                 horizon=120,
                 evaltraj=False
                )
    
    # run_one_episode(); exit()
    alg = ['ppo', 'sac', 'dqn'][0]
    args.eval = True
    if not args.eval:
        # if training
        train(alg, env_kwargs)
    else:
        ####it works for li[1][0] and selected_traj[:20, :]
        # if evaluating
        eval_traj = True
        if eval_traj:
            # model = PPO.load('logs/ppo_ubots/weights/best_model/best_model.zip')
            model = PPO.load(r'C:\Users\Das_Lab_Admin\Desktop\REPOS\MLP\Multi-Layer-Perceptron-Experiments\toMax\best_model.zip')
            trajs = np.load('traj.npy', allow_pickle=True)
            li = [(i,len(t)) for i, t in enumerate(trajs)]
            ##sort by length
            li.sort(key=lambda x: x[1])

            traj_idx = li[5][0]  # or any index you want
            selected_traj = np.array(trajs[traj_idx])  # shape (T, 4)
            # selected_traj = selected_traj[:20, :]
            # Load environment
            env = uBotsGym(N=2, render_mode="human", horizon=100, XMIN=-420, XMAX=420, YMIN=-350, YMAX=350, evaltraj=True)
            
            # Evaluate
            evaluate_on_trajectory(model, env, selected_traj, max_steps_per_waypoint=30)
            # stats = pstats.Stats(profiler).sort_stats('cumtime')  # Sort by cumulative time
            # stats.print_stats(30)  # Show top 30 slowest function calls
        else:
            evaluate(alg, env_kwargs)