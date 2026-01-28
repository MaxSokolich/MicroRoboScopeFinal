import numpy as np
import matplotlib.pyplot as plt
import itertools

import gurobipy as gp
from gurobipy import GRB
import sys
import json
import scipy.io as sio
import os

def greedy_min_deviation_ordering(velocities, durations):
    N = len(velocities)
    d = len(velocities[0])

    # Compute the total displacement vector
    s_list = [v * t for v, t in zip(velocities, durations)]
    total_disp = sum(s_list)

    # Normalize direction of total displacement
    total_dir = total_disp / np.linalg.norm(total_disp)

    # Build orthogonal projection matrix to total direction
    P = np.eye(d) - np.outer(total_dir, total_dir)

    remaining = list(range(N))
    ordering = []
    current_dev = np.zeros(d)

    for _ in range(N):
        best_idx = None
        best_dev = None

        for i in remaining:
            step = s_list[i]
            step_perp = P @ step
            new_dev = current_dev + step_perp
            norm_dev = np.linalg.norm(new_dev)

            if best_dev is None or norm_dev < best_dev:
                best_dev = norm_dev
                best_idx = i

        ordering.append(best_idx)
        current_dev += P @ s_list[best_idx]
        remaining.remove(best_idx)

    return ordering


def find_max_scale_to_fit_box(polygon, box_bounds = [(-90,90),(-90,90),(-90,90),(-90,90)]):
    """
    Parameters:
    - polygon: List of N-dimensional points (tuples or lists).
    - fixed_corner: A single N-dimensional point.
    - box_bounds: A list of (min_i, max_i) tuples for each dimension.
    
    Returns:
    - max_scale: The largest possible scale factor (<= 1.0).
    """


    polygon = np.array(polygon)
    fixed_corner = polygon[0]
    fixed_corner = np.array(fixed_corner)
    dims = len(fixed_corner)
    max_scale = 1.0

    for point in polygon:
        if np.allclose(point, fixed_corner):
            continue  # Skip the fixed point

        direction = point - fixed_corner
        for i in range(dims):
            if direction[i] == 0:
                continue

            s_min = (box_bounds[i][0] - fixed_corner[i]) / direction[i]
            s_max = (box_bounds[i][1] - fixed_corner[i]) / direction[i]

            # Keep valid s within (0,1]
            valid_scales = []
            if direction[i] > 0:
                if 0 < s_max <= 1:
                    valid_scales.append(s_max)
            else:
                if 0 < s_min <= 1:
                    valid_scales.append(s_min)

            if valid_scales:
                max_scale = min(max_scale, min(valid_scales))

    return max_scale

def compute_min_n_by_polygon_shrinkage(velocities, durations, x0, box_lower, box_upper):
    N = len(velocities)
    d = len(x0)

    # Step 1: Compute the full polygonal path
    p = [x0.copy()]
    for i in range(N):
        p.append(p[-1] + velocities[i] * durations[i])

    S = p[-1] - x0  # total displacement
    shape_vectors = [pi - x0 for pi in p]  # relative polygon shape

    # Step 2: For each dimension, find how much room we have along the segment
    tightest_n = 0
    for j in range(d):
        # Compute min margin along the segment in dim j
        x_j_min = min(x0[j], x0[j] + S[j])
        x_j_max = max(x0[j], x0[j] + S[j])
        margin_j = min(box_upper[j] - x_j_max, x_j_min - box_lower[j])

        if margin_j <= 0:
            return None  # even the line doesn't fit

        # Max absolute deviation of polygon in this dimension
        max_dev_j = max(abs(s[j]) for s in shape_vectors)

        # Required n in this dimension
        n_j = max_dev_j / margin_j
        tightest_n = max(tightest_n, n_j)

    return int(np.ceil(tightest_n))

def compute_exact_min_n(velocities, durations, x0, box_lower, box_upper):
    """
    velocities: list of np.arrays of shape (d,)
    durations: list of scalars of same length
    x0: np.array of shape (d,)
    box_lower: np.array of shape (d,)
    box_upper: np.array of shape (d,)
    """

    N = len(velocities)
    d = len(x0)

    # Step 1: Compute displacement vectors and total displacement
    s = [v * t for v, t in zip(velocities, durations)]
    S = sum(s)  # total displacement

    # Step 2: Build the deviation set Gamma
    gamma = []
    p = np.zeros(d)
    for k in range(N + 1):
        alpha = k / N
        target = alpha * S
        deviation = p - target
        gamma.append(deviation.copy())
        if k < N:
            p += s[k]

    # Step 3: For each gamma and each dimension, compute required n
    max_n = 0
    for g in gamma:
        for j in range(d):
            # Consider the path along the line x0 + alpha * S
            x_line_min = min(x0[j], x0[j] + S[j])
            x_line_max = max(x0[j], x0[j] + S[j])

            margin_upper = box_upper[j] - x_line_max
            margin_lower = x_line_min - box_lower[j]

            if g[j] > 0:
                if margin_upper <= 0:
                    return None  # violates box no matter what
                n_j = g[j] / margin_upper
            elif g[j] < 0:
                if margin_lower <= 0:
                    return None
                n_j = -g[j] / margin_lower
            else:
                n_j = 0

            max_n = max(max_n, n_j)


    
    return int(np.ceil(max_n))
def find_max_scale_to_fit_box(polygon, box_bounds = [(-90,90),(-90,90),(-90,90),(-90,90)]):
    """
    Parameters:
    - polygon: List of N-dimensional points (tuples or lists).
    - fixed_corner: A single N-dimensional point.
    - box_bounds: A list of (min_i, max_i) tuples for each dimension.
    
    Returns:
    - max_scale: The largest possible scale factor (<= 1.0).
    """


    polygon = np.array(polygon)
    fixed_corner = polygon[0]
    fixed_corner = np.array(fixed_corner)
    dims = len(fixed_corner)
    max_scale = 1.0

    for point in polygon:
        if np.allclose(point, fixed_corner):
            continue  # Skip the fixed point

        direction = point - fixed_corner
        for i in range(dims):
            if direction[i] == 0:
                continue

            s_min = (box_bounds[i][0] - fixed_corner[i]) / direction[i]
            s_max = (box_bounds[i][1] - fixed_corner[i]) / direction[i]

            # Keep valid s within (0,1]
            valid_scales = []
            if direction[i] > 0:
                if 0 < s_max <= 1:
                    valid_scales.append(s_max)
            else:
                if 0 < s_min <= 1:
                    valid_scales.append(s_min)

            if valid_scales:
                max_scale = min(max_scale, min(valid_scales))

    return max_scale

class Chop:
    def __init__(self, velocities,durations, x0):
        """
        polygon: list of points in R^d, where each point is a list or np.array of length d
        """
        # self.polygon = [np.array(p) for p in polygon]
        self.velocities = velocities
        self.durations = durations
        self.x0 = x0
        self.ordering = greedy_min_deviation_ordering(velocities, durations)
        self.velocities = [velocities[i] for i in self.ordering]
        self.durations = [durations[i] for i in self.ordering]

    def get_chopped_trajectory(self, n , ds):
        """
        n: number of chopped points
        make sure that durations are less than dt
        """

        # n = self.get_chop_step()
        # chopped_points = [self.x0]
        vel_applied = []
        time_ls = []
        for j in range(n):
            for i in range(len(self.velocities)):
                vel_applied.append(self.velocities[i])
                time_ls.append(self.durations[i]/n)
                # chopped_points.append(chopped_points[-1] + self.velocities[i]*self.durations[i]*((1)/n))
        # return np.array(chopped_points)
        return self.fixed_distance_chop(vel_applied, time_ls, ds)


    def fixed_timeperiod(self, velocities, durations, dt):
   
        """
        Chop trajectory into discrete steps of duration dt.
        Returns trajectory points at every dt along the original path.
        
        dt: fixed time step
        """
        chopped_points = [self.x0]
        current_point = self.x0
        commands_applied = []
        for vel, duration in zip(velocities, durations):
            t = 0.0
            while t + dt < duration:
                current_point = current_point + vel * dt
                chopped_points.append(current_point)
                t += dt
                commands_applied.append(vel)
            # Add the last leftover piece if any
            remaining_time = duration - t
            if remaining_time > 1e-4:  # tolerance to avoid floating point issues
                current_point = current_point + vel * remaining_time
                chopped_points.append(current_point)
                commands_applied.append(vel)

        return np.array(chopped_points), np.array(commands_applied)
    
    def fixed_distance_chop(self, velocities, durations, ds):
        """
        Chop trajectory into points that are equally spaced in distance (ds).
        
        velocities: list of velocity vectors (np.array)
        durations: list of durations for each velocity
        ds: fixed distance step size
        """
        chopped_points = [self.x0.copy()]
        current_point = self.x0.copy()
        commands_applied = []
        
        for vel, duration in zip(velocities, durations):
            total_disp = vel * duration
            segment_length = np.linalg.norm(total_disp)
            if segment_length < 1e-6:
                continue  # skip near-zero movement
            
            direction = total_disp / segment_length
            num_segments = int(np.floor(segment_length / ds))
            
            for _ in range(num_segments):
                current_point = current_point + direction * ds
                chopped_points.append(current_point.copy())
                commands_applied.append(vel)
            
            # Add the remainder if needed
            remaining_length = segment_length - num_segments * ds
            if remaining_length > 1e-4:
                current_point = current_point + direction * remaining_length
                chopped_points.append(current_point.copy())
                commands_applied.append(vel)
        
        return np.array(chopped_points), np.array(commands_applied)


    
    def visualize(self):
        polygon_np = np.array(self.polygon)
        farthest_point, dist = self.find_farthest_corner()

        plt.figure(figsize=(8, 6))
        plt.plot(polygon_np[:, 0], polygon_np[:, 1], 'bo-', label='Polygon')
        plt.plot(
            [self.polygon[0][0], self.polygon[-1][0]],
            [self.polygon[0][1], self.polygon[-1][1]],
            'r--', label='Line (First to Last Point)'
        )
        plt.plot(farthest_point[0], farthest_point[1], 'ro', markersize=10, label='Farthest Corner')
        plt.title("Farthest Corner from Line Segment (First to Last Point)")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()

   

    def get_chop_step(self, box_min = -1*np.array([420,350,420,350]), box_max = np.array([420,350,420, 350])):
        ## find the max scale to fit the box
        # print("Finding max scale to fit boxxx")
        print("save for mehdi: self.velocities = {}, self.durations = {}, self.x0 = {}".format(self.velocities, self.durations, self.x0))
        """n = compute_min_n_by_polygon_shrinkage(self.velocities, self.durations, self.x0, box_min, box_max)
        print(n)
        if n is not None:
            return n
        else:
            n = int(1/find_max_scale_to_fit_box( self.get_chopped_trajectory(1,2)))+1
            while True:
                trj = self.get_chopped_trajectory(n)
                print(trj)
                outbound = np.max(np.abs(trj))
                if (outbound < box_max).all() and (outbound > box_min).all():
                    break
                else:
                    n = int(n*np.max(outbound/box_max))+1
                    print(n)"""
        return 1









def find_max_scale_to_fit_box(polygon, box_bounds = [(-90,90),(-90,90),(-90,90),(-90,90)]):
    """
    Parameters:
    - polygon: List of N-dimensional points (tuples or lists).
    - fixed_corner: A single N-dimensional point.
    - box_bounds: A list of (min_i, max_i) tuples for each dimension.
    
    Returns:
    - max_scale: The largest possible scale factor (<= 1.0).
    """


    polygon = np.array(polygon)
    fixed_corner = polygon[0]
    fixed_corner = np.array(fixed_corner)
    dims = len(fixed_corner)
    max_scale = 1.0

    for point in polygon:
        if np.allclose(point, fixed_corner):
            continue  # Skip the fixed point

        direction = point - fixed_corner
        for i in range(dims):
            if direction[i] == 0:
                continue

            s_min = (box_bounds[i][0] - fixed_corner[i]) / direction[i]
            s_max = (box_bounds[i][1] - fixed_corner[i]) / direction[i]

            # Keep valid s within (0,1]
            valid_scales = []
            if direction[i] > 0:
                if 0 < s_max <= 1:
                    valid_scales.append(s_max)
            else:
                if 0 < s_min <= 1:
                    valid_scales.append(s_min)

            if valid_scales:
                max_scale = min(max_scale, min(valid_scales))

    return max_scale


def get_all_commands():
    
    alphas = np.arange(0, 2 * np.pi, np.pi /2)
    frq_map = []
    vs = [[
        2.72, 4.06, 5.80, 6.81, 9.07, 9.46, 11.32, 11.95, 14.11, 14.49, 16.15,
        16.49, 17.30, 17.09, 18.35, 19.68, 19.45, 21.39, 22.50, 23.65
        ],
        [16.62, 27.30, 37.71, 48.13, 58.72, 66.67, 78.15, 84.48,
            96.43, 108.05, 119.22, 120.53, 127.00, 133.90, 131.50,
            151.17, 153.06, 161.49, 170.00, 170.95
    ]]
    vs_sampled = vs
    nsize = len(vs_sampled[0])
    all_commands = []
    for i in range(len(vs[0])):
        speed1 = vs[0][i]
        speed2 = vs[1][i]
        for ia in range(len(alphas)):
            v = np.array([
                speed1 * np.cos(alphas[ia]), speed1 * np.sin(alphas[ia]),
                speed2 * np.cos(alphas[ia]), speed2 * np.sin(alphas[ia])
            ])
            all_commands.append(v)
            frq_map.append([i])

    all_commands = np.array(all_commands)
    return all_commands,frq_map





def LP(initial_configuration, final_configuration, all_commands, frq_map, plot = False):
   
    size_T = len(all_commands)
    m = gp.Model()
    T = m.addMVar(size_T, lb = 0, name= 'Time periods')
   
    b = m.addVar(lb =0, ub= 10**8)
    bounds = m.addMVar(4, lb=0, name="bounds")  # Bounds for constraints
    for i in range(4):
        m.addConstr(bounds[i] == b) 
    max_abs_goal = 0.1
    # error_margin = m.addMVar(4,ub = max_abs_goal, lb = -max_abs_goal, name= 'error_margin')


    # abs_error = m.addMVar(4, lb=0, name="abs_error")

    # Add constraints to represent absolute value
    # for i in range(4):
    #     m.addConstr(abs_error[i] >= error_margin[i])   # abs_error >= error_margin
    #     m.addConstr(abs_error[i] >= -error_margin[i])  # abs_error >= -error_margin


    m.addConstr(initial_configuration+all_commands.T@T == final_configuration)

    # for i in range(1,size_T):
    #     m.addConstr(initial_configuration+all_commands.T[:,:i]@T[:i] <= bounds)
    #     m.addConstr(-bounds<=initial_configuration+all_commands.T[:,:i]@T[:i] )
    cost = 0
    cost += gp.quicksum(np.linalg.norm(all_commands[i])*T[i] for i in range(size_T))
    # cost = 10**2*b
    cost += gp.quicksum(T)
    # cost += gp.quicksum(1*abs_error)
    # for i, v in enumerate(all_commands):
    #     m.addConstr(np.linalg.norm(v)*T[i] <= 10000)
    
    m.update()
    m.setObjective(cost, sense=gp.GRB.MINIMIZE)



    m.update()
    # m.params.NonConvex = 2
    m.optimize()
    if m.Status == gp.GRB.OPTIMAL:
        # print(T.X)
        # print('abs error',error_margin.X )
        print(f"Optimal solution found in {m.Runtime:.5f} seconds.")

        T_values = T.X  # This is your solution for time periods
        cost = m.objVal
        print('cost', cost)


        all_commands = np.array(all_commands)
        frq_map = np.array(frq_map)
        non_zero_T = np.where(T_values > 0)[0]
        print('num_steps', len([non_zero_T[0]]))
     

       
    else:
        print("No solution found.")
        T_values = [[]]
        all_commands = [[]]
        non_zero_T = 0


    return all_commands[non_zero_T], T_values[non_zero_T]

def discrete_path(positions, frq_ls, alpha_ls, dx = 4):
    path = []
    applied_frq = []
    applied_alpha = []
    for i in range(len(positions)-1):
        p1 = positions[i]
        p2 = positions[i+1]
        v = p2 - p1
        n = int(np.linalg.norm(v)/dx)
        v = v/n
        for j in range(n):
            applied_frq.append(frq_ls[i])
            applied_alpha.append(alpha_ls[i])
            path.append(p1 + j*v)
    
    return np.array(path), np.array(applied_frq), np.array(applied_alpha)


# final_configuration = np.array([10 ,-10.1 , 40.1 , 35.1])
# counter0 = 0
   
# initial_configuration = np.array([1.1, 1.1 , -10.1 , -1.1])


# all_commands, frq_map = get_all_commands()
# all_commands, T_values = LP(initial_configuration, final_configuration, all_commands, frq_map, plot=True)





def generate_reference_path(initial_configuration, final_configuration, plot = True):
    #initial_configuration = np.array([1.1, 1.1 , -10.1 , -1.1])
    #final_configuration = np.array([10 ,-10.1 , 40.1 , 35.1])
    all_commands, frq_map = get_all_commands()
    ###solving LP
    u_seq, T_values = LP(initial_configuration, final_configuration, all_commands, frq_map, plot=True)
   
    u_seq = np.array(u_seq)
    T_values = np.array(T_values) 

    
    chop_c = Chop(u_seq, T_values, initial_configuration)
    chop_step = chop_c.get_chop_step()
    ### Get the final trajectory
    path, actions_chopped = chop_c.get_chopped_trajectory(chop_step, ds = 2)
    obs_ls = []
    goal = np.array(path[-1])
    N =2
    for i in range(len(path)):
        agents_array = np.asarray(path[i]).flatten()
     
        obs_ls.append(agents_array)
     
    path1 = path[:,0:2]
    path2 = path[:,2:4]
  

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
        plt.show()
        plt.savefig('test.png')
        plt.close()


    
    # Wrap in DictObs

    obs_ls = np.array(obs_ls)

    return path, actions_chopped





