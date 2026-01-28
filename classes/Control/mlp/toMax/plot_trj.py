import numpy as np
import matplotlib.pyplot as plt


trajs = np.load('traj.npy', allow_pickle=True)
# id = np.random.randint(0, len(trajs))
id = 20
path1 = trajs[id][:, 0:2]
path2 = trajs[id][:, 2:4]
initial_configuration = trajs[id][0]
goal = trajs[id][-1]
fig, ax = plt.subplots()
traj = trajs[id]
dif_ls = []
for i in range(len(traj)-1):
    dif = np.linalg.norm(traj[i+1] - traj[i])
    dif_ls.append(dif)
    
print(f'max dif: {max(dif_ls)}')

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
# plt.savefig('test.png')
plt.close()
