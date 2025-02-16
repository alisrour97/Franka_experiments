import pandas as pd
import matplotlib.pyplot as plt
import datetime
import os
import numpy as np
# Specify the path to your CSV file
# csv_file_path = '../Data/log_kin.csv'

traj_name = 'INITw_ki_0'
csv_file_path = '../Data4/log_dyn_' + traj_name + '.csv'


traj_name = 'PI_tmp_1_ki_0'
csv_file_path = '../Data_video/log_dyn_' + traj_name + '.csv'

now = datetime.datetime.now()
save_path ='save/runs/' + traj_name + "_" + now.strftime(
    '%Y-%m-%d')
if not os.path.isdir(save_path):
    os.mkdir(save_path)

# Read the CSV file into a DataFrame
df = pd.read_csv(csv_file_path)

# Extract the data as lists
time_list = df['Time'].tolist()
q_list = df[['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']].values.tolist()
dq_list = df[['dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7']].values.tolist()
t_list = df[['t1', 't2', 't3', 't4', 't5', 't6', 't7']].values.tolist()
ex_list = df[['ex1', 'ex2', 'ex3', 'ex4', 'ex5', 'ex6']].values.tolist()
ev_list = df[['ev1', 'ev2', 'ev3', 'ev4', 'ev5', 'ev6']].values.tolist()

pos_d = df[['xd', 'yd', 'zd']].values
pos = df[['x', 'y', 'z']].values
vel_d = df[['vxd', 'vyd', 'vzd']].values
vel = df[['vx', 'vy', 'vz']].values


# Plotting in 3D
fig = plt.figure(figsize=(12, 6))
# Plot for NL
ax1 = fig.add_subplot(111, projection='3d')
ax1.set_title('3D Tracking')
ax1.plot(pos_d[:, 0], pos_d[:, 1], pos_d[:, 2], 'k--', label='Desired Position', linewidth=2)
ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'r', label='Actual Position', linewidth=2)
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
# Set scale of 0.1m on each axis
ax1.set_xlim([min(pos[:, 0])-0.1, max(pos[:, 0])+0.1])
ax1.set_ylim([min(pos[:, 1])-0.1, max(pos[:, 1])+0.1])
ax1.set_zlim([min(pos[:, 2])-0.1, max(pos[:, 2])+0.1])
ax1.legend()
ax1.view_init(elev=10, azim=70)  # Adjust elevation and azimuth angles as needed
plt.savefig(save_path + "/" + "position_3d.pdf")

# fig, axs = plt.subplots(3, 1, figsize=(9, 5), sharex=True)
#
# axs[0].plot(time_list, pos_d[:, 0], 'k--', linewidth=2)
# axs[0].plot(time_list, pos[:, 0], 'r', linewidth=2)
# axs[0].set_ylabel('x tracking [m]')
#
# axs[1].plot(time_list, pos_d[:, 1], 'k--', linewidth=2)
# axs[1].plot(time_list, pos[:, 1], 'r', linewidth=2)
# axs[1].set_ylabel('y tracking [m]')
#
# axs[2].plot(time_list, pos_d[:, 2], 'k--', linewidth=2)
# axs[2].plot(time_list, pos[:, 2], 'r', linewidth=2)
# axs[2].set_ylabel('z tracking [m]')
# axs[2].set_xlabel('time [s]')
#
# plt.show()
#
#
# fig, axs = plt.subplots(3, 1, figsize=(9, 5), sharex=True)
#
# axs[0].plot(time_list, vel_d[:, 0], 'k--', linewidth=2)
# axs[0].plot(time_list, vel[:, 0], 'r', linewidth=2)
# axs[0].set_ylabel('vx tracking [m]')
#
# axs[1].plot(time_list, vel_d[:, 1], 'k--', linewidth=2)
# axs[1].plot(time_list, vel[:, 1], 'r', linewidth=2)
# axs[1].set_ylabel('vy tracking [m]')
#
# axs[2].plot(time_list, vel_d[:, 2], 'k--', linewidth=2)
# axs[2].plot(time_list, vel[:, 2], 'r', linewidth=2)
# axs[2].set_ylabel('vz tracking [m]')
# axs[2].set_xlabel('time [s]')
#
# plt.show()



# Plotting q values
plt.figure(figsize=(12, 8))
for i in range(len(q_list[0])):
    plt.plot(time_list, [q[i] for q in q_list], label=f'q{i+1}')

plt.xlabel('Time')
plt.ylabel('Values')
plt.title('Plot of q with respect to Time')
plt.legend()
plt.grid(True)

plt.savefig(save_path + "/" + "q.pdf")
# Plotting dq values
plt.figure(figsize=(12, 8))
for i in range(len(dq_list[0])):
    plt.plot(time_list, [dq[i] for dq in dq_list], label=f'dq{i+1}')

plt.xlabel('Time')
plt.ylabel('Values')
plt.title('Plot of dq with respect to Time')
plt.legend()
plt.grid(True)
plt.savefig(save_path + "/" + "dq.pdf")

# Plotting t values
plt.figure(figsize=(12, 8))
for i in range(len(t_list[0])):
    plt.plot(time_list, [t[i] for t in t_list], label=f't{i+1}')

plt.xlabel('Time')
plt.ylabel('Values')
plt.title('Plot of tau command with respect to Time')
plt.legend()
plt.grid(True)
plt.savefig(save_path + "/" + "torques.pdf")
plt.show()



#
# # Plotting ex values
# plt.figure(figsize=(12, 8))
# for i in range(len(ex_list[0])):
#     plt.plot(time_list, [ex[i] for ex in ex_list], label=f'ex{i+1}')
#
# plt.xlabel('Time')
# plt.ylabel('Values')
# plt.title('Plot of ex with respect to Time')
# plt.legend()
# plt.grid(True)
# plt.show()
#
# # Plotting ev values
# plt.figure(figsize=(12, 8))
# for i in range(len(ev_list[0])):
#     plt.plot(time_list, [ev[i] for ev in ev_list], label=f'ev{i+1}')
#
# plt.xlabel('Time')
# plt.ylabel('Values')
# plt.title('Plot of ev with respect to Time')
# plt.legend()
# plt.grid(True)
# plt.show()
