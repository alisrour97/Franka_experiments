import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import datetime
import os

# latex for figures
plt.rcParams['text.latex.preamble'] = r'\usepackage{bm} \usepackage{amsmath} \usepackage{lmodern}'
params = {'text.usetex': True,
          'font.size': 16,
          'font.family': 'lmodern'
          }
plt.rcParams.update(params)

traj_name = "traj4"

now = datetime.datetime.now()
save_path ='save/' + traj_name + "_" + now.strftime(
    '%Y-%m-%d')
if not os.path.isdir(save_path):
    os.mkdir(save_path)

# Name of the csv files
csv_file_path_NL = '../Data/log_dyn_' + traj_name + "_NL.csv"
csv_file_path_L = '../Data/log_dyn_' + traj_name + "_L.csv"

# Read the CSV file into a DataFrame
df_NL = pd.read_csv(csv_file_path_NL)
df_L = pd.read_csv(csv_file_path_L)

# Extract the data as lists
time_list_NL = df_NL['Time'].tolist()
q_list_NL = df_NL[['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']].values.tolist()
dq_list_NL = df_NL[['dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7']].values.tolist()
t_list_NL = df_NL[['t1', 't2', 't3', 't4', 't5', 't6', 't7']].values.tolist()
ex_list_NL = df_NL[['ex1', 'ex2', 'ex3', 'ex4', 'ex5', 'ex6']].values.tolist()
ev_list_NL = df_NL[['ev1', 'ev2', 'ev3', 'ev4', 'ev5', 'ev6']].values.tolist()

pos_d_NL = df_NL[['xd', 'yd', 'zd']].values
pos_NL = df_NL[['x', 'y', 'z']].values
vel_d_NL = df_NL[['vxd', 'vyd', 'vzd']].values
vel_NL = df_NL[['vx', 'vy', 'vz']].values

# Extract the data as lists
time_list_L = df_L['Time'].tolist()
q_list_L = df_L[['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']].values.tolist()
dq_list_L = df_L[['dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7']].values.tolist()
t_list_L = df_L[['t1', 't2', 't3', 't4', 't5', 't6', 't7']].values.tolist()
ex_list_L = df_L[['ex1', 'ex2', 'ex3', 'ex4', 'ex5', 'ex6']].values.tolist()
ev_list_L = df_L[['ev1', 'ev2', 'ev3', 'ev4', 'ev5', 'ev6']].values.tolist()


pos_d_L = df_L[['xd', 'yd', 'zd']].values
pos_L = df_L[['x', 'y', 'z']].values
vel_d_L = df_L[['vxd', 'vyd', 'vzd']].values
vel_L = df_L[['vx', 'vy', 'vz']].values

"""Plot the pos and velocity tracking"""
#
# fig, axs = plt.subplots(3, 2, figsize=(9, 5), sharex=True)
#
# axs[0, 0].set_title("NL")
# axs[0, 0].plot(time_list_NL, pos_d_NL[:, 0], 'k--', linewidth=2)
# axs[0, 0].plot(time_list_NL, pos_NL[:, 0], 'r', linewidth=2)
# axs[0, 0].set_ylabel('x  [m]')
#
# axs[1, 0].plot(time_list_NL, pos_d_NL[:, 1], 'k--', linewidth=2)
# axs[1, 0].plot(time_list_L, pos_NL[:, 1], 'r', linewidth=2)
# axs[1, 0].set_ylabel('y  [m]')
#
# axs[2, 0].plot(time_list_NL, pos_d_NL[:, 2], 'k--', linewidth=2)
# axs[2, 0].plot(time_list_NL, pos_NL[:, 2], 'r', linewidth=2)
# axs[2, 0].set_ylabel('z  [m]')
# axs[2, 0].set_xlabel('time [s]')
#
#
# axs[0, 1].set_title("L")
# axs[0, 1].plot(time_list_L, pos_d_L[:, 0], 'k--', linewidth=2)
# axs[0, 1].plot(time_list_L, pos_L[:, 0], 'r', linewidth=2)
#
# axs[1, 1].plot(time_list_L, pos_d_L[:, 1], 'k--', linewidth=2)
# axs[1, 1].plot(time_list_L, pos_L[:, 1], 'r', linewidth=2)
#
# axs[2, 1].plot(time_list_L, pos_d_L[:, 2], 'k--', linewidth=2)
# axs[2, 1].plot(time_list_L, pos_L[:, 2], 'r', linewidth=2)
# axs[2, 1].set_xlabel('time [s]')
#
# plt.savefig(save_path + "/" + "position_tracking" + '.pdf')
#
#
# fig, axs = plt.subplots(3, 2, figsize=(9, 5), sharex=True)
#
# axs[0, 0].set_title("NL")
# axs[0, 0].plot(time_list_NL, vel_d_NL[:, 0], 'k--', linewidth=2)
# axs[0, 0].plot(time_list_NL, vel_NL[:, 0], 'r', linewidth=2)
# axs[0, 0].set_ylabel('vx  [m]')
#
# axs[1, 0].plot(time_list_NL, vel_d_NL[:, 1], 'k--', linewidth=2)
# axs[1, 0].plot(time_list_NL, vel_NL[:, 1], 'r', linewidth=2)
# axs[1, 0].set_ylabel('vy  [m]')
#
# axs[2, 0].plot(time_list_NL, vel_d_NL[:, 2], 'k--', linewidth=2)
# axs[2, 0].plot(time_list_NL, vel_NL[:, 2], 'r', linewidth=2)
# axs[2, 0].set_ylabel('vz  [m]')
# axs[2, 0].set_xlabel('time [s]')
#
#
# axs[0, 1].set_title("L")
# axs[0, 1].plot(time_list_L, vel_d_L[:, 0], 'k--', linewidth=2)
# axs[0, 1].plot(time_list_L, vel_L[:, 0], 'r', linewidth=2)
#
# axs[1, 1].plot(time_list_L, vel_d_L[:, 1], 'k--', linewidth=2)
# axs[1, 1].plot(time_list_L, vel_L[:, 1], 'r', linewidth=2)
#
# axs[2, 1].plot(time_list_L, vel_d_L[:, 2], 'k--', linewidth=2)
# axs[2, 1].plot(time_list_L, vel_L[:, 2], 'r', linewidth=2)
# axs[2, 1].set_xlabel('time [s]')
#
# plt.savefig(save_path + "/" + "velocity_tracking" + '.pdf')



fig = plt.figure(figsize=(12, 6))

# Plot for NL
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title('NL')
ax1.plot(pos_d_NL[:, 0], pos_d_NL[:, 1], pos_d_NL[:, 2], 'k--', label='Desired Position', linewidth=2)
ax1.plot(pos_NL[:, 0], pos_NL[:, 1], pos_NL[:, 2], 'r', label='Actual Position', linewidth=2)
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
ax1.set_xlim([min(pos_NL[:, 0])-0.1, max(pos_NL[:, 0])+0.1])
ax1.set_ylim([min(pos_NL[:, 1])-0.1, max(pos_NL[:, 1])+0.1])
ax1.set_zlim([min(pos_NL[:, 2])-0.1, max(pos_NL[:, 2])+0.1])
ax1.legend()

# Plot for L
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_title('L')
ax2.plot(pos_d_L[:, 0], pos_d_L[:, 1], pos_d_L[:, 2], 'k--', label='Desired Position', linewidth=2)
ax2.plot(pos_L[:, 0], pos_L[:, 1], pos_L[:, 2], 'r', label='Actual Position', linewidth=2)
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Y [m]')
ax2.set_zlabel('Z [m]')
ax2.set_xlim([min(pos_NL[:, 0])-0.1, max(pos_NL[:, 0])+0.1])
ax2.set_ylim([min(pos_NL[:, 1])-0.1, max(pos_NL[:, 1])+0.1])
ax2.set_zlim([min(pos_NL[:, 2])-0.1, max(pos_NL[:, 2])+0.1])
ax2.legend()

plt.tight_layout()

# Save the figure
plt.savefig(save_path + "/" + "position_3d.pdf")



# fig = plt.figure(figsize=(12, 6))
#
# # Plot for NL
# ax1 = fig.add_subplot(121, projection='3d')
# ax1.set_title('NL')
# ax1.plot(vel_d_NL[:, 0], vel_d_NL[:, 1], vel_d_NL[:, 2], 'k--', label='Desired Velocity', linewidth=2)
# ax1.plot(vel_NL[:, 0], vel_NL[:, 1], vel_NL[:, 2], 'r', label='Actual Velocity', linewidth=2)
# ax1.set_xlabel('VX [m]')
# ax1.set_ylabel('VY [m]')
# ax1.set_zlabel('VZ [m]')
# ax1.legend()
#
# # Plot for L
# ax2 = fig.add_subplot(122, projection='3d')
# ax2.set_title('L')
# ax2.plot(vel_d_L[:, 0], vel_d_L[:, 1], vel_d_L[:, 2], 'k--', label='Desired Velocity', linewidth=2)
# ax2.plot(vel_L[:, 0], vel_L[:, 1], vel_L[:, 2], 'r', label='Actual Velocity', linewidth=2)
# ax2.set_xlabel('VX [m]')
# ax2.set_ylabel('VY [m]')
# ax2.set_zlabel('VZ [m]')
# ax2.legend()
#
# plt.tight_layout()
#
# # Save the figure
# plt.savefig(save_path + "/" + "velocity_tracking_3d.pdf")
# plt.show()


#### plot the joint values:

# # Plotting ex - position values for NL and L cases side by side
# fig, axs = plt.subplots(1, 2, figsize=(9, 6), sharey=True)
#
# # Plot NL case
# for i in range(3):
#     axs[0].plot(time_list_NL, [ex[i] for ex in ex_list_NL], label=f'NL ex{i + 1}')
#
# axs[0].set_ylabel('error in [m]')
# axs[0].set_xlabel('Time')
# axs[0].set_title('NL Case')
# axs[0].legend()
# axs[0].grid(True)
#
# # Plot L case
# for i in range(3):
#     axs[1].plot(time_list_L, [ex[i] for ex in ex_list_L], label=f'L ex{i + 1}')
#
# axs[1].set_xlabel('Time')
# axs[1].set_title('L Case')
# axs[1].legend()
# axs[1].grid(True)
#
# # Customize the layout
# plt.tight_layout()
#
#
# plt.savefig(save_path + "/" + "position_error_p" + '.pdf')
#
#
# # Plotting ex- angles values for NL and L cases side by side
# fig, axs = plt.subplots(1, 2, figsize=(9, 6), sharey=True)
#
# # Plot NL case
# for i in range(3, len(ex_list_NL[0])):
#     axs[0].plot(time_list_NL, [ex[i] for ex in ex_list_NL], label=f'NL ex{i + 1}')
#
# axs[0].set_ylabel('error in [rad]')
# axs[0].set_xlabel('Time')
# axs[0].set_title('NL Case')
# axs[0].legend()
# axs[0].grid(True)
#
# # Plot L case
# for i in range(3, len(ex_list_L[0])):
#     axs[1].plot(time_list_L, [ex[i] for ex in ex_list_L], label=f'L ex{i + 1}')
#
# axs[1].set_xlabel('Time')
# axs[1].set_title('L Case')
# axs[1].legend()
# axs[1].grid(True)
#
# # Customize the layout
# plt.tight_layout()
#
#
# plt.savefig(save_path + "/" + "position_error_a" + '.pdf')
#
# # Plotting ev position values for NL and L cases side by side
# fig, axs = plt.subplots(1, 2, figsize=(9, 5), sharey=True)
#
# # Plot NL case
# for i in range(3):
#     axs[0].plot(time_list_NL, [ev[i] for ev in ev_list_NL], label=f'NL ev{i + 1}')
#
# axs[0].set_ylabel('error in [m/s]')
# axs[0].set_xlabel('Time')
# axs[0].set_title('NL Case')
# axs[0].legend()
# axs[0].grid(True)
#
# # Plot L case
# for i in range(3):
#     axs[1].plot(time_list_L, [ev[i] for ev in ev_list_L], label=f'L ev{i + 1}')
#
# axs[1].set_xlabel('Time')
# axs[1].set_title('L Case')
# axs[1].legend()
# axs[1].grid(True)
#
# # Customize the layout
# plt.tight_layout()
# plt.savefig(save_path + "/" + "velocity_error_p" + '.pdf')
#
#
#
# # Plotting ev angles values for NL and L cases side by side
# fig, axs = plt.subplots(1, 2, figsize=(9, 5), sharey=True)
# # Plot NL case
# for i in range(3, len(ev_list_NL[0])):
#     axs[0].plot(time_list_NL, [ev[i] for ev in ev_list_NL], label=f'NL ev{i + 1}')
#
# axs[0].set_ylabel('error in [rad/s]')
# axs[0].set_xlabel('Time')
# axs[0].set_title('NL Case')
# axs[0].legend()
# axs[0].grid(True)
#
# # Plot L case
# for i in range(3, len(ev_list_L[0])):
#     axs[1].plot(time_list_L, [ev[i] for ev in ev_list_L], label=f'L ev{i + 1}')
#
# axs[1].set_xlabel('Time')
# axs[1].set_title('L Case')
# axs[1].legend()
# axs[1].grid(True)
#
# # Customize the layout
# plt.tight_layout()
# plt.savefig(save_path + "/" + "velocity_error_a" + '.pdf')


# Plotting q angles values for NL and L cases side by side
fig, axs = plt.subplots(1, 2, figsize=(9, 5), sharey=True)
# Plot NL case
for i in range(0, len(q_list_NL[0])):
    axs[0].plot(time_list_NL, [q[i] for q in q_list_NL], label=f'NL q{i + 1}')

axs[0].set_ylabel('joint position [rad]')
axs[0].set_xlabel('Time')
axs[0].set_title('NL Case')
axs[0].legend()
axs[0].grid(True)

# Plot L case
for i in range(0, len(q_list_L[0])):
    axs[1].plot(time_list_L, [q[i] for q in q_list_L], label=f'L q{i + 1}')

axs[1].set_xlabel('Time')
axs[1].set_title('L Case')
axs[1].legend()
axs[1].grid(True)

# Customize the layout
plt.tight_layout()
plt.savefig(save_path + "/" + "joints" + '.pdf')

# Plotting q angles values for NL and L cases side by side
fig, axs = plt.subplots(1, 2, figsize=(9, 5), sharey=True)
# Plot NL case
for i in range(0, len(dq_list_NL[0])):
    axs[0].plot(time_list_NL, [dq[i] for dq in dq_list_NL], label=f'NL dq{i + 1}')

axs[0].set_ylabel('joint velocity [rad/s]')
axs[0].set_xlabel('Time')
axs[0].set_title('NL Case')
axs[0].legend()
axs[0].grid(True)

# Plot L case
for i in range(0, len(dq_list_L[0])):
    axs[1].plot(time_list_L, [dq[i] for dq in dq_list_L], label=f'L dq{i + 1}')

axs[1].set_xlabel('Time')
axs[1].set_title('L Case')
axs[1].legend()
axs[1].grid(True)

# Customize the layout
plt.tight_layout()
plt.savefig(save_path + "/" + "joints_velocity" + '.pdf')


# Plotting q angles values for NL and L cases side by side
fig, axs = plt.subplots(1, 2, figsize=(9, 5), sharey=True)
# Plot NL case
for i in range(0, len(t_list_NL[0])):
    axs[0].plot(time_list_NL, [t[i] for t in t_list_NL], label=f'NL torque{i + 1}')

axs[0].set_ylabel('torques [Nm]')
axs[0].set_xlabel('Time')
axs[0].set_title('NL Case')
axs[0].legend()
axs[0].grid(True)

# Plot L case
for i in range(0, len(t_list_L[0])):
    axs[1].plot(time_list_L, [t[i] for t in t_list_L], label=f'L torque{i + 1}')

axs[1].set_xlabel('Time')
axs[1].set_title('L Case')
axs[1].legend()
axs[1].grid(True)

# Customize the layout
plt.tight_layout()
plt.savefig(save_path + "/" + "efforts" + '.pdf')
