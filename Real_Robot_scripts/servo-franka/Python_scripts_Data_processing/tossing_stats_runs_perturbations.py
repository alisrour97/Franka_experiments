import pandas as pd
import matplotlib.pyplot as plt
import datetime
import os
import numpy as np
import seaborn as sns
# Specify the path to your CSV file


'''latex for figures'''
plt.rcParams['text.latex.preamble'] = r'\usepackage{bm} \usepackage{amsmath} \usepackage{lmodern}'
plt.rcParams.update({'text.usetex': True, 'font.size': 16, 'font.family': 'lmodern'})
plt.rcParams.update({'figure.max_open_warning': 0})

fontsizetitle = 32

num = 15 # specify how many logs there are
traj_name = 'INITw_ki' # specify trajectory name
traj_name2 = 'PIw_ki' # specify trajectory name


now = datetime.datetime.now()
save_path = 'save/stats/' + "compare__" + traj_name + "__" + traj_name2 + "__" + now.strftime(
    '%Y-%m-%d')
if not os.path.isdir(save_path):
    os.mkdir(save_path)

# Plotting in 3D
fig = plt.figure(figsize=(16, 9))
# Plot for NL
ax1 = fig.add_subplot(121, projection='3d')
ax2 = fig.add_subplot(122, projection='3d')
plt.suptitle('3D Tracking')


list_x_init = []
list_x_pi = []

list_v_init = []
list_v_pi = []

list_u_init = []
list_u_pi = []

index = 1400

tmp_path = '../Data4/log_dyn_'
for i in range(num):

    csv_file_path = tmp_path + traj_name + '_' + str(i) + '.csv'
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_file_path)
    # Extract the data as lists
    time_list = df['Time'].tolist()
    q_list = df[['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']].values.tolist()
    dq_list = df[['dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7']].values.tolist()
    t_list = df[['t1', 't2', 't3', 't4', 't5', 't6', 't7']].values.tolist()
    ex_list = df[['ex1', 'ex2', 'ex3', 'ex4', 'ex5', 'ex6']].values.tolist()
    ev_list = df[['ev1', 'ev2', 'ev3', 'ev4', 'ev5', 'ev6']].values.tolist()

    pos = df[['x', 'y', 'z']].values
    vel = df[['vx', 'vy', 'vz']].values
    u = df[['t1', 't2', 't3', 't4', 't5', 't6', 't7']].values

    if i == 0: # always nominal case when i = 0 or log with number=0
        pos_d = df[['xd', 'yd', 'zd']].values
        vel_d = df[['vxd', 'vyd', 'vzd']].values
        ax1.plot(pos_d[:, 0], pos_d[:, 1], pos_d[:, 2], 'k--', label=r"Reference Trajectory", linewidth=2)
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'r', label=r"Nominal Tracking  $p = p_{c}$", linewidth=2)
        target_nom_init = pos_d[index, :]
        ax1.plot(*target_nom_init, "o", color="b", linewidth=5, markersize=15, alpha=0.4)

    if i == 1:
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'g--', label=r' Tracking $p \neq p_{c}$', linewidth=2)
        ax1.legend()

    else:
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'g--', linewidth=2)

    list_u_init.append(u)
    list_x_init.append(np.linalg.norm(pos_d[index, :] - pos[index, :]))
    list_v_init.append(np.linalg.norm(vel_d[index, :] - vel[index, :]))


    # Traj2
    csv_file_path = tmp_path + traj_name2 + '_' + str(i) + '.csv'
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_file_path)
    # Extract the data as lists
    time_list = df['Time'].tolist()
    q_list = df[['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']].values.tolist()
    dq_list = df[['dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7']].values.tolist()
    t_list = df[['t1', 't2', 't3', 't4', 't5', 't6', 't7']].values.tolist()
    ex_list = df[['ex1', 'ex2', 'ex3', 'ex4', 'ex5', 'ex6']].values.tolist()
    ev_list = df[['ev1', 'ev2', 'ev3', 'ev4', 'ev5', 'ev6']].values.tolist()

    pos = df[['x', 'y', 'z']].values
    vel = df[['vx', 'vy', 'vz']].values
    u = df[['t1', 't2', 't3', 't4', 't5', 't6', 't7']].values
    tim = df['Time'].values

    if i == 0:  # always nominal case when i = 0 or log with number=0
        pos_d2 = df[['xd', 'yd', 'zd']].values
        vel_d2 = df[['vxd', 'vyd', 'vzd']].values
        ax2.plot(pos_d2[:, 0], pos_d2[:, 1], pos_d2[:, 2], 'k--', label=r"Reference Trajectory", linewidth=2)
        ax2.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'r', label=r"Nominal Tracking  $p = p_{c}$", linewidth=2)
        target_nom_pi = pos_d2[index, :]
        ax2.plot(*target_nom_pi, "o", color="b", linewidth=5, markersize=15, alpha=0.4)

    if i == 1:
        ax2.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'g--', label=r' Tracking $p \neq p_{c}$', linewidth=2)
        ax2.legend()

    else:
        ax2.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'g--', linewidth=2)


    list_x_pi.append(np.linalg.norm(pos_d2[index, :] - pos[index, :]))
    list_v_pi.append(np.linalg.norm(vel_d[index, :] - vel[index, :]))
    list_u_pi.append(u)



min_x = -0.1
max_x = 0.65
min_y = -0.55
max_y = 0.3
min_z = 0.4
max_z = 0.7
elev = 15
azim = -15


ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
# # Set scale of 0.1m on each axis
ax1.set_xlim([min_x, max_x])
ax1.set_ylim([min_y, max_y])
ax1.set_zlim([min_z, max_z])

ax1.view_init(elev=elev, azim=azim)  # Adjust elevation and azimuth angles as needed
ax1.set_title(r"")

ax2.set_xlim([min_x, max_x])
ax2.set_ylim([min_y, max_y])
ax2.set_zlim([min_z, max_z])


ax2.view_init(elev=elev, azim=azim)  # Adjust elevation and azimuth angles as needed
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Y [m]')
ax2.set_zlabel('Z [m]')


plt.savefig(save_path + "/" + "position_3d.pdf")


name = ["$INIT$", "$OPT_a$"]
d1 = np.array(list_x_init)
d2 = np.array(list_x_pi)
df_d1 = pd.DataFrame({name[0]: d1})
df_d2 = pd.DataFrame({name[1]: d2})

d3 = np.array(list_v_init)
d4 = np.array(list_v_pi)
df_d3 = pd.DataFrame({name[0]: d3})
df_d4 = pd.DataFrame({name[1]: d4})

# df_pos.columns = [r"INIT", r"$OPT_a$]

# figure, axes = plt.subplots(1, 1, figsize=(12, 8))
# sns.violinplot(data=df_pos, order=["INIT", "PI"], linewidth=4, ax=axes)
# axes.set_ylabel(r'Target reach in [\text{m}]', fontsize=fontsizetitle)
# axes.set_title(r'Target reach', fontsize=fontsizetitle)
# axes.tick_params(axis='both', which='major', labelsize=25)
# axes.yaxis.grid()
# plt.savefig(save_path + "/" + "violin_targets.pdf")


num_bins = np.arange(0, 0.1, 0.01)
figure, ax = plt.subplots(1, 1, figsize=(12, 8))
# Define colors for the samples
sample_colors = ['blue', 'red']

for i, df in enumerate([df_d1, df_d2]):
    sns.histplot(data=df, x=name[i], bins=num_bins, label=name[i], alpha=0.5, ax=ax, color=sample_colors[i])
    mean_val = df[name[i]].mean()  # Calculate the mean
    ax.axvline(mean_val, color=sample_colors[i], linewidth=2, linestyle='dashed', label=f'Average: {mean_val:.3f}')
ax.set_xlabel('')
ax.set_ylabel('')
ax.legend()

# Add common title, x-label, and y-label
figure.suptitle(r'Experiments', fontsize=30)
figure.text(0.5, 0.04, r'Distance to the desired target in [m]', ha='center', fontsize=30)
figure.text(0.02, 0.5, 'Count', va='center', rotation='vertical', fontsize=30)

plt.savefig(save_path + "/" + "histogram_dist_dev.pdf")



"""Velocity"""
num_bins = np.arange(0, 0.41, 0.05)
figure, ax = plt.subplots(1, 1, figsize=(12, 8))
# Define colors for the samples
sample_colors = ['blue', 'red']

for i, df in enumerate([df_d3, df_d4]):
    sns.histplot(data=df, x=name[i], bins=num_bins, label=name[i], alpha=0.5, ax=ax, color=sample_colors[i])
    mean_val = df[name[i]].mean()  # Calculate the mean
    ax.axvline(mean_val, color=sample_colors[i], linewidth=2, linestyle='dashed', label=f'Average: {mean_val:.3f}')
ax.set_xlabel('')
ax.set_ylabel('')
ax.legend()

# Add common title, x-label, and y-label
figure.suptitle(r'Experiments', fontsize=30)
figure.text(0.5, 0.04, r'Velocity Deviation', ha='center', fontsize=30)
figure.text(0.02, 0.5, 'Count', va='center', rotation='vertical', fontsize=30)

plt.savefig(save_path + "/" + "histogram_vel_dev.pdf")



# Plotting q angles values for NL and L cases side by side
fig, axs = plt.subplots(1, 2, figsize=(9, 5), sharey=True)
# Plot NL case
h = 4
for i in range(0, num):
    if i == 0:
        axs[0].plot(tim, list_u_init[i][:, h], 'r', linewidth= 2)
    else:
        axs[0].plot(tim, list_u_init[i][:, h], 'g', linewidth=1.5, alpha=0.7)

axs[0].set_ylabel('torques [Nm]')
axs[0].set_xlabel('Time')
axs[0].set_title('INIT Case')
axs[0].grid(True)

# Plot L case
for i in range(0, num):
    if i == 0:
        axs[1].plot(tim, list_u_pi[i][:, h], 'r', linewidth= 2, label=f'Torque {h} nominal')
    else:
        axs[1].plot(tim, list_u_pi[i][:, h], 'g', linewidth=1.5, alpha=0.7, label=f'Torque {h} Perturbed')
        if i == 1:
            axs[1].legend()



axs[1].set_xlabel('Time')
axs[1].set_title('PI Case')
axs[1].grid(True)


# Customize the layout
plt.tight_layout()
plt.savefig(save_path + "/" + "efforts" + '.pdf')

plt.show()