import numpy as np
import matplotlib.pyplot as plt 

def inv_kinematics_2R(x,y, l1,l2):
    ans = []
    r = np.sqrt(x**2 + y**2) 
    if r > l1+l2 or r < abs(l1-l2):
        print("the point is not reachable")
        return
    theta_1 = np.arccos(((x**2 + y**2 - l1**2 - l2**2)/(2 * l1 * l2)))
    theta_2 = - theta_1
    q1_1 = np.arctan2(y,x) - np.arctan2(l2 * np.sin(theta_1) , (l1 + l2 * np.cos(theta_1)))
    q1_2 = np.arctan2(y,x) - np.arctan2(l2 * np.sin(theta_2) , (l1 + l2 * np.cos(theta_2)))
    q2_1 = theta_1 + q1_1
    q2_2 = theta_2 + q1_2
    ans.append((q1_1,q2_1))
    # ans.append((q1_2,q2_2)) 
    """ the above code gives two values of theta and therefore shall output two values of q1 and q2, but we can ignore the second values as the trajectories ploted from both the values will overlap """
    return ans[0]

def fwd_kinematics_2R(q1,q2, l1, l2):
    ans = []
    x = l1 * np.cos(q1) + l2 * np.cos(q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q2)
    ans.append((x, y))
    return ans[0]

def trajectory(x,y):
    ee_traj = []
    for x_target, y_target in zip(x, y):
        # Compute inverse kinematics to find joint angles
        q_new = inv_kinematics_2R(x_target, y_target, 5,5)
        q1 = q_new[0]
        q2 = q_new[1]

        x_ee,y_ee = fwd_kinematics_2R(q1, q2, 5,5)
        ee_traj.append([x_ee, y_ee])
# Convert to arrays for plotting
    ee_traj = np.array(ee_traj)
    return ee_traj

def plot(x,y,ee_traj):
    # Plot the results
    plt.figure()
    plt.plot(x, y, 'r--', label='Desired trajectory')
    plt.plot(ee_traj[:, 0], ee_traj[:, 1], 'b-', label='End-effector path', alpha = 0.5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('End-effector Trajectory Following')
    plt.show()
