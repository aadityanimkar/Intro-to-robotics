import numpy as np
import matplotlib.pyplot as plt 

def inv_kinematics_2R(x,y):
    ans = []
    l1 = 5
    l2 = 5
    r = np.sqrt(x**2 + y**2) 
    if r > l1+l2 or r < abs(l1-l2):
        print("the point is not reachable")
        return
    theta_1 = np.arccos(((x**2 + y**2 - l1**2 - l2**2)/(2 * l1 * l2)))
    theta_2 = - theta_1
    q1_1 = np.arctan2(l2 * np.sin(theta_1) , (l1 + l2 * np.cos(theta_1)))
    q1_2 = np.arctan2(l2 * np.sin(theta_2) , (l1 + l2 * np.cos(theta_2)))
    q2_1 = theta_1 + q1_1
    q2_2 = theta_2 + q1_2
    ans.append((q1_1,q2_1))
    ans.append((q1_2,q2_2))
    return ans

print(inv_kinematics_2R(3,5))