from matplotlib import pyplot as plt
import numpy as np

def rot_mat(ang_rad):
    c = np.cos(ang_rad)
    s = np.sin(ang_rad)
    return np.array([[c, -s],[s, c]])

def servo_traj(view_deg, gain, hand_start_x, hand_start_y): 
    target = np.array([0,0])
    hand_start = np.array([hand_start_x, hand_start_y])
    robot_view = rot_mat(view_deg/180.0 * np.pi)
    
    t_0 = 0.0
    t_end = 10.0
    hz = 15.0
    T = 1.0/hz
    
    t = t_0
    hand_pos = hand_start
    hand_traj = []
    hand_traj.append(hand_pos)
    while t < t_end: 
        target_view = np.matmul(robot_view, target)
        hand_view = np.matmul(robot_view, hand_pos)
        view_error = target_view - hand_view
        hand_pos = hand_pos + (gain * view_error)
        hand_traj.append(hand_pos)
        t = t + T
        
    return hand_traj

def plot_traj(hand_traj):
    fig = plt.figure()
    x = [p[0] for p in hand_traj]
    y = [p[1] for p in hand_traj]
    plt.plot(x,y, 'o-', color='blue')
    plt.show()

def plot_servo(view_deg=0.0, gain=0.2, hand_start_x=10.0, hand_start_y=10.0):
    traj = servo_traj(view_deg, gain, hand_start_x, hand_start_y)
    plot_traj(traj)

if __name__ == '__main__':
    plot_servo()
