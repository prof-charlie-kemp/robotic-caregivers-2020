#!/usr/bin/python
from __future__ import print_function
import stretch_body.robot as rb
import time
import numpy as np

def deg_to_rad(deg):
    return deg * (3.14159265359 / 180.0)

def wrist_forward_kinematics(base_m, lift_m, arm_m):

    use_unit_vectors = True
    if use_unit_vectors: 
        x_axis = np.array([1.0, 0.0, 0.0])
        y_axis = np.array([0.0, 1.0, 0.0])
        z_axis = np.array([0.0, 0.0, 1.0])

        position_xyz = (x_axis * arm_m) + (y_axis * base_m) + (z_axis * lift_m)
        print('unit vectors: position_xyz =', position_xyz)
        
    use_matrices = True
    if use_matrices:
        matrix = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
        matrix = np.eye(3)

        joints = np.array([arm_m, base_m, lift_m])
        
        position_xyz = np.matmul(matrix, joints)
        print('matrix multiplication: position_xyz =', position_xyz)

    return position_xyz


def get_robot_joint_state(robot):
    lift_height_m = robot.lift.status['pos']
    arm_extension_m = robot.arm.status['pos']
    wrist_yaw_rad = robot.end_of_arm.motors['wrist_yaw'].status['pos']
    base_x = 100.0 * robot.base.status['x']
    base_y = 100.0 * robot.base.status['y']
    base_theta = robot.base.status['theta']
    wrist_yaw_rad = robot.end_of_arm.motors['wrist_yaw'].status['pos']

    joint_state = {'lift_height_m':lift_height_m,
                    'arm_extension_m': arm_extension_m,
                    'wrist_yaw_rad': wrist_yaw_rad,
                    'base_x': base_x,
                    'base_y': base_y,
                    'base_theta': base_theta,
                    'wrist_yaw_rad': wrist_yaw_rad}
    return joint_state

def main():
    robot = rb.Robot()
    robot.startup()
    robot_calibrated = robot.is_calibrated()

    if not robot_calibrated:
        print('ERROR: Robot not calibrated! Exiting...')
        robot.stop()
        return

    time.sleep(1.0)

    joint_state = get_robot_joint_state(robot)
    print('start joint_state =', joint_state)

    base_m = 0.1
    lift_m = 0.3
    arm_m = 0.2
    wrist_position_xyz = wrist_forward_kinematics(base_m, lift_m, arm_m)
    print('wrist_position_xyz =', wrist_position_xyz)
    
    d_m = 0.1
    robot.base.translate_by(d_m)
    #d_rad = deg_to_rad(10.0)
    #robot.base.rotate_by(d_rad)
    d_m = 0.005
    robot.lift.move_by(d_m)
    d_m = 0.005
    robot.arm.move_by(d_m)
    d_rad = deg_to_rad(10.0)
    robot.end_of_arm.move_by('wrist_yaw', d_rad)
    pan_rad = deg_to_rad(10.0)
    tilt_rad = deg_to_rad(10.0)
    robot.head.move_by((pan_rad, tilt_rad))
    gripper_percent = 0.1
    robot.end_of_arm.move_by('stretch_gripper', gripper_percent)
    robot.push_command()
    
    time.sleep(2.0)

    joint_state = get_robot_joint_state(robot)
    print('end joint_state =', joint_state)
    
    robot.stop()

if __name__ == "__main__":
    main()

