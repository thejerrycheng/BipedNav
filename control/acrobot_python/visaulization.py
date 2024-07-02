import pybullet as p
import pybullet_data
import time
import math
import scipy.io
import numpy as np

def main():
    # Load the MAT-file data
    mat_data = scipy.io.loadmat('simulation_data.mat')
    all_time = mat_data['all_time'].flatten()
    all_torque = mat_data['all_torque'].flatten()

    # Connect to PyBullet simulator (GUI mode)
    physicsClient = p.connect(p.GUI)

    # Optionally add search path for PyBullet to load URDF files
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load the ground plane
    plane_id = p.loadURDF("plane.urdf")

    # Convert 60 degrees to radians
    initial_angle_rad = math.radians(-90)
    
    # Quaternion for 60-degree rotation around the z-axis
    initial_orientation = p.getQuaternionFromEuler([0, initial_angle_rad, 0])

    # Load the URDF file with initial orientation
    urdf_path = "urdf/acrobot.urdf"  # Replace with your URDF file path
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], baseOrientation=initial_orientation, useFixedBase=False)  # Adjust the base position to be above the ground

    # Optionally, set gravity (for simulation purposes)
    p.setGravity(0, 0, -9.81)

    # Set collision filters to prevent link collisions
    link1_index = 0  # First link
    link2_index = 1  # Second link

    # Disable collision between link1 and link2
    p.setCollisionFilterPair(robot_id, robot_id, link1_index, link2_index, enableCollision=0)

    # Joint index based on URDF structure
    joint_index = 0  # Joint2 (floating joint, not controlled)
    
    # Set the initial velocity for the base link
    initial_linear_velocity = [10, 0, 0]  # m/s in the x-direction
    initial_angular_velocity = [0, 0, 0]  # rad/s (no initial angular velocity)
    p.resetBaseVelocity(robot_id, linearVelocity=initial_linear_velocity, angularVelocity=initial_angular_velocity)


    # Set the initial joint states
    initial_angle_joint = math.radians(180)  # 30 degrees 
    p.resetJointState(robot_id, joint_index, initial_angle_joint)

    # Control loop
    start_time = time.time()
    step_index = 0

    while step_index < len(all_time):
        # Calculate the elapsed time
        elapsed_time = time.time() - start_time

        current_time = all_time[step_index]
        current_torque = all_torque[step_index]
        
        if elapsed_time >= current_time:
            # Control the joint using torque control
            p.setJointMotorControl2(bodyIndex=robot_id, jointIndex=joint_index, controlMode=p.TORQUE_CONTROL, force=current_torque)
            print(f"Time: {current_time:.2f}, Torque: {current_torque:.2f}")
            step_index += 1

        # Step simulation
        p.stepSimulation()

        # Sleep to maintain real-time simulation
        if step_index < len(all_time):
            next_time = all_time[step_index]
            time_to_sleep = next_time - elapsed_time
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

    # Disconnect from the simulator
    p.disconnect()

if __name__ == "__main__":
    main()
