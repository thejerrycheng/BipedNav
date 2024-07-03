import pybullet as p
import pybullet_data
import time
import numpy as np
import math

def main():
    # Connect to PyBullet simulator (GUI mode)
    physicsClient = p.connect(p.GUI)

    # Optionally add search path for PyBullet to load URDF files
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load the ground plane
    plane_id = p.loadURDF("plane.urdf")

    # Load the URDF file
    urdf_path = "urdf/biped.urdf"  # Replace with your URDF file path
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 1.35], useFixedBase=False)  # Adjust the base position to be above the ground

    # Optionally, set gravity (for simulation purposes)
    p.setGravity(0, 0, -9.81)
    
        # Set collision filters to prevent link collisions
    p.setCollisionFilterPair(robot_id, robot_id, 0, 1, enableCollision=0)  # base_link and thigh_link_1
    p.setCollisionFilterPair(robot_id, robot_id, 0, 2, enableCollision=0)  # base_link and thigh_link_2
    p.setCollisionFilterPair(robot_id, robot_id, 1, 2, enableCollision=0)  # thigh_link_1 and thigh_link_2
    p.setCollisionFilterPair(robot_id, robot_id, 1, 3, enableCollision=0)  # thigh_link_1 and calf_link_1
    p.setCollisionFilterPair(robot_id, robot_id, 2, 4, enableCollision=0)  # thigh_link_2 and calf_link_2
    p.setCollisionFilterPair(robot_id, robot_id, 3, 4, enableCollision=0)  # calf_link_1 and calf_link_2
    p.setCollisionFilterPair(robot_id, robot_id, 3, 5, enableCollision=0)  # calf_link_1 and foot_link_1
    p.setCollisionFilterPair(robot_id, robot_id, 4, 6, enableCollision=0)  # calf_link_2 and foot_link_2


    # Joint indices based on the URDF structure
    hip_joint_indices = [0, 1]  # Adjust these indices based on your URDF
    knee_joint_indices = [2, 3]  # Adjust these indices based on your URDF

    # Define the polynomial coefficients for hip and knee joint trajectories
    T = 2  # Duration of the gait cycle in seconds
    A = np.array([
        [0**3, 0**2, 0, 1],
        [T**3, T**2, T, 1],
        [3*0**2, 2*0, 1, 0],
        [3*T**2, 2*T, 1, 0]
    ])

    b_hip = np.array([0, 0.5, 0, 0])
    b_knee = np.array([0, 0.5, 0, 0])

    coeff_hip = np.linalg.solve(A, b_hip)
    coeff_knee = np.linalg.solve(A, b_knee)

    # Simulation parameters
    start_time = time.time()
    simulation_duration = 10  # seconds
    t = 0

    # Initial positions: legs vertical
    initial_hip_angle = 0
    initial_knee_angle = 0
    p.resetJointState(robot_id, hip_joint_indices[0], initial_hip_angle)
    p.resetJointState(robot_id, hip_joint_indices[1], initial_hip_angle)
    p.resetJointState(robot_id, knee_joint_indices[0], initial_knee_angle)
    p.resetJointState(robot_id, knee_joint_indices[1], initial_knee_angle)

    while t < simulation_duration:
        # Calculate the elapsed time
        elapsed_time = time.time() - start_time

        # Calculate the joint positions using the polynomial coefficients
        t_norm = elapsed_time % T  # Normalize time to the duration of the gait cycle
        hip_position = np.polyval(coeff_hip, t_norm)
        knee_position = np.polyval(coeff_knee, t_norm)

        if t_norm < T/2:
            # First phase: lift one leg (left leg, index 0)
            p.setJointMotorControl2(robot_id, hip_joint_indices[0], p.POSITION_CONTROL, targetPosition=math.pi/2)
            p.setJointMotorControl2(robot_id, knee_joint_indices[0], p.POSITION_CONTROL, targetPosition=-math.pi/2)
            p.setJointMotorControl2(robot_id, hip_joint_indices[1], p.POSITION_CONTROL, targetPosition=math.pi/6)
            p.setJointMotorControl2(robot_id, knee_joint_indices[1], p.POSITION_CONTROL, targetPosition=-math.pi/12)
        else:
            # Second phase: move the lifted leg forward, bend the other leg
            p.setJointMotorControl2(robot_id, hip_joint_indices[0], p.POSITION_CONTROL, targetPosition=-math.pi/6)
            p.setJointMotorControl2(robot_id, knee_joint_indices[0], p.POSITION_CONTROL, targetPosition=-math.pi/6)
            p.setJointMotorControl2(robot_id, hip_joint_indices[1], p.POSITION_CONTROL, targetPosition=math.pi/2)
            p.setJointMotorControl2(robot_id, knee_joint_indices[1], p.POSITION_CONTROL, targetPosition=-math.pi/3)

        # Detect impact event (first leg hits the ground vertically)
        if t_norm >= T/2 and abs(hip_position) < 0.01:
            print("Impact event: first leg hit the ground")

        # Step simulation
        p.stepSimulation()
        time.sleep(1./240.)
        t = time.time() - start_time

    # Disconnect from the simulator
    p.disconnect()

if __name__ == "__main__":
    main()


# import pybullet as p
# import pybullet_data
# import time
# import math

# def main():
#     # Connect to PyBullet simulator (GUI mode)
#     physicsClient = p.connect(p.GUI)

#     # Optionally add search path for PyBullet to load URDF files
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())

#     # Load the ground plane
#     plane_id = p.loadURDF("plane.urdf")

#     # Load the URDF file
#     urdf_path = "urdf/biped.urdf"  # Replace with your URDF file path
#     robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 1.35], useFixedBase=False)  # Adjust the base position to be above the ground

#     # Optionally, set gravity (for simulation purposes)
#     p.setGravity(0, 0, -9.81)
    
#     # Set collision filters to prevent link collisions
#     p.setCollisionFilterPair(robot_id, robot_id, 0, 1, enableCollision=0)  # base_link and thigh_link_1
#     p.setCollisionFilterPair(robot_id, robot_id, 0, 2, enableCollision=0)  # base_link and thigh_link_2
#     p.setCollisionFilterPair(robot_id, robot_id, 1, 2, enableCollision=0)  # thigh_link_1 and thigh_link_2
#     p.setCollisionFilterPair(robot_id, robot_id, 1, 3, enableCollision=0)  # thigh_link_1 and calf_link_1
#     p.setCollisionFilterPair(robot_id, robot_id, 2, 4, enableCollision=0)  # thigh_link_2 and calf_link_2
#     p.setCollisionFilterPair(robot_id, robot_id, 3, 4, enableCollision=0)  # calf_link_1 and calf_link_2
#     p.setCollisionFilterPair(robot_id, robot_id, 3, 5, enableCollision=0)  # calf_link_1 and foot_link_1
#     p.setCollisionFilterPair(robot_id, robot_id, 4, 6, enableCollision=0)  # calf_link_2 and foot_link_2


#     # Joint indices based on the URDF structure
#     hip_joint_indices = [0, 1]  # Adjust these indices based on your URDF
#     knee_joint_indices = [2, 3]  # Adjust these indices based on your URDF

#     # Set initial joint states to zero
#     for joint_index in hip_joint_indices + knee_joint_indices:
#         p.resetJointState(robot_id, joint_index, targetValue=0)

#     # Simulation parameters
#     frequency = 0.5  # Walking frequency in Hz
#     amplitude = 0.5  # Joint angle amplitude in radians
#     phase_offset = math.pi  # Phase offset for opposite legs

#     # Run the simulation for a few seconds to visualize
#     start_time = time.time()
#     while time.time() - start_time < 10:  # Run for 10 seconds
#         elapsed_time = time.time() - start_time
#         # Calculate the joint positions
#         for i, joint_index in enumerate(hip_joint_indices):
#             angle = amplitude * math.sin(2 * math.pi * frequency * elapsed_time + i * phase_offset)
#             p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=angle)
        
#         for i, joint_index in enumerate(knee_joint_indices):
#             angle = amplitude * math.sin(2 * math.pi * frequency * elapsed_time + i * phase_offset + math.pi / 2)
#             p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=angle)

#         # Step simulation
#         p.stepSimulation()
#         time.sleep(1./240.)

#     # Disconnect from the simulator
#     p.disconnect()

# if __name__ == "__main__":
#     main()
