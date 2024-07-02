import pybullet as p
import pybullet_data
import os

# Connect to PyBullet
p.connect(p.GUI)

# Path to your URDF file
urdf_file_path = os.path.abspath("new_acrobot.urdf")

# Verify the URDF file exists
if not os.path.exists(urdf_file_path):
    raise FileNotFoundError(f"URDF file not found: {urdf_file_path}")

# Load the acrobot URDF
acrobot_id = p.loadURDF(urdf_file_path, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1])

# Set the initial pose position and orientation
p.resetBasePositionAndOrientation(acrobot_id, [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

# Set the initial joint velocities
p.resetJointState(acrobot_id, jointIndex=0, targetValue=0, targetVelocity=1.0) # base_joint1
p.resetJointState(acrobot_id, jointIndex=1, targetValue=0, targetVelocity=0)   # base_joint2
p.resetJointState(acrobot_id, jointIndex=2, targetValue=0, targetVelocity=0)   # joint1

# Run the simulation
for _ in range(10000):
    p.stepSimulation()
    p.setTimeStep(1./240.)

p.disconnect()
