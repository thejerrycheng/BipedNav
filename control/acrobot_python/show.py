import pybullet as p
import pybullet_data
import time

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

    # Run the simulation for a few seconds to visualize
    for _ in range(10000):
        p.stepSimulation()
        time.sleep(1./240.)

    # Disconnect from the simulator
    p.disconnect()

if __name__ == "__main__":
    main()
