
# Extract Point Clouds from ROS Bag

## Overview

`extract_pointclouds.py` is a Python script that extracts point cloud data from a ROS bag file, applies subsampling and noise reduction filters, and saves the processed point clouds to PLY files. The point clouds are saved in a directory named `pointcloud`.

## Features

- Extracts point clouds from a specified topic in a ROS bag file.
- Subsamples the point cloud based on a given percentage.
- Applies noise reduction filters and other useful filters from the Open3D library.
- Saves the processed point clouds to PLY format.

## Dependencies

- ROS (Robot Operating System)
- Open3D
- NumPy

## Setup

### Install ROS

Ensure you have ROS installed. Follow the official ROS installation guide for your operating system and ROS distribution.

### Create a Conda Environment

Create a conda environment named `biped` using Python 3.10:

```bash
conda create -n biped python=3.10
conda activate biped
```

### Install Python Dependencies

Create a `requirements.txt` file with the following content:

```
open3d
numpy
```

Then install the dependencies:

```bash
pip install -r requirements.txt
```

### Install ROS Packages

Install the necessary ROS packages:

```bash
sudo apt-get install ros-<ros_distro>-rosbag
sudo apt-get install ros-<ros_distro>-sensor-msgs
```

Replace `<ros_distro>` with your specific ROS distribution (e.g., `melodic`, `noetic`, etc.).

## Usage

1. **Ensure ROS Environment**: Source your ROS environment. For example:

   ```bash
   source /opt/ros/<ros_distro>/setup.bash
   ```

2. **Make the Script Executable**: Change the file permissions to make the script executable:

   ```bash
   chmod +x extract_pointclouds.py
   ```

3. **Run the Script**: Execute the script with the path to your ROS bag file and the subsample percentage. The script will create a `pointcloud` folder if it does not exist and save the extracted point clouds in PLY format there:

   ```bash
   ./extract_pointclouds.py your_rosbag_file.bag 0.1
   ```

   Replace `your_rosbag_file.bag` with the path to your ROS bag file and `0.1` with the desired subsample percentage (e.g., `0.1` for 10%).

## Example

```bash
./extract_pointclouds.py daniel_1_20240610_190355.bag 0.2
```

This command will extract point cloud data from `daniel_1_20240610_190355.bag`, subsample it to 20% of its original size, apply noise reduction filters, and save the processed point clouds to the `pointcloud` folder in PLY format.