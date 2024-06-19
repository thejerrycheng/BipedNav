## process.py

This script processes our .bag data, extracting subsequences of depth and RGB images, as well as camera parameters.

### Installation

You need Python 3.8.10 exactly.
```
conda create -n bpdproc python=3.8.10 pip
```

This relies on a ROS Noetic Installation. To install Python dependencies, run the script:
```
pip install -r process_requirements.txt
```

### Running the script

**Keep the previous .bag files after processing in case we need to reference them in the future.**

The script is invoked via Python and takes two command line arguments: 
* --bag: path to the bag file to process
* --out: an output directory (ex. "processed")

Sample Usage:
```
python data_scripts/process.py --bag ../dataset/WI_20240608_174726.bag --out processed
```

The script opens a cv2 window with four images in a grid. The top row contains depth images, and the bottom contains RGB. The first column corresponds to the start frame of the subsequence, and the second column is the end frame.

### Controls

#### Moving start, end frame.

Use keyboard controls T, Y, G, H to move start frame position, and U, I, J, K to move end frame.
* T - move start frame left by 5 frames
* Y - move start frame right by 5 frames
* G - move start frame left by 1 frame
* H - move start frame right by 1 frame
* U - move end frame left by 5 frames
* I - move end frame right by 5 frames
* J - move end frame left by 1 frame
* K - move end frame right by 1 frame

#### Auxilary Controls

You can play a video between the start and end frame by pressing "p", and stop by pressing "x".

To exit the script nicely, press "x" with the cv2 displays open. If you accidentally close the cv2 window and the script is running, you have to manually send a kill signal to the original process.

#### Saving

Press "s" to save the selected subsequence. A tkinter GUI window will open, prompting you to enter a suffix. You can enter anything, but for consistency, use "up" to label sequences climbing stairs, and "down" for descending stairs.

The script will save the sequence into the following locations. Currently, the script samples every 6th frame in the subsequence, turning 30Hz into 5Hz.
```
{out}/{bag_filename}-{suffix}/rgb/*.png
{out}/{bag_filename}-{suffix}/depth/*.png
{out}/{bag_filename}-{suffix}/color_camera_info.json
```