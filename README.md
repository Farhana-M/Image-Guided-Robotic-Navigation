# Image-Guided Robotic Navigation

This system is designed to select an optimal trajectory from a set of inputs. The PathPlanningV1 algorithm takes in a set of possible entry and target points (represented as a `vtkMRMLMarkupsFiducialNode`), and two binary image volumes representing the critical structures and target structure (represented as either a `vtkMRMLLabelMapVolumeNode` or `vtkMRMLLabelVolumeNode`). It returns two points representing a selected final trajectory (represented as `vtkMRMLMarkupsFiducialNode` and `vtkMRMLMarkupLineNode`). The final trajectory is selected with the constraints:
- (a) Avoidance of a user-defined critical structure,
- (b) Placement of the tool into a user-defined target structure,
- (c) Trajectory below a user-defined length,
- (d) Maximizing distance from the user-defined critical structure.

The selected trajectory points are sent to ROS via OpenIGTLink communication. OpenIGTLinkIF and ROS-IGTL-Bridge is used to facilitate communication between 3D Slicer and ROS. A node referred to as `Rmove_robot_to_pose` is implemented in ROS to receive data from ROS-IGTL-Bridge, convert this message to a pose message, and command the robot to move to the received position.

## Prerequisites

Before using the modules inthis repository, ensure you have the following installed:

- [3D Slicer](https://www.slicer.org)
- [ROS Noetic](http://www.ros.org)
- [MoveIt! for ROS Noetic](https://moveit.ros.org)
- [VTK](https://vtk.org)
- [SimpleITK](http://www.simpleitk.org)
- [OpenIGTLinkIF](http://www.slicerigt.org/wp/openigtlink/)
- [Scipy](https://www.scipy.org)
- [ROS IGTL Bridge](https://github.com/openigtlink/ROS-IGTL-Bridge)

## Installation

### Step 1: Install 3D Slicer and Extension

1. **Download and Install 3D Slicer**: 
   - Go to the [3D Slicer website](https://www.slicer.org) and download the appropriate version for your operating system.
   - Install 3D Slicer following the provided instructions.

2. **Download and Add PathPlanningV1 Extension**:
   - Download the PathPlanningV1 extension package from the [GitHub repository](https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/tree/main/PathPlanningV1).
   - Open 3D Slicer.
   - Go to `Edit` > `Application Settings` > `Modules`.
   - In the `Additional module paths` section, click the `Add` button and browse to the directory where the PathPlanningV1 extension is located.
   - Select the folder containing the PathPlanningV1 extension and click `OK`.
   - Restart 3D Slicer.

### Step 2: Install ROS Noetic and Dependencies

1. **Install ROS Noetic**:
   - Follow the official [ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation) for your operating system.

2. **Install MoveIt!**:
   - Follow the [MoveIt! installation guide](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) for ROS Noetic.

3. **Install ROS IGTL Bridge**:
   - Clone and build the ROS IGTL Bridge repository:
     ```bash
     cd ~/catkin_ws/src
     git clone https://github.com/openigtlink/ROS-IGTL-Bridge.git
     cd ~/catkin_ws
     rosdep install --from-paths src --ignore-src -r -y
     catkin_make
     ```

### Step 3: Clone the GitHub Repository for MoveIt! Config

1. **Clone the moveit_config3 Config Package**:
   - Navigate to the `src` directory of your ROS workspace:
     ```bash
     cd ~/catkin_ws/src
     ```
   - Clone the repository:
     ```bash
     git clone https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/tree/main/moveit_config3/config
     ```

### Step 4: Install Dependencies and Build Workspace

1. **Install Dependencies**:
   - Navigate to your ROS workspace:
     ```bash
     cd ~/catkin_ws
     ```
   - Install dependencies:
     ```bash
     rosdep install --from-paths src --ignore-src -r -y
     ```

2. **Build the Workspace**:
   - Build the ROS workspace:
     ```bash
     catkin_make
     ```
   - Alternatively, if using `catkin_tools`:
     ```bash
     catkin build
     ```

3. **Source the Workspace**:
   - Source the workspace:
     ```bash
     source ~/catkin_ws/devel/setup.bash
     ```

4. **Change the Path to the URDF**:
   - Ensure that the path to the robot's URDF file is correctly set in the MoveIt! configuration files. Typically, this can be found and modified in the `moveit_config3/config` directory, for example in the `robot_description.yaml` file:
     ```yaml
     robot_description: 'file:///path/to/6dof_final_V4.urdf'
     ```

## Using PathPlanningV1

### Step 1: Prepare Your Data

1. **Upload Data**:
   - Download the brain parcellation set from the [GitHub repository](https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/tree/main/BrainParcellation).
   - Load the data into 3D Slicer.
   - When the data is loaded, select `Show Options` and ensure to select `brainstem`, `cortex`, `r_hippo`, `r_mtg`, `ventricles`, and `vessels` as label maps.

### Step 2: Set Input Data

1. **Select PathPlanningV1 Module**:
   - In 3D Slicer, navigate to the `PathPlanningV1` module.

2. **Input Parameters**:
   - Select the target region from the dropdown menu (e.g., `r_hippo` in this dataset).
   - Select a critical structure.
   - Set the entry and target points.
   - Use the slider to set a length threshold (0 to 200 mm).
   - Select `Create new point list in output Fiducials`.
   - The `Output Fiducials` will contain the target points within the target region.

3. **Generate Trajectory**:
   - Click on `Apply` when all input parameters are set.
   - If the algorithm finds a path that meets all the constraints, it will output a `vtkMRMLMarkupsFiducialNode` called `Trajectory points`, which contains the entry and target points of the selected trajectory.
   - A `vtkMRMLLineNode` called `Trajectory display` will display the path to the user.
   - Additionally, the distance map is also displayed.

### Step 3: Send Data to ROS Using OpenIGTLink

1. **Configure 3D Slicer as the Server**:
   - Open the `OpenIGTLinkIF` module in 3D Slicer.
   - Add a new connector and configure it as a server:
     - Set the `Connector Name` to something identifiable, e.g., `SlicerToROS`.
     - Set the `Type` to `Server`.
     - Choose an appropriate port, e.g., `18944`.
     - In the `I/O Configuration` section, add `Trajectory Points` to `Out`.
   - Click `Activate` to start the server.

2. **Configure ROS as the Client**:
   - On your ROS machine, use the ROS IGTL Bridge to connect to the 3D Slicer server:
     ```bash
     roslaunch ros_igtl_bridge bridge.launch
     ```

### Step 4: Launch MoveIt!

1. **Download move_robot_to_pose.py**:
   - Download the `move_robot_to_pose.py` script from the [GitHub repository](https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/tree/main/move_robot_to_pose).
     
2. **Add to Workspace and Make Executable**:
   - Move the downloaded script to your ROS workspace:
     ```bash
     mv path/to/move_robot_to_pose.py ~/catkin_ws/src/moveit_config3/
     ```
   - Make the script executable:
     ```bash
     chmod +x ~/catkin_ws/src/moveit_config3/move_robot_to_pose.py
     
3. **Launch MoveIt!**:
   - Navigate to the launch directory of the `moveit_config` package:
     ```bash
     cd ~/catkin_ws/src/moveit_config3/launch
     ```
   - Launch the MoveIt! configuration:
     ```bash
     roslaunch moveit_config3 demo.launch
     ```
3. **Add to Workspace and Make Executable**:
   - Move the downloaded script to your ROS workspace:
     ```bash
     mv path/to/move_robot_to_pose.py ~/catkin_ws/src/moveit_config3/
     ```
   - Make the script executable:
     ```bash
     chmod +x ~/catkin_ws/src/moveit_config3/move_robot_to_pose.py

4. **Run the ROS Node**:
   - Execute the ROS node to move the robot:
     ```bash
     rosrun moveit_config3 move_robot_to_pose.py
     ```

4. **Send Trajectory Points**:
   - In 3D Slicer, press the `Send` button to send the `Trajectory Points` to ROS.
   - The motion will be planned and executed.
   - If the pose, orientation, or target is not achieved, a message will be displayed to the user.
   - If the execution is successful, a message will be sent to the user indicating the success.

## Troubleshooting

- **Compatibility Issues**: Ensure that your versions of 3D Slicer, ROS Noetic, and MoveIt! are compatible.
- **Missing Dependencies**: Make sure all required dependencies are installed.
- **Error Messages**: Check the console output for any error messages and follow the hints provided to resolve issues.

For further assistance, refer to the official documentation of [3D Slicer](https://www.slicer.org), [ROS](http://www.ros.org), [MoveIt!](https://moveit.ros.org), and [ROS IGTL Bridge](https://github.com/openigtlink/ROS-IGTL-Bridge).

##

