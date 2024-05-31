# Image-Guided Robotic Navigation

# System Overview

This system utilizes 3D Slicer, OpenIGTLink communication, and ROS to select the optimal trajectory for a surgical navigation task, send the trajectory to ROS, and move a simulated 6DOF robot to the desired Cartesian coordinates.

The `PathPlanningV1` algorithm takes in a set of possible entry and target points (represented as `vtkMRMLMarkupsFiducialNode`), and two binary image volumes representing the critical structures and target structure (represented as `vtkMRMLLabelMapVolumeNode`). It returns two points representing a selected final trajectory (represented as `vtkMRMLMarkupsFiducialNode` and `vtkMRMLMarkupsLineNode`). The final trajectory is selected with the constraints:

- **Target Placement**: Ensures the tool is accurately placed within a user-defined target structure.
- **Critical Structure Avoidance**: Avoids user-defined critical structures to prevent damage.
- **Length Constraint**: Maintains the trajectory below a user-defined length for efficiency.
- **Maximized Distance**: Maximizes the distance from critical structures for added safety.

## Communication with ROS

The selected trajectory points are sent to ROS via OpenIGTLink communication. This system leverages `OpenIGTLinkIF` and `ROS-IGTL-Bridge` to facilitate communication between 3D Slicer and ROS. In ROS, a node referred to as `move_robot_to_pose` is implemented to:

1. **Receive Data**: Accepts trajectory data from the `ROS-IGTL-Bridge`.
2. **Convert Data**: Transforms the received trajectory points into a pose message.
3. **Command the Robot**: Utilizes the `moveit_config3` package to instruct the robot to move to the specified position.

## Prerequisites

Before using the modules in this repository, ensure you have the following installed:

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
   - Go to `Modules` > `Examples`. The PathPlanningV1 extension should be visible here. 

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

### Step 3: Configuring and Launching Your MoveIt! Package

In this step, you will configure and launch your MoveIt! package. This involves downloading the necessary directories, updating the URDF file path, sourcing the workspace, and launching the MoveIt! configuration. Follow these detailed instructions to complete the setup:

1. **Download and Extract the `moveit_config3` Directory**
   - Download and extract the directory:
     ```bash
     # Navigate to your catkin workspace's src directory
     cd ~/catkin_ws/src

     # Download the zip file of the repository
     wget https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/archive/main.zip -O Image-Guided-Robotic-Navigation.zip

     # Extract the zip file
     unzip Image-Guided-Robotic-Navigation.zip

     # Move the extracted contents to the new directory
     mv Image-Guided-Robotic-Navigation-main/moveit_config3 ~/catkin_ws/src/moveit_config3

     # Clean up
     rm -rf Image-Guided-Robotic-Navigation.zip Image-Guided-Robotic-Navigation-main
     ```

2. **Download the Robot's URDF**:
   - Download the URDF file for the 6dof robot from the [GitHub directory](https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/blob/main/URDF/6dof_final_V4.urdf).

3. **Locate and Edit the `.setup_assistant` File**:
   - Navigate to the folder where the `.setup_assistant` file is located. This can be found within the `moveit_config3` configuration package:
     ```bash
     cd ~/catkin_ws/src/moveit_config3
     ```
   - Open the `.setup_assistant` file in a text editor:
     ```bash
     nano .setup_assistant
     ```
   - Find the section that specifies the path to the URDF file. It should look something like this:
     ```yaml
     moveit_setup_assistant_config:
       URDF:
         package: ""
         relative_path: /home/rosbox/Downloads/6dof_final_V4.urdf
         xacro_args: ""
       SRDF:
         relative_path: config/6dof_robot.srdf
       CONFIG:
         author_name: FM
         author_email: moosafarhana@gmail.com
         generated_timestamp: 1717066434
     ```
   - Update the `relative_path` to point to the new location of your URDF file. For example:
     ```yaml
     moveit_setup_assistant_config:
       URDF:
         package: ""
         relative_path: /home/rosbox/catkin_ws/src/moveit_config3/6dof_final_V4.urdf
         xacro_args: ""
       SRDF:
         relative_path: config/6dof_robot.srdf
       CONFIG:
         author_name: FM
         author_email: moosafarhana@gmail.com
         generated_timestamp: 1717066434
     ```
   - Save the changes:
     - Press `Ctrl+O` to save.
     - Press `Enter` to confirm.
   - Exit nano:
     - Press `Ctrl+X` to exit.

4. **Source the Workspace**:
   - Navigate to your catkin workspace and source the workspace:
     ```bash
     cd ~/catkin_ws
     source devel/setup.bash
     ```

5. **Launch MoveIt!**:
   - Launch the MoveIt! configuration to verify everything is working:
     ```bash
     roslaunch moveit_config3 demo.launch
     ```

## Using PathPlanningV1

### Step 1: Prepare Your Data

1. **Upload Data**:
   - Download the brain parcellation set from the [GitHub repository](https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/tree/main/BrainParcellation).
   - Load the data into 3D Slicer.
   - When the data is loaded, select `Show Options` and ensure to select `brainstem`, `cortex`, `r_hippo`, `r_mtg`, `ventricles`, and `vessels` as label maps.

### Step 2: Set Input Data

1. **Select PathPlanningV1 Module**:
   - In 3D Slicer, navigate to the `PathPlanningV1` module (`Modules` > `Examples`).

2. **Input Parameters**:
   - Select the target region from the dropdown menu (e.g., `r_hippo` in this dataset).
   - Select a critical structure (e.g., `ventricles` or `vessels`in this dataset).
   - Set the entry and target points (e.g., `entries` and `targets`in this dataset).
   - Use the slider to set a length threshold (0 to 200 mm).
   - Select `Create new point list` in Output Fiducials dropdown list.
   - The `Output Fiducials` will contain the target points within the target region.

3. **Generate Trajectory**:
   - Click on `Apply` when all input parameters are set.
   - If the algorithm finds a path that meets all the constraints, it will output a `vtkMRMLMarkupsFiducialNode` called `Trajectory points`, which contains the entry and target points of the selected trajectory.
   - A `vtkMRMLLineNode` called `Trajectory display` will display the path to the user.
   - Additionally, the distance map is also displayed.

### Step 3: Send Data to ROS Using OpenIGTLink

1. **Configure 3D Slicer as the Server**:
   - Open the `OpenIGTLinkIF` module in 3D Slicer (`Modules` >`IGT` > `OpenIGTLinkIF`).
   - Add a new connector and configure it as a server:
     - Set the `Connector Name` to something identifiable, e.g., `SlicerToROS`.
     - Set the `Type` to `Server`.
     - Choose an appropriate port, e.g., `18944`.
     - In the `I/O Configuration` section, add `Trajectory Points` to `Out`.
   - Click `Activate` to start the server.

2. **Configure ROS as the Client**:
   - If you are using a virtual machine to run ROS, make sure you have “NAT” enabled on your network.
   - Open a new terminal on your ROS machine.
   - Navigate to your catkin workspace:
     ```bash
     cd ~/catkin_ws
     ```
   - Source the workspace:
     ```bash
     source devel/setup.bash
     ```
   - Edit the `bridge.launch` file to set the server address and port number to match Slicer's address and port number:
     ```bash
     nano src/ros_igtl_bridge/launch/bridge.launch
     ```
   - Locate and update the parameters for the server address and port number. If you are running ROS on a virtual machine, the server address will typically be `10.0.2.2`. If you have configured your NAT correctly, your real computer's `localhost` should map to `10.0.2.2` on the virtual machine.
     ```xml
     <param name="server_ip" value="10.0.2.2" />
     <param name="server_port" value="18944" />
     ```
   - Save the changes:
     - Press `Ctrl+O` to save.
     - Press `Enter` to confirm.
   - Exit nano:
     - Press `Ctrl+X` to exit.
   - Use the ROS IGTL Bridge to connect to the 3D Slicer server:
     ```bash
     roslaunch ros_igtl_bridge bridge.launch
     ```
   - If prompted, select ROS as client and enter the IP address and port number of the 3D Slicer server.

### Step 4: Launch MoveIt!

1. **Download move_robot_to_pose.py**:
   - Download the `move_robot_to_pose.py` script from the [GitHub directory](https://github.com/Farhana-M/Image-Guided-Robotic-Navigation/tree/main/move_robot_to_pose).

2. **Add to Workspace and Make Executable**:
   - Move the downloaded script to your ROS workspace:
     ```bash
     mv path/to/move_robot_to_pose.py ~/catkin_ws/src/moveit_config3
     ```
   - Make the script executable:
     ```bash
     chmod +x ~/catkin_ws/src/moveit_config3/move_robot_to_pose.py
     ```

3. **Launch roscore**:
   - Open a new terminal, before starting `roscore`, source the ROS Noetic setup file:
     ```bash
     source /opt/ros/noetic/setup.bash
     ```
   - Start the ROS master:
     ```bash
     roscore
     ```

4. **Launch MoveIt!**:
   - Open another terminal(if not already opened) and launch the `moveit_config3` package:
     ```bash
     cd ~/catkin_ws
     ```
   - Source the workspace:
     ```bash
     source devel/setup.bash
     ```
   - Launch the MoveIt! configuration:
     ```bash
     roslaunch moveit_config3 demo.launch
     ```

5. **Run the ROS Node**:
   - Open another terminal and navigate to your catkin workspace:
     ```bash
     cd ~/catkin_ws
     ```
   - Source the workspace:
     ```bash
     source devel/setup.bash
     ```
   - Execute the ROS node to move the robot:
     ```bash
     rosrun moveit_config3 move_robot_to_pose.py
     ```

### Step 5: Send Trajectory Points to ROS

1. **Send Trajectory Points**:
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

