Follow these steps For next lab :

### 1. Set Up the Workspace and Clone the Repository

1. Navigate to your `catkin_ws/src` directory:

   ```bash
   cd ~/catkin_ws/src
   ```
2. Clone the TurtleBot3 simulations repository (Noetic branch):

   ```bash
   git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
   ```

### 2. Modify the Launch File

1. Open the `turtlebot3_world.launch` file for editing:

   ```bash
   nano turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch
   ```
2. Add the following lines to the file (usually before the closing `</launch>` tag):

   ```xml
   <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>

   <!-- Send fake joint values-->
   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
     <param name="use_gui" value="true"/>
   </node>
   ```
3. Save and exit the editor (`Ctrl+O`, `Enter`, `Ctrl+X`).

### 3. Build the Workspace

1. Build the workspace using `catkin_make`:

   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
2. Source the workspace:

   ```bash
   source devel/setup.bash
   ```

### 4. Set the TurtleBot3 Model

Set the TurtleBot3 model to `waffle` (or `burger` if you prefer):

```bash
export TURTLEBOT3_MODEL=waffle
```

### 5. Launch the Gazebo Simulation

Launch the TurtleBot3 world in Gazebo:

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### 6. Install the `gmapping` Package

If you haven't already installed the `gmapping` package, install it using `apt`:

```bash
sudo apt-get install ros-noetic-slam-gmapping
```
