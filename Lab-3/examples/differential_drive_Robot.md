# Creating a Xacro File for Your Robot in ROS

## 1. Create a ROS Package

To organize your URDF/Xacro files properly, first, create a ROS package:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot 
cd ~/catkin_ws
catkin_make
```

2. Create a Directory for URDF Files

Inside your package, create a directory to store your URDF/Xacro files:

```bash
cd ~/catkin_ws/src/my_robot
mkdir urdf
```

## 3. Create a `.xacro` File

Navigate to the `urdf` directory and create a new file called  `myrobot.xacro`:

```bash
cd urdf
touch myrobot.xacro
```

## 4. Structure of the `myrobot.xacro` File

Below is an explanation of each part of the Xacro file.

### 4.1 Declaring the Robot Name and Xacro Namespace

At the beginning of the file, declare the XML version and define the Xacro namespace.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="myrobot">
```

### 4.2 Defining the Base Link

The base link represents the main body of the robot.

```xml
  <link name="base_link">
      <inertial>
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <mass value="5.0"/>
          <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <geometry>
             <cylinder radius="0.25" length="0.15"/>
          </geometry>
          <material name="red">
              <color rgba="1.0 0.0 0.0 1.0"/>
          </material>
      </visual>
      <collision>
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <geometry>
             <cylinder radius="0.5" length="0.25"/>
          </geometry>
      </collision>
  </link>
```

* **`<inertial>`** : Defines the mass and inertia properties.
* **`<visual>`** : Specifies how the robot will appear in visualization tools like RViz.
* **`<collision>`** : Defines the collision model used for physics simulations.

### 4.3 Defining the Left and Right Wheels

Each wheel is defined as a separate link.

#### Left Wheel:

```xml
  <link name="left_wheel">
      <inertial>
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <mass value="1.0"/>
          <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
      <visual>
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <geometry>
              <cylinder radius="0.1" length="0.07"/>
          </geometry>
          <material name="black">
              <color rgba="0.0 0.0 0.0 1.0"/>
          </material>
      </visual>
      <collision>
          <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <geometry>
              <cylinder radius="0.1" length="0.05"/>
          </geometry>
      </collision>
  </link>
```

####  **right wheel** 

```xml
<link name="right_wheel">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <mass value="1.0"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual name="right_wheel_visual">
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
          <cylinder radius="0.1" length="0.07"/>
        </geometry>
        <material name="black">
            <color rgba="0.0 0.0 0.0 1.0"/>
            <texture filename=""/>
        </material>
    </visual>
    <collision>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
           <cylinder radius="0.1" length="0.05"/>
        </geometry>
    </collision>
</link>
```

#### Defining Joints for the Wheels:

```xml
  <joint name="left_wheel_joint" type="continuous">
      <origin xyz="0.065 0.15 -0.05" rpy="0.0 1.5707 1.5707"/>
      <parent link="base_link"/>
      <child link="left_wheel"/>
      <axis xyz="0.0 0.0 1.0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
      <origin xyz="0.065 -0.15 -0.05" rpy="0.0 1.5707 1.5707"/>
      <parent link="base_link"/>
      <child link="right_wheel"/>
      <axis xyz="0.0 0.0 1.0"/>
  </joint>
```

* The **joints** define the connection between the base link and the wheels.

### 4.4 Adding a Caster Wheel

A **caster wheel** helps balance the robot.

```xml
  <link name="caster_wheel">
      <inertial>
          <mass value="0.2"/>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
      <visual>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry>
              <sphere radius="0.05"/>
          </geometry>
          <material name="gray">
              <color rgba="0.5 0.5 0.5 1"/>
          </material>
      </visual>
      <collision>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry>
              <sphere radius="0.05"/>
          </geometry>
      </collision>
  </link>

  <joint name="caster_wheel_joint" type="fixed">
      <parent link="base_link"/>
      <child link="caster_wheel"/>
      <origin xyz="-0.15 0 -0.1" rpy="0 0 0"/>
  </joint>
```

* The caster wheel is **fixed** to the base.

### 4.5 Closing the Robot Tag

At the end of the file, close the `<robot>` tag.

```xml
</robot>
```
