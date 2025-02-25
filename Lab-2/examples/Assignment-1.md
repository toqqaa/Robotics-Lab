# **📝 Assignment: Custom ROS Publisher & Subscriber for TurtleBot3**

### **Objective**

In this assignment, you will:

1. Create a **custom ROS message** to send commands (e.g., direction and speed) to the TurtleBot3.
2. Write a **publisher node** that takes user input (direction and speed) and publishes it as a custom message.
3. Write a **subscriber node** that listens to the custom message and controls the TurtleBot3 by sending velocity commands to the `/cmd_vel` topic.
4. Test the system in a **Gazebo simulation** and observe the TurtleBot3 moving based on the commands.

---

## **📂 ROS Package Details**

* **Package Name** : `turtlebot3_custom_control`
* **Custom Message Name** : `Command.msg`
* **Topics** :
* `/turtle_control` (published by your publisher)
* `/cmd_vel` (published by your subscriber to control TurtleBot3)
* **Node Names** :
* `turtle_control_publisher` (publisher node)
* `turtle_control_subscriber` (subscriber node)

---

## **🛠 Prerequisites**

Before starting, make sure you have:

* **ROS Noetic installed** .
* **TurtleBot3 packages installed** :

```bash
sudo apt install ros-noetic-turtlebot3*
```

* **Your ROS workspace set up** (`~/catkin_ws`).

---

## **🚀 Steps to Follow**

### **1️⃣ Create the ROS Package**

* Create a package named `turtlebot3_custom_control`.
* Add the required dependencies.

  ```bash
  cd ~/catkin_ws/src
  catkin_create_pkg turtlebot3_custom_control rospy std_msgs geometry_msgs message_generation message_runtime
  ```

### **2️⃣ Create a Custom Message**

* Create a new ROS message type called `Command.msg`.
* The message should contain two fields:

  * `direction` (string): The direction of movement (e.g., "forward", "backward", "left", "right").
  * `speed` (float32): The speed of movement (e.g., 0.1 to 1.0).
* 
* Modify `CMakeLists.txt` and `package.xml` to support custom messages.

### **3️⃣ Implement the Publisher**

1. **Create a Publisher Node** :

* Write a Python script (`turtle_control_publisher.py`) that:
  * Initializes a ROS node.
  * Creates a publisher to send the custom message to a topic (e.g., `/turtlebot3_command`).
  * Takes user input for direction and speed.
  * Publishes the custom message at a fixed rate.

2. **Make the Script Executable**

### **4️⃣ Implement the Subscriber**

1. **Create a Subscriber Node** :

* Write a Python script (`turtle_control_subscriber`.py) that:
  * Initializes a ROS node.
  * Subscribes to the custom message topic (`/turtlebot3_command`).
  * Based on the received message, sends velocity commands to the `/cmd_vel` topic to control the TurtleBot3.

2. **Make the Script Executable**

### **5️⃣ Run the TurtleBot3 Simulation**

* Before launching,

  1. **export the TurtleBot3 model** :

  ```bash
  export TURTLEBOT3_MODEL=burger
  ```

2. Launch the simulation:

   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
   ```

### **6️⃣ Run Your Nodes**

* **Run the publisher node** .
* **Run the subscriber node** .

### **7️⃣ Verify the Topics**

To check if messages are being published:

```bash
rostopic echo /turtlebot3_command)
```

To check if TurtleBot3 is moving:

```bash
rostopic echo /cmd_vel
```

---

## **📌 Submission Requirements**

* A working  **ROS package**  (.zip).
* A properly defined  **custom message** .
* Publisher and subscriber scripts.
* Proof of TurtleBot3 movement (short video).
