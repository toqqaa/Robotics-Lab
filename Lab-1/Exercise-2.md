# Exercise 2 : publisher and Subscriber c++ Example


## **Step 1: Create a ROS Package**

1. Navigate to your `catkin_ws/src` folder:

   ```bash
   cd ~/catkin_ws/src
   ```
2. Create a new ROS package:

   ```bash
   catkin_create_pkg cpp_talker_listener roscpp std_msgs
   ```
3. Build the workspace:

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

---

## **Step 2: Create the Talker (Publisher) Node**

1. Navigate to the package folder:

   ```bash
   cd ~/catkin_ws/src/cpp_talker_listener
   ```
2. Create a `src` folder (if it doesn’t exist) and add the talker node:

   ```bash
   mkdir -p src
   nano src/talker.cpp
   ```
3. Add the following code to `talker.cpp`:

   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   #include <sstream>

   int main(int argc, char **argv) {
       // Initialize the ROS node
       ros::init(argc, argv, "talker");

       // Create a NodeHandle object
       ros::NodeHandle nh;

       // Create a Publisher object that publishes to the "chatter" topic
       ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

       // Set the publishing rate (10 messages per second)
       ros::Rate loop_rate(10);

       int count = 0;
       while (ros::ok()) {
           // Create a message object
           std_msgs::String msg;

           // Set the message data
           std::stringstream ss;
           ss << "Hello World " << count;
           msg.data = ss.str();

           // Publish the message
           ROS_INFO("Publishing: %s", msg.data.c_str());
           chatter_pub.publish(msg);

           // Spin once and sleep to maintain the publishing rate
           ros::spinOnce();
           loop_rate.sleep();
           ++count;
       }

       return 0;
   }
   ```

---

## **Step 3: Create the Listener (Subscriber) Node**

1. In the same `src` folder, create the listener node:
   

   ```bash
   nano src/listener.cpp
   ```
2. Add the following code to `listener.cpp`:


   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"

   void chatterCallback(const std_msgs::String::ConstPtr& msg) {
       // This function is called whenever a new message is received
       ROS_INFO("I heard: %s", msg->data.c_str());
   }

   int main(int argc, char **argv) {
       // Initialize the ROS node
       ros::init(argc, argv, "listener");

       // Create a NodeHandle object
       ros::NodeHandle nh;

       // Create a Subscriber object that listens to the "chatter" topic
       ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

       // Keep the node running
       ros::spin();

       return 0;
   }
   ```

---

## **Step 4: Update `CMakeLists.txt`**

1. Open the `CMakeLists.txt` file:

   ```
   nano CMakeLists.txt
   ```
2. update cmake

   ```cmake
   add_executable(talker src/talker.cpp)
   target_link_libraries(talker ${catkin_LIBRARIES})

   add_executable(listener src/listener.cpp)
   target_link_libraries(listener ${catkin_LIBRARIES})
   ```

   #### For Better understand why we need to update cmake for c++ node

   To compile your C++ nodes (`talker.cpp` and `listener.cpp`), you need to add them as **executables** in the `CMakeLists.txt` file.

   #### Add the `talker` Node:


   ```cmake
   add_executable(talker src/talker.cpp)
   ```

   * `add_executable(talker src/talker.cpp)`:
     * `talker`: The name of the executable (you can name it anything).
     * `src/talker.cpp`: The path to the source file.

   #### Add the `listener` Node:

   ```cmake
   add_executable(listener src/listener.cpp)
   ```

   * `add_executable(listener src/listener.cpp)`:
     * `listener`: The name of the executable.
     * `src/listener.cpp`: The path to the source file.

   ---

   ### **Link Libraries**

   After adding the executables, you need to link them to the required libraries (e.g., `roscpp`, `std_msgs`). This is done using the `target_link_libraries` command.

   #### Link Libraries for `talker`:

   ```cmake
   target_link_libraries(talker ${catkin_LIBRARIES})
   ```

   * `target_link_libraries(talker ${catkin_LIBRARIES})`:
     * `talker`: The name of the executable.
     * `${catkin_LIBRARIES}`: A CMake variable that includes all the libraries specified in `find_package`.

   #### Link Libraries for `listener`:

   ```cmake
   target_link_libraries(listener ${catkin_LIBRARIES})
   ```


## **Step 5: Build the Package**

1. Build the workspace:


   ```
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

---

## **Step 6: Create a Launch File**

1. Create a `launch` folder in your package:

   ```bash
   cd ~/catkin_ws/src/cpp_talker_listener
   mkdir launch
   ```
2. Create a launch file:

   ```bash
   nano launch/talker_listener.launch
   ```
3. Add the following content to the launch file:

   ```xml
   <launch>
       <!-- Start the talker node -->
       <node pkg="cpp_talker_listener" type="talker" name="talker" output="screen" />

       <!-- Start the listener node -->
       <node pkg="cpp_talker_listener" type="listener" name="listener" output="screen" />
   </launch>
   ```

   Save and close the file.

---

## **Step 7: Run the Launch File**

1. Run the launch file:

   ```bash
   roslaunch cpp_talker_listener talker_listener.launch
   ```

   This will start both the `talker` and `listener` nodes, and you should see the following output in the terminal:

   ```
   [INFO] [WallTime: ...] Publishing: Hello World 0
   [INFO] [WallTime: ...] I heard: Hello World 0
   [INFO] [WallTime: ...] Publishing: Hello World 1
   [INFO] [WallTime: ...] I heard: Hello World 1
   ```

---

## **Folder Structure**

Your package structure should now look like this:

```
cpp_talker_listener/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── talker_listener.launch
├── src/
│   ├── talker.cpp
│   └── listener.cpp
└── include/
```

---
