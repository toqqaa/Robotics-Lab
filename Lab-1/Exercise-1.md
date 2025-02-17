# **Lab Exercise: Custom Message with Publisher and Subscriber**

## **Objective:**

1. Create a custom ROS message containing two integers.
2. Write a publisher node that sends two integers.
3. Write a subscriber node that receives the two integers, sums them, and prints the result.

### Step 1 : **Create a new ROS package:**

```bash
cd ~/catkin_ws/src
catkin_create_pkg custom_msg_example rospy std_msgs message_generation message_runtime
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### **Step 2: Create a Custom Message**

1. Navigate to the package folder:

```bash
cd ~/catkin_ws/src/custom_msg_example
```

2. Create a `msg` folder and a custom message file:

```bash
mkdir msg
nano msg/TwoInts.msg
```

3. Define the custom message:

```bash
int32 a
int32 b
```

### **Step 3: Create the Publisher Node**

1. Create scripts folder to add publisher pyhon node

   ```
   mkdir scripts 
   nano scripts/two_ints_publisher.py
   ```
2. Add the following code:

   ```bash
   #!/usr/bin/env python
   import rospy
   from custom_msg_example.msg import TwoInts

   def send_two_ints():
       # Initialize the ROS node
       rospy.init_node('send_two_ints_node', anonymous=True)

       # Create a publisher object that publishes to the 'two_ints_topic' topic
       pub = rospy.Publisher('sum', TwoInts, queue_size=10)

       # Set the publishing rate (1 message per second)
       rate = rospy.Rate(1)

       while not rospy.is_shutdown():
           # Create a custom message
           msg = TwoInts()
           msg.a = 5
           msg.b = 10

           # Publish the message
           rospy.loginfo("Publishing: a = %d, b = %d", msg.a, msg.b)
           pub.publish(msg)

           # Sleep to maintain the publishing rate
           rate.sleep()

   if __name__ == '__main__':
       try:
           send_two_ints()
       except rospy.ROSInterruptException:
           pass
   ```
3. make the file executable

   ```bash
   cd ~/catkin_ws/src/custom_msg_example/scripts
   chmod +x two_ints_publisher.py
   ```

### **Step 4: Create the Subscriber Node**

1. Create subscriber node

   ```bash
   nano scripts/two_ints_subscriber.py
   ```
2. Add the following code:

```python
#!/usr/bin/env python
import rospy
from custom_msg_example.msg import TwoInts

def callback(msg):
    # This function is called whenever a new message is received
    sum_result = msg.a + msg.b
    rospy.loginfo("Received: a = %d, b = %d, Sum = %d", msg.a, msg.b, sum_result)

def subscriber():
    # Initialize the ROS node
    rospy.init_node('two_ints_subscriber', anonymous=True)

    # Create a subscriber object that listens to the 'two_ints_topic' topic
    rospy.Subscriber('sum', TwoInts, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

3. make the file executable:

   ```bash
   cd ~/catkin_ws/src/custom_msg_example/scripts
   chmod +x two_ints_subscriber.py
   ```

### **Step 5: Edit Cmakelist.txt**

add

```cmake
add_message_files(
  FILES
  TwoInts.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_install_python(PROGRAMS
  scripts/two_ints_publisher.py
  scripts/two_ints_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### Build and source the package

```bash
catkin_make
source devel/setup.bash
```

### Start ros master

```bash
roscore
```

### Run the publisher node

```bash
source devel/setup.bash
rosrun custom_msg_example two_ints_publisher.py 
```

### Run Subscriber in another terminal

```bash
source devel/setup.bash
 rosrun custom_msg_example two_ints_subscriber.py 
```

### **To create Launch File**

### **1. Create a `launch` Folder**

Inside your `custom_msg_example` package, create a folder named `launch`:

```bash
cd ~/catkin_ws/src/custom_msg_example
mkdir launch
```

---

### **2. Create a Launch File**

Inside the `launch` folder, create a new file with a `.launch` extension. For example, let’s create a file named `two_ints.launch`:

```bash
cd launch
touch two_ints.launch
```

Now, open the file in a text editor (e.g., `nano`, `gedit`, or `vim`):

```bash
nano two_ints.launch
```

Add the following content to the file:

```bash
<launch>
    <!-- Start the publisher node -->
    <node pkg="custom_msg_example" type="two_ints_publisher.py" name="two_ints_publisher" output="screen" />

    <!-- Start the subscriber node -->
    <node pkg="custom_msg_example" type="two_ints_subscriber.py" name="two_ints_subscriber" output="screen" />
</launch>
```

Run HTML

Save and close the file.

---

### **3. Build the Workspace**

After creating the launch file, rebuild your workspace to ensure everything is up-to-date:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### **4. Run the Launch File**

Now you can run the launch file using the `roslaunch` command:

```bash
roslaunch custom_msg_example two_ints.launch
```

This will:

1. Start the ROS Master (if not already running).
2. Launch the `two_ints_publisher` and `two_ints_subscriber` nodes.
3. Display the output of both nodes in the terminal.
