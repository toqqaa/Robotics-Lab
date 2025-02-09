
### **Task: Draw the First Letter of Your Name with Turtlesim!**

#### **Scenario: "Turtle Letter Drawer"**

Your Turtlesim turtle will receive a **single-letter command** and will move to  **draw that letter** .

---

### **🔹 Task Requirements**

✅ **Publisher Node (C++ - Letter Sender)**

* Publishes **a single character** (e.g., `"A"`, `"S"`, `"M"`, etc.) on a topic **`/turtle_letter`** using `std_msgs/String`.
* The letter represents the first letter of your name.

✅ **Subscriber Node (C++ - Letter Drawer)**

* Subscribes to **`/turtle_letter`** and converts the letter into  **predefined movement steps** .
* Moves the turtle in **Turtlesim** to draw the letter.
* Publishes velocity commands to **`/turtle1/cmd_vel`** using  **`geometry_msgs/Twist`** .

✅ **Launch File:**

* Automates starting both nodes and  **Turtlesim** .
