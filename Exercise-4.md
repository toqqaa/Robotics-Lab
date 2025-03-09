### **Exercise: Adding a Camera to Your Robot**

#### **Objective:**

In this exercise, you will add a camera to your robot model using the `robot_description` package located in the `lab4` folder. You will:

1. Create a `camera_link` in the `myrobot.xacro` file.
2. Use an STL, STEP, or DAE file for the camera in the `meshes` folder.
3. Add an RGB camera plugin in the `.gazebo` file.
4. Visualize the camera data in **RViz** and take a screenshot.

---

### **Steps:**

#### **1. Create a `camera_link` in `myrobot.xacro`**

* Open the `myrobot.xacro` file in the `robot_description` package located in the `lab4` folder.
* Define a new link for the camera. Include:
  * A visual component with a mesh file for the camera (STL, STEP, or DAE format).
  * A collision component (use a simple box for this).
  * An inertial component (define mass and inertia properties).
* Attach the `camera_link` to an existing joint (e.g., `base_link`) 

---

#### **2. Add a Camera Mesh File**

* Search online for a camera model in  **STL, STEP, or DAE format** . You can find free models on websites like [GrabCAD](https://grabcad.com/)
* Download the file and place it in the `meshes` folder of the `robot_description` package. 
* Update the `<mesh>` tag in the `camera_link` definition to point to the downloaded file.

---

#### **3. Add an RGB Camera Plugin in `.gazebo`**

* Open the `.gazebo` file associated with your robot (e.g., `myrobot.gazebo`).
* Add an RGB camera plugin for the `camera_link`. Configure:
  * The camera's field of view, resolution, and update rate.
  * The ROS topics for publishing image data (e.g., `/camera/image_raw`).
* Refer to the [Gazebo ROS Plugins Documentation](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins) for guidance on setting up the plugin.

---

#### **4. Test Your Camera in RViz**

* Take a screenshot of the camera data in RViz.


### **Deliverables:**

1. Updated `myrobot.xacro` file with the `camera_link` and joint.
2. Camera mesh file (STL, STEP, or DAE) in the `meshes` folder of the `robot_description` package.
3. Updated `.gazebo` file with the RGB camera plugin.
4. Screenshot of the camera data (`/camera/image_raw`) in RViz.
