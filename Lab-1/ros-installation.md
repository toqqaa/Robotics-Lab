# Installation Guide: Ubuntu 20.04 and ROS 1 Noetic

## Install Ubuntu 20.04

There are two ways to install Ubuntu 20.04: **using a Virtual Machine (VM)** or  **setting up a dual-boot system** . Choose the method that best suits your needs


### Option 1: Install Ubuntu on a Virtual Machine

A Virtual Machine (VM) allows you to run Ubuntu alongside your existing operating system without modifying your disk partitions. This is a good option if you want to test Ubuntu or ROS without affecting your current setup.

For a **detailed step-by-step guide** on installing Ubuntu 20.04 on a Virtual Machine, refer to this resource:

[Ubuntu VM Installation Guide](https://github.com/arab-meet/1.Robotics-Tools-Workshop/blob/master/Introduction%20to%20Linux%20and%20ubuntu%20installation/Hands-on%20Ubuntu%20VM%20Installation.md)

### Option 2: Dual-Boot Ubuntu with Your Existing OS

Dual-booting allows you to install Ubuntu alongside your current operating system  (Windows).

For a **detailed step-by-step guide** on dual-booting Ubuntu 20.04, refer to this resource:

[Ubuntu dual-boot installation guide ](https://github.com/arab-meet/1.Robotics-Tools-Workshop/blob/master/Introduction%20to%20Linux%20and%20ubuntu%20installation/Introduction%20to%20Linux%20and%20ubuntu%20installation.md)


## Set Up Ubuntu for ROS Installation

### Step 1: Update and Upgrade System

Open a terminal and run the following command to ensure your system is up to date:

```bash
sudo apt update
```

### Step 2: Enable Required Repositories

ROS Noetic requires the `universe`, `multiverse`, and `restricted` repositories to be enabled. These are usually enabled by default, but you can verify and enable them with the following commands:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update
```

## Install ROS 1 Noetic

### Step 1: Set Up ROS Sources

1. Add the ROS Noetic repository to your system:

   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```
2. Add the ROS key to authenticate packages:

   ```bash
   sudo apt install curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```


3. Update the package list:

   ```bash
   sudo apt update
   ```


### Step 2: Install ROS Noetic

1.  Install ROS Dependencies

Install additional tools and dependencies for building ROS packages:

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-catkin-tools
```


### Step 4: Initialize rosdep

1. Initialize `rosdep`:

   ```bash
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```


## Configure ROS Environment

### Step 1: Source ROS Setup Script

To use ROS, you need to source the setup script in every new terminal session. Add the following line to your `~/.bashrc` file to automate this:


## Verify ROS Installation

### Step 1: Test ROS Installation

1. Open a new terminal and run the following command to start the ROS master:

   ```bash
   roscore
   ```
2. In another terminal, run the following command to list active ROS topics:

   ```bash
   rostopic list
   ```

If you see output like `/rosout` and `/rosout_agg`, ROS is installed correctly.
