# Self-Balancing Project : FRA501 Robotics DevOps
This project is part of the FRA501 Robotics DevOps course of third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT) to simulate the work system of a **Self-Balancing Robot** with **2 reaction wheels** on Gazebo program with ROS2. 

[Screencast from 12-20-2023 05:42:58 AM.webm](https://github.com/kkwxnn/self_balancing/assets/122891621/9ce6314e-1718-485a-aff3-34d2ca084544)




## **Objective**
1) To simulate the work system of a **Self-Balancing Robot** with 2 reaction wheels on Gazebo program with ROS2.
2) To learn how to setup the environment for simulating robot operations.

## **Package**

![sb drawio](https://github.com/kkwxnn/self_balancing/assets/122891621/83433a7b-9782-4744-9514-3532ae13acd2)


### **Config**
- collision_parameters.yaml
- controller_parameters.yaml
- dynamics_parameters.yaml
- kinematics_parameters.yaml
- visual_parameters.yaml

### **sb_robot**
- properties.xacro
- manipulator.xacro
- robot.xacro
- motor_controller.gazebo.xacro
- imusensor.xacro

### **script**
- sb_motor_controller.py
      - This file is for controlling motor velocity utilizing PID control from this equation
![image](https://github.com/kkwxnn/self_balancing/assets/122891621/344370de-de5e-4966-80ab-8bdc8c1b7cca)


### **worlds**
- sample.world

### **launch**
- sb_robot_spawn_launch.py



## **Installation**
Step 1: Clone the repository to the src directory of your workspace. You must unzip and put each folder in the src directory. 
```
cd ~/[your_workspace]/src
git clone https://github.com/kkwxnn/self_balancing.git
```

Step 2: Run rosdep install to install dependencies
```
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Step 3: Build "self_balancing" in your workspace.
```
cd ~/[your_workspace]
colcon build 
source install/setup.bash
```

## **Testing out "self_balancing"**
Run launch file in your workspace

```
ros2 launch self_balancing sb_robot_spawn_launch.py
```

## **Our Team**
1) Monsicha Sopitlaptana 64340500071
2) Peerawat Santifuengkul 64340500043

