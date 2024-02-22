# FRA501 Robotics DevOps: Self-Balancing Project 
This project is part of the FRA501 Robotics DevOps course for third-year students at the Institute of Field Robotics (FIBO), King Mongkut’s University of Technology Thonburi (KMUTT) to simulate the work system of a **Self-Balancing Robot** with **2 reaction wheels** on Gazebo program with ROS2. 

[Screencast from 12-20-2023 05:42:58 AM.webm](https://github.com/kkwxnn/self_balancing/assets/122891621/9ce6314e-1718-485a-aff3-34d2ca084544)




## **Objective**
1) To simulate the work system of a **Self-Balancing Robot** with 2 reaction wheels on Gazebo program with ROS2.
2) To learn how to set up the environment for simulating robot operations.

## **Package**

![sb drawio](https://github.com/kkwxnn/self_balancing/assets/122891621/83433a7b-9782-4744-9514-3532ae13acd2)


### **1. Config**
The directory that contains a parameter file that defines the properties of the model. You can adjust the values in the parameter file and observe how the self-balancing model behaves.
- **kinematics_parameters.yaml**
  
  The parameters file contains the position and orientation of the coordinated frame in the model, describing the pose of the links in the model. In the default of this package, I have defined the kinematic parameter values of the model, referring to the model’s frame in SOLIDWORKS.

Example: 

  ![image](https://github.com/kkwxnn/self_balancing/assets/122667170/b2cdd75f-115d-46f3-871b-85cae84bd00d)
```yaml
base_link:
  orientation: 0.0 0.0 0.0 # Unit in radian 
  position: 0.0 0.0 0.0 # Unit in meter 
link_1: 
  orientation: 1.5708 0.0 0.0
  position: -0.03655 -0.04386 0.2348
link_2: 
  orientation: 0.0 1.5708 0.0
  position: 0.04386 0.03655 0.2348
```
- **dynamics_parameters.yaml**

  The parameters file contains physical properties, including mass, center of mass (COM), and inertia for each link in the model. In the default of this package, I have defined the dynamic parameter values of the model, referencing the mass properties in SOLIDWORKS. It's important to note that the model's value needs to be referenced from the major coordinate of each link.

Example:

  ![image](https://github.com/kkwxnn/self_balancing/assets/122667170/5fd3b383-223b-4d04-af9a-5d0e93db0a74)
```yaml
base_link:
  mass: 0.543326 # Unit in kilograms 
  com: -0.012639 0.012676 0.129321 # Unit in meter 
  inertia: # Unit in kilograms*meter^2 
    xx: 0.003732
    yy: 0.003731
    zz: 0.000287
    xy: 0.000006
    xz: 0.000068
    yz: -0.000066
link_1: 
  mass: 0.482692
  com: 0.0 0.0 0.0
  inertia:
    xx: 0.001086
    yy: 0.001086
    zz: 0.002120
    xy: 0.000000
    xz: 0.000000
    yz: 0.000000
link_2: 
  mass: 0.482692
  com: 0.0 0.0 0.0
  inertia:
    xx: 0.001086
    yy: 0.001086
    zz: 0.002120
    xy: 0.000000
    xz: 0.000000
    yz: 0.000000
```
- **visual_parameters.yaml**

  The parameters file includes the position and orientation of the model within the visualization section, referencing the kinematic frame.
  
- **collision_parameters.yaml**

  The parameters file includes the position and orientation of the model within the collision section, describing the hit box of the model. The collision frame is referenced from the kinematic frame.
  
- **controller_parameters.yaml**

  The file describes the controller type and specifies the joints we intend to control. Moreover, it can set the update rate and interface of the controller. The format of the controller is explained in the 'controller_manager' package.

  Example:
```yaml
controller_manager: # Package Name 
  ros__parameters:
    update_rate: 50 # Hz

    velocity_controllers: # Type of controller 
      type: velocity_controllers/JointGroupVelocityController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
      
velocity_controllers:
  ros__parameters:
    joints: # Define the joint that you want to control 
      - joint0 # Joint's name must be defined in the robot model 
      - joint1
    command_interface: 
      - velocity
    state_interface: 
      - velocity
      - effort
```
### **2. sb_robot**

  In this directory, there is a file named file.xacro used to construct the robot model. Each file in this directory undergoes a distinct process, making it easier to adjust in the future.
- **properties.xacro**

  The file that will get the parameters from the config file as a variable for constructing the robot model in other files.

  Example:
```xacro
  <!-- File paths-->
  <xacro:property name="kinematics_params_path" value="$(find ${description_pkg})/config/kinematics_parameters.yaml"/> 
  <xacro:property name="visual_params_path" value="$(find ${description_pkg})/config/visual_parameters.yaml"/> 
  <xacro:property name="collision_params_path" value="$(find ${description_pkg})/config/collision_parameters.yaml"/> 
  <xacro:property name="dynamics_params_path" value="$(find ${description_pkg})/config/dynamics_parameters.yaml"/> 
  
  <xacro:property name="kp" value="${xacro.load_yaml(kinematics_params_path)}"/> 
  <xacro:property name="vp" value="${xacro.load_yaml(visual_params_path)}"/> 
  <xacro:property name="cp" value="${xacro.load_yaml(collision_params_path)}"/> 
  <xacro:property name="dp" value="${xacro.load_yaml(dynamics_params_path)}"/>
```
  
- **manipulator.xacro**

  This file will use the variables that are defined in properties.xacro for constructing the visual, collision, and inertial of a link model and joint in Gazebo.

  Example:
```xacro
  # Define link
  <xacro:macro name="link1_visual" params=""> # visual
        <visual>
            <origin xyz="${LINK1_VISUAL['position']}" rpy="${LINK1_VISUAL['orientation']}"/>
            <geometry>
                <mesh filename="file://$(find ${description_pkg})/meshes/Wheel.stl"/>
            </geometry>
            <material name="link_color">
                <color rgba="${LINK1_VISUAL['color']}"/>
            </material>
        </visual>
    </xacro:macro>

    <xacro:macro name="link1_inertial" params=""> # inertial
        <inertial>
            <origin xyz="${LINK1_INERTIAL['com']}"/>
            <mass value="${LINK1_INERTIAL['mass']}"/>
            <inertia 
                ixx="${LINK1_INERTIAL['inertia']['xx']}" 
                ixy="${-LINK1_INERTIAL['inertia']['xy']}" 
                ixz="${-LINK1_INERTIAL['inertia']['xz']}" 
                iyy="${LINK1_INERTIAL['inertia']['yy']}" 
                iyz="${-LINK1_INERTIAL['inertia']['yz']}" 
                izz="${LINK1_INERTIAL['inertia']['zz']}"/>
        </inertial>
    </xacro:macro>

    <xacro:macro name="link1_collision" params=""> # collision
        <collision>
            <origin xyz="${LINK1_COLLISION['position']}" rpy="${LINK1_COLLISION['orientation']}"/>
            <geometry>
                <mesh filename="file://$(find ${description_pkg})/meshes/Wheel.stl"/>
            </geometry>
        </collision>
    </xacro:macro>

    # Define joint
    <xacro:macro name="joint0" params="">
        <joint name="joint0" type="continuous">
            <parent link="base_link"/>
            <child link="link_1"/>
            <origin xyz="${LINK1['position']}" rpy="${LINK1['orientation']}"/>
            <axis xyz="0 0 1"/>
        </joint>
    </xacro:macro>
```

- **motor_controller.gazebo.xacro**

  Use the control plugin from "libgazebo_ros2_control.so" and to define the joint interface types.

    Example:
```xacro
    # Define joint interface 
    <joint name="joint0">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
```
  
- **imusensor.xacro**

  Use the imu sensor plugin from "libgazebo_ros_imu_sensor.so" and to define where to reference the sensor in the model.
  
- **robot.xacro**

  To gather all of xacro files (properties, manipulator, motor_controller.gazebo, imusensor) for use in the launch file.

### **3. script**
- **sb_motor_controller.py**
  - **Imu_sensor_callback(msg)**: This function gets orientation, angular velocity, and linear acceleration from the Inertial Measurement Unit (IMU) sensor.
  - **quaternion_to_euler(q)**: This function is for converting quaternion, which is a 4-element vector to Euler angles, which are three angles that represent the rotation about the intrinsic axes (roll, pitch, yaw).
  - **Controller()**: This function controls motor velocity to be at an orientation that makes the center of mass (CM) be above the Pivot Point utilizing PID control from this equation, And the velocity of each joint will be published to the velocity controller. (Joint 0 is for the control orientation of X-axes and Joint 1 is for the control orientation of Y-axes)
    
![image](https://github.com/kkwxnn/self_balancing/assets/122891621/344370de-de5e-4966-80ab-8bdc8c1b7cca)

```python
class Motor_Velo_control(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        ...
  # Tuning Kp Ki Kd here
        self.Kp_x = 50000.0
        self.Kp_y = 50000.0

        self.Ki_x = 0.0
        self.Ki_y = 0.0

        self.Kd_x = 10000.0
        self.Kd_y = 10000.0
        ...
```

```python
    def Controller(self):
    ...
            # Set goal here
            self.x_orientation_error = -0.0018151424 + self.rotation[2] 
            self.y_orientation_error = self.rotation[1] + 0.0018849556
    ...
```

### **4. worlds**
- **sample.world**

  Is a file that describes the gazebo’s world properties such as Sun movement and Friction. In this project, we focus on the friction properties of the ground model because it highly affects the self-balancing robot behavior. If the friction is too small, It will make the model fail easily.

```xacro
<model name='ground_plane'>
  <static>1</static>
        <link name='link'>
        ...
        ...
          <friction>
              <ode>
                <mu>1000</mu> # set the friction coefficient here 
                <mu2>1000</mu2>     
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
        ...
        ...
</model>
```

### **5. launch**
- **sb_robot_spawn_launch.py**

  The launch file 'sb_robot_spawn_launch.py' initiates every node and command necessary to start the model simulation and spawn it in Gazebo. In this project, the simulation construction requires the installation and execution of dependency packages, including robot_state_publisher, gazebo_ros, and controller_manager. Additionally, another file essential for execution is our controller, defined in the script directory.




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

