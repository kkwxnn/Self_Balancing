# Self Balancing Project : FRA501 Robotics DevOps
โครงงานนี้เป็นส่วนหนึ่งของวิชา FRA501 Robotics DevOps ของนักศึกษาชั้นปีที่ 3 สถาบันวิทยาการหุ่นยนต์ภาคสนาม (FIBO) เพื่อ Simulate ระบบการทำงานของหุ่นยนต์ทรงตัว (Self-Balancing Robot) ที่มี Reaction Wheel จำนวน 2 ล้อ

## **Objective**
1) เพื่อสร้าง Simulation การทำงานของหุ่นยนต์ใน Gazebo
2) เพื่อเรียนรู้วิธีการ Simulate Environment ที่ใช้ในการทำ Simulation การทำงานของหุ่นยนต์

## **System Architecture**
![Selfbalancing drawio (1)](https://github.com/kkwxnn/Self_Balancing/assets/122891621/66a10d95-7333-4e2d-86d5-d92942cc946f)

## **Installation**
Step 1: Download zip file and unzip file.

Step 2: Go to the src directory and copy the "Self Balancing" package to your src directory in your workspace.

Step 3: Build "Self Balancing" in your workspace.
```
cd ~/[your_workspace]
colcon build 
source install/setup.bash
```
## Testing out Self Balancing
Terminal1: run launch file

```
cd ~/[your_workspace]
ros2 launch  
```

