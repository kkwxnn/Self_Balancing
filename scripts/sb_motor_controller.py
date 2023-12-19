#!/usr/bin/python3

# from motor_sb.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import numpy as np
import sys
import math
import time

class Motor_Velo_control(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.pub_float64 = self.create_publisher(Float64MultiArray,"/velocity_controllers/commands",10)
        self.create_timer(0.01,self.timer_callback)
        # self.create_timer(0.01,self.Controller())
        self.motor_velo_cur = [0.0,0.0]
        self.is_change_point = False
        self.timestamp = 0
        self.i = 0
        self.create_subscription(Imu, "/imu_plugin/out", self.Imu_sensor_callback, 10)
        self.Imu_orientation = [0.0, 0.0, 0.0, 0.0]
        self.Imu_angular_velocity = [0.0, 0.0, 0.0]
        self.Imu_linear_acceleration = [0.0, 0.0, 0.0]
        self.isEnableController = False
        self.Kp_x = 1000.0
        self.Kp_y = 1000.0

        self.Ki_x = 0.0
        self.Ki_y = 0.0

        self.Kd_x = 200.0
        self.Kd_y = 200.0

        self.rotation = [0.0, 0.0]

        self.x_orientation_error = 0.0
        self.y_orientation_error = 0.0
        self.sum_error_x = 0.0
        self.sum_error_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        # self.motor_velo_1 = [5.0,5.0]
        # self.motor_velo_2 = [-5.0,-5.0]

    def Imu_sensor_callback(self,msg):
        self.Imu_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.Imu_angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.Imu_linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.isEnableController = True
        self.Controller()
        # self.rotation = self.quaternion_to_euler(self.Imu_orientation)
        # self.x_orientation_error =  self.rotation[2] + 0.0177 # rad
        # self.y_orientation_error = 0.0177 + self.rotation[1] 
        # # self.y_orientation_error = 0.00052 - rotation[1] # rad
        # # self.x_orientation_error = 0.00021 - rotation[2] 

        # if self.rotation[1] <= 0.0177 + 0.00177 and self.rotation[1] >= 0.0177 - 0.00177:
        #     self.sum_error_y = 0.0
        # if self.rotation[2] <= 0.0177 + 0.00177 and self.rotation[2] >= 0.0177 - 0.00177:
        #     self.sum_error_x = 0.0

        # print(self.Imu_orientation)
        # print(self.Imu_angular_velocity)
        # print(self.Imu_linear_acceleration)
        
    def quaternion_to_euler(self,q):
            (x, y, z, w) = (q[0], q[1], q[2], q[3])
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(t0, t1)
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch = math.asin(t2)
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(t3, t4)
            return [yaw, pitch, roll]
    
    def Controller(self):
        if self.isEnableController == True :
            self.rotation = self.quaternion_to_euler(self.Imu_orientation)
            self.x_orientation_error = 0.0 + self.rotation[2]  # rad
            self.y_orientation_error = 0.0 + self.rotation[1] 

            # if abs(self.y_orientation_error) < 0.0144 :
            #     self.sum_error_y = 0.0
            # if abs(self.x_orientation_error) < 0.0144 :
            #     self.sum_error_x = 0.0

            ## Goal Reach ##
            # if self.sum_error_x == 0 and self.sum_error_y == 0:
            #     self.motor_velo_cur = [0.0,0.0]
                
            #     msg_velo = Float64MultiArray()
            #     msg_velo.data = self.motor_velo_cur
            #     self.pub_float64.publish(msg_velo)

            # Kp
            P_X = self.x_orientation_error*self.Kp_x
            P_Y = self.y_orientation_error*self.Kp_y
            # Ki
            self.sum_error_x += self.x_orientation_error
            self.sum_error_y += self.y_orientation_error
            I_X = self.sum_error_x*self.Ki_x
            I_Y = self.sum_error_y*self.Ki_y
            # Kd
            D_X = (self.x_orientation_error - self.prev_error_x) * self.Kd_x
            D_Y = (self.y_orientation_error - self.prev_error_y) * self.Kd_y
            # D_X = joint1_velo * self.Kd_x
            # D_Y = joint0_velo * self.Kd_y
            self.prev_error_x = self.x_orientation_error
            self.prev_error_y = self.y_orientation_error

            joint0_velo = P_Y + I_Y + D_Y
            joint1_velo = P_X + I_X + D_X

            maxVel = 1500.0
            if joint0_velo >= maxVel:
                joint0_velo = maxVel
            elif joint0_velo <= -maxVel:
                joint0_velo = -maxVel
            
            if joint1_velo >= maxVel:
                joint1_velo = maxVel
            elif joint1_velo <= -maxVel:
                joint1_velo = -maxVel

            self.motor_velo_cur = [round(joint0_velo,2),round(joint1_velo,2)]
            
            msg_velo = Float64MultiArray()
            msg_velo.data = self.motor_velo_cur
            self.pub_float64.publish(msg_velo)
            self.isEnableController = False

    def timer_callback(self):
        pass
        # self.timestamp += 0.01
        # joint 0 y
        # joint 1 x
       
        

def main(args=None):
    rclpy.init(args=args)
    node = Motor_Velo_control()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__=='__main__':
    main()
