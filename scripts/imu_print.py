#!/usr/bin/python3

from self_balancing.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import matplotlib.pyplot as plt


class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.create_subscription(Imu, "/imu_plugin/out", self.Imu_sensor_callback, 10)
        self.Imu_orientation = [0.0, 0.0, 0.0, 0.0]
        self.Imu_angular_velocity = [0.0, 0.0, 0.0]
        self.Imu_linear_acceleration = [0.0, 0.0, 0.0]
        self.x_orientation_error = 0.0
        self.y_orientation_error = 0.0
        self.create_timer(0.0001,self.timer_callback)
        self.motor_velo_cur = [0.0,0.0]
        self.Kp_x = 4000.0
        self.Kp_y = 4000.0

        self.Ki_x = 0.1
        self.Ki_y = 0.1

        self.Kd_x = 0.1
        self.Kd_y = 0.1

        self.x_orientation_error = 0.0
        self.y_orientation_error = 0.0
        self.sum_error_x = 0.0
        self.sum_error_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

    def Imu_sensor_callback(self,msg):
        self.Imu_orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.Imu_angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.Imu_linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        rotation = self.quaternion_to_euler(self.Imu_orientation)
        self.y_orientation_error = 0.0144 - rotation[1] # rad
        self.x_orientation_error = 0.0144 - rotation[2]
        # print(self.Imu_orientation)
        # print(self.Imu_angular_velocity)
        # print(self.Imu_linear_acceleration)
        print('x = ' + str(rotation[2]) + 'y = ' + str(rotation[1]))
        print('X_error :',self.x_orientation_error)
        print('Y_error :',self.y_orientation_error)
        

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
    
        
    def timer_callback(self):
        # self.timestamp += 0.01
        # joint 0 y
        # joint 1 x

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
        self.prev_error_x = self.x_orientation_error
        self.prev_error_y = self.y_orientation_error

        joint0_velo = P_Y + I_Y + D_Y
        joint1_velo = P_X + I_X + D_X
        self.motor_velo_cur = [joint0_velo,joint1_velo]
        # print('Joint_X Velo', joint1_velo)
        # print('Joint_Y Velo', joint0_velo)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()