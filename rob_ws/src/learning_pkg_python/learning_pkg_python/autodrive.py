#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions #for crtl+c setting
from geometry_msgs.msg import Twist #Twist message

class AutoDrive(Node):
    def __init__(self):
        super().__init__("auto_drive")

        self.publisher = self.create_publisher(Twist,"cmd_vel", 10)
        self.segments = [
            # (time,line speed,angular speed,"explain")s
            (2.0, 0.0, 0.0, "ready"),
            (5.0, 0.2, 0.0, "go straight"),
            (5.0, 0.2, 0.2, "turn left"),
            (10.0, 0.5, 0.0, "go straigh"),
            (3.0, 0.0, 0.0, "stop"),
        ]   #total time = ?

        self.start_time = None
        self.last_segment = -1  # init num
        self.finished = False   # task state
        
        #every 0.05s send once == 20HZ 
        self.timer = self.create_timer(0.05, self.update)

        # init log 
        self.get_logger().info(
            "waiting subscriber, please start the bridge and play the GAZEBO"
        )
    # publish speed 
    def send_velocity(self,linear,angular):
        msg = Twist() #creat a Twist message
        msg.linear.x = float(linear) #linear speed
        msg.angular.z = float(angular) #angular speed
        self.publisher.publish(msg)
    
    #timer
    def update(self): 
        if self.publisher.get_subscription_count() == 0: # if subscriber is empty
            self.send_velocity(0.0,0.0)                  # keep speed 0
            return

        now = self.get_clock().now().nanoseconds / 1e9 # transfer into second

        if self.start_time is None:
            self.start_time = now
            self.get_logger().info("开始执行轨迹")
            # self.get_logger().info("start running")

        period_time = now - self.start_time #working time
        segment_end = 0.0 # init end time

        for index, (duration, linear, angular, label) in enumerate(self.segments):
            segment_end += duration             # calculate the end time
            if period_time < segment_end:       # if not end time
                if index != self.last_segment:   # if is new stage
                    self.get_logger().info(
                        f"{index + 1}:"         # print num
                        f"v={linear} m/s, w={angular} rad/s" # print speed 
                    )
                    self.last_segment = index # update num

                self.send_velocity(linear,angular) 
                return

        self.send_velocity(0.0,0.0) # if out of loop, it means time out
        self.timer.cancel() #cancel the update
        self.finished = True
        self.get_logger().info("轨迹执行完成，小车已收到停车指令")  
        # self.get_logger().info("finish task, car has received stop")  
    
def main(args=None):
    # ros2 init
    rclpy.init(
        args=args,
        signal_handler_options= SignalHandlerOptions.NO,    # close rclpy and go to ctrl+c
    )

    node = AutoDrive()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:                       # ctrl+c
        node.get_logger().info("manually interupt") # stop car
    finally:                                        # when finish task
        if rclpy.ok():                              # continue send stop
            for _ in range(5):
                node.send_velocity(0.0,0.0)
                time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()