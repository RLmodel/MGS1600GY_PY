import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String, Int64, Bool
from geometry_msgs.msg import Twist
from enum import IntEnum,auto, Enum
from copy import deepcopy
import numpy as np

##
from collections import deque
class Deque(deque):

    def __init__(self):
        self.left_value = 0
        self.right_value = 0
        self.average = 0
        self.len = 20
    def update_average(self):
        self.average = sum(self) / self.len
##      


# 센서값 범위 설정
sensor_range = (-100, 100)

# LUT 크기 설정
lut_size = 201 # 201
# LUT 생성
sensor_values = np.linspace(sensor_range[0], sensor_range[1], lut_size)
steering_values = np.tanh(sensor_values *0.01) # tanh 함수를이용하여 LUT 생성
lut = dict(zip(sensor_values, steering_values))

logger = rclpy.logging.get_logger('mgs_control')

class StateMachine(Enum):
    NAV = 0
    TRANS1 = 1 # NAVIGATION to Magnetic tape
    MGS = 2
    TRANS2 = 3 # Magnetic Tape to NAVIGATION

class MgsControl(Node):
    
    def __init__(self):
        super().__init__('mgs_control') 
        self.sub_T = self.create_subscription(Int64, "/mgs_T", self.set_angular, 10)
        self.sub_state = self.create_subscription(Bool, "/state_machine", self.state_callback, 10)
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_ = self.create_timer(0.02, self.timer_callback)
        
        self.msg = Twist()
        self.state = StateMachine.NAV
        self.prev_state = StateMachine.NAV
        self.value_T = 0
        self.count = 0

        ##
        self.queue = Deque()
        self.MAX = 10
        ##

    def __del__(self):
        msg = self.msg
        msg.linear.x = 0
        msg.angular.z = 0
        self.pub_twist.publish(msg)

    def state_callback(self, data):
        
        # if self.state == StateMachine.TRANSITION:
        #     # logger.info(f"{self.state}")
        #     self.count += 1
        #     self.state = StateMachine.TRANSITION
        #     self.msg.linear.x = 0.0
        #     if self.count > 20:
        #         self.count = 0
        #         self.state = StateMachine.IDLE
        if self.state == StateMachine.TRANS1:
            pass
        
        elif self.state == StateMachine.TRANS2 and data.data == True:
            self.state = StateMachine.MGS

        elif self.state == StateMachine.TRANS2:
            pass

        elif data.data == True:
            self.state = StateMachine.MGS
            # self.msg.linear.x = 0.2
            # logger.info(f"{self.state}")

        elif data.data == False:
            self.state = StateMachine.NAV
            # logger.info(f"{self.state}")
        else :
            pass
        
        self.set_linear()
        self.prev_state = deepcopy(self.state)
        logger.info("STATE: {}".format(self.state))

    def set_linear(self):
        
        if self.state == StateMachine.TRANS1:
            if abs(self.value_T) > 15:
                self.msg.linear.x = 0.0
            else:
                self.state = StateMachine.MGS
        
        ##

        elif self.state == StateMachine.TRANS2:
            logger.info("TRANS2 working: {} vs {}".format(self.queue.left_value, self.queue.right_value))
            if self.queue.average > 10:
                self.msg.angular.z = float(lut[round(self.queue.average)])
                self.msg.linear.x = 0.0
                logger.warn("Recovery: Turn LEFT")
            elif self.queue.average < -10:
                self.msg.angular.z = float(lut[round(self.queue.average)])
                self.msg.linear.x = 0.0
                logger.warn("Recovery: Turn RIGHT")
            else: 
                self.state = StateMachine.NAV
            # else:
            #     self.msg.linear.x = 0.1
            #     self.msg.angular.z = 0.0
            #     self.count += 1
            #     logger.warn("Recovery: Straight")
            #     if self.count > 10:
            #         self.count = 0
            #         self.state = StateMachine.NAV
        ##
        
        elif self.state == StateMachine.MGS and self.prev_state == StateMachine.MGS:
            self.msg.linear.x = 0.2
            if abs(self.value_T) > 50:
                self.msg.linear.x = 0.1
            elif abs(self.value_T > 75):
                self.msg.linear.x = 0.0

        elif self.state == StateMachine.MGS and self.prev_state == StateMachine.NAV:
            self.state = StateMachine.TRANS1        
        
        elif self.state == StateMachine.NAV and self.prev_state == StateMachine.MGS:
            self.state = StateMachine.TRANS2

        else:
            pass

    def set_angular(self, data):
        # logger.info("type: {}".format(type(self.msg.angular.z)))
        self.value_T = data.data

        ##
        self.queue.appendleft(self.value_T)
        self.queue.update_average()
        if len(self.queue) > self.queue.len:
            self.queue.pop()
        logger.info("\nqueue: {}\naverage: : {}".format(self.queue, self.queue.average))    
        ##


        if data.data > 10:
            # logger.info("go LEFT")
            self.msg.angular.z = float(lut[data.data])
            # self.msg.angular.z = data.data*0.005
            
        elif data.data < -10:
            # logger.info("go RIGHT")
            self.msg.angular.z = float(lut[data.data])
            # self.msg.angular.z = data.data*0.005
        else:
            self.msg.angular.z = 0.0


            
    def timer_callback(self):
        # logger.info(f"{self.state}")
        if self.state == StateMachine.NAV:
            return False
        
        # elif self.state == StateMachine.TRANSITION:
        #     self.pub_twist.publish(self.msg)

        else:
            # self.state == StateMachine.MGS:
            self.pub_twist.publish(self.msg)
            
      
def main(args=None):
    rclpy.init(args=args)
    node = MgsControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

