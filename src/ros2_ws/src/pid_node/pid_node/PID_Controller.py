import rclpy
from rclpy.node import Node
# from 
# from scion_types.msg import pidData
from simple_pid import PID
import time
import numpy as np
from can_client import Can_Client
# from


PID_UPDATE_PERIOD = 1

yawP, yawI, yawD = .5, .1, .01
pitchP, pitchI, pitchD = .5, .1, .01
rollP, rollI, rollD = .5, .1, .01
xP, xI, xD = .5, .1, .01
yP, yI, yD = .5, .1, .01
zP, zI, zD = .5, .1, .01
# 
yawPID = PID(yawP, yawI, yawD, 0)
pitchPID = PID(pitchP, pitchI, pitchD, 0)
rollPID = PID(rollP, rollI, rollD, 0)
# 
xPID = PID(xP, xI, xD, 0)
yPID = PID(yP, yI, yD, 0)
zPID = PID(zP, zI, zD, 0)

controlMatrix = np.array([[0],  #yaw
                          [0],  #pitch
                          [0],  #roll
                          [0],  #x
                          [0],  #y
                          [0]]) #z

motorControlMatrix = np.array([[0],  #motor1
                               [0],  #motor2
                               [0],  #motor3
                               [0],  #motor4
                               [0],  #motor5
                               [0],  #motor6
                               [0],  #motor7
                               [0]]) #motor8

junebugMatrix = np.array([[1, 0, 0, 1, 0, 0],  #motor1 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor2 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor3 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor4 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor5 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor6 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0],  #motor7 [yaw, pitch, roll, x, y, z]
                          [1, 0, 0, 1, 0, 0]]) #motor8 [yaw, pitch, roll, x, y, z]

class PID_Controller(Node):
    def __init__(self):
        super().__init__('PID_Controller')
        self.can_client = Can_Client()
        # self.cur_state_client = Node.create_client(CurState, "cur_state")
        # self.dest_state_client = Node.create_client(DestState, "dest_state")
        

        self.timer = self.create_timer(PID_UPDATE_PERIOD, self.listener_callback)
        # self.srv = self.create_service(SendMotor, 'motor_response', self.send_response)
    
        
        # self.subscription = self.create_subscription(
        #     String,
        #     'PID_Topic',
        #     self.listener_callback,
        #     1)
        # self.subscription  # prevent unused variable warning

        # Change to service from topic to get the data

        # Get destination data
    
        # Create two clients: one for getting the current state and one for the desired state
        # We have a service inside of the PID itself

    def send_repsonse(self,):
        

        controlMatrix[0,0] = yawPID(-1) 
        controlMatrix[1,0] = pitchPID(-1)
        controlMatrix[2,0] = rollPID(-2)
        controlMatrix[3,0] = xPID(-3)
        controlMatrix[4,0] = yPID(-4)
        controlMatrix[5,0] = zPID(-5)

        motorControlMatrix = np.dot(junebugMatrix, controlMatrix)
        something = self.convertToList(motorControlMatrix)

        self.can_client.make_motor_request(something)

    def convertToList(self, matrix):
        newlist = []

        for dumbshit in matrix:
            newlist.append(int([dumbshit][0]))
        
        return newlist
    
def main(args = None):
    rclpy.init(args = args)
    minimal_subscriber = PID_Controller()

    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()