import rclpy
from rclpy.node import Node
from scion_types.msg import pidData
from simple_pid import PID
import time
import numpy as np
from can_client import Can_Client

yawP, yawI, yawD = .5, .1, .01
pitchP, pitchI, pitchD = .5, .1, .01
rollP, rollI, rollD = .5, .1, .01
xP, xI, xD = .5, .1, .01
yP, yI, yD = .5, .1, .01
zP, zI, zD = .5, .1, .01

yawPID = PID(yawP, yawI, yawD, 0)
pitchPID = PID(pitchP, pitchI, pitchD, 0)
rollPID = PID(rollP, rollI, rollD, 0)
xPID = PID(xP, xI, xD, 0)
yPID = PID(yP, yI, yD, 0)
zPID = PID(zP, zI, zD, 0)

controlMatrix = np.array[[0], [0], [0], [0], [0], [0]] #{yaw, pitch, roll, x, y, z}

motorControlMatrix = [[0], [0]]

junebugMatrix = [[-1, 0, 0, 1, 0, 0], #motor1 [yaw, pitch, roll, x, y, z]
                 [ 1, 0, 0, 1, 0, 0]] #motor2 [yaw, pitch, roll, x, y, z]


class PID_Controller(Node):
    def __init__(self):
        super().__init__('PID_Controller')
        self.subscription = self.create_subscription(
            String,
            'PID_Topic',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        controlMatrix[0,0] = yawPID(msg.axes[0]) 
        controlMatrix[1,0] = pitchPID(msg.axes[1])
        controlMatrix[2,0] = rollPID(msg.axes[2])
        controlMatrix[3,0] = xPID(msg.axes[3])
        controlMatrix[4,0] = yPID(msg.axes[4])
        controlMatrix[5,0] = zPID(msg.axes[5])

        motorControlMatrix = np.dot(junebugMatrix, controlMatrix)

        can_client.make_motor_request(ravel(motorControlMatrix))

        
        
    
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