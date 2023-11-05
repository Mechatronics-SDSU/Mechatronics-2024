import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from simple_pid import PID
import time

locP = 1
locI = .05
locD = .01
angleP = 1
angleI = .05
angleD = .01
desiredLocation = [0, 0] #[x, y]
currLocation = [0, 0] #[x, y]
desiredAngle = 0.0
currAngle = 0.0

locationPID = PID(locP, locI, locD, desiredLocation)
anglePID = PID(angleP, angleI, angleD, desiredAngle)


class PID_Controller(Node):
    def __init__(self):
        super().__init__('PID_Controller')
        self.subscription = self.create_subscription(
            String,
            'PID_Topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        self.correctAngle()
        self.moveTo()

    def correctAngle(self):
        global currAngle
        while (currAngle != desiredAngle):
            currAngle += anglePID(currAngle)
            anglePID.reset()

    def moveTo(self):
        global currLocation
        while(currLocation != desiredLocation):
            currLocation += locationPID(currLocation)
            locationPID.reset()
    
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