import rclpy
from rclpy.node import Node

from std_msgs.msg import String

timerInterval = 5.0

class PID_Node(Node):

    def __init__(self):
        super().__init__('PID_Node')
        self.publisher_ = self.create_publisher(String, 'PID_Topic', 10)
        timerInterval  # seconds
        self.timer = self.create_timer(timerInterval, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args = None):
    rclpy.init(args=args)

    pidnode = PID_Node()

    rclpy.spin(pidnode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pidnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  