import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class Node1(Node):

    def __init__(self):
        super().__init__('node1')
        self.publisher1_ = self.create_publisher(Float32, 'num1', 10)
        self.publisher2_ = self.create_publisher(Float32, 'num2', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        msg1 = Float32()
        msg1.data = 6.0
        msg2 = Float32()
        msg2.data = 6.0

        self.publisher1_.publish(msg1)
        self.publisher2_.publish(msg2)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Node1()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()