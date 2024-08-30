import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class Node2(Node):

    def __init__(self):
        super().__init__('node2')
        self.num1 = 0
        self.num2 = 0
        self.subscription1 = self.create_subscription(
            Float32,
            'num1',
            self.listener_num1,
            10)
        self.subscription2 = self.create_subscription(
            Float32,
            'num2',
            self.listener_num2,
            10)
        self.publisher = self.create_publisher(Float32, 'sum', 10)
        self.timer = self.create_timer(0.5, self.pub_sum)
    
    def pub_sum(self):
        msg = Float32()
        msg.data = self.num1 + self.num2

        self.publisher.publish(msg)
    
    def listener_num1(self, msg):
        self.num1 = msg.data

    def listener_num2(self, msg):
        self.num2 = msg.data
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Node2()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()