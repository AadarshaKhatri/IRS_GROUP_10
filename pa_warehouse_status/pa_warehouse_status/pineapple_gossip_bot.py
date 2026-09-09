import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import String


class Pineapple_gossip_bot(Node):

    def __init__(self):
        super().__init__('pineapple_gossip_bot')
        self.publisher_ = self.create_publisher(String, '/status_updates', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.messages = ["Hello","Hi","Test Message"]
        self.i = 0


    def timer_callback(self):
        msg = String()
        msg.data = random.choice(self.messages)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    pineapple_gossip_bot = Pineapple_gossip_bot()

    rclpy.spin(pineapple_gossip_bot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pineapple_gossip_bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()