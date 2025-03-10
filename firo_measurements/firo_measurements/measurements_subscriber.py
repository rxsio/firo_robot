import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class

class MeasurementsSubscriber(Node):
    def __init__(self):
        super().__init__('measurements_subscriber')
        self.declare_parameter('topic_name', '/topic_name')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        message_type = get_msg_class(self, topic_name, include_hidden_topics=True)
        print('Message type:', message_type)
        self.create_subscription(message_type, '/topic_name', self.on_message_received, 1)

    def on_message_received(self, message):
        print(message)


def main(args=None):
    rclpy.init(args=args)
    test_subscriber = MeasurementsSubscriber()
    rclpy.spin(test_subscriber)
    test_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()