import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from can_msgs.msg import Frame
from sensor_msgs.msg import BatteryState


class StatusBroadcaster(Node):

    def __init__(self):
        super().__init__('status_broadcaster')
        self.publisher_battery_state = self.create_publisher(BatteryState, 'battery_state', 10)
        self.publisher_can = self.create_publisher(Frame, 'can_frame', 10)
        self.subscriber_can= self.create_subscription(
            Frame,
            'can_frame',
            self.battery_callback,
            10)
        
        self.declare_parameter('update_period', 1.0)
        self.timer = self.create_timer(self.get_parameter('update_period').value, self.publish_rtr)

    def battery_callback(self, msg):
        if msg.id != 0x461 or msg.is_rtr or msg.dlc != 2:
            return
        data = msg.data
        voltage = float((data[1] << 8) | data[0])
        battery = BatteryState()
        battery.voltage = voltage
        self.publisher_battery_state.publish(battery)

        
    def publish_rtr(self):
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.is_rtr = True
        msg.id = 0x461
        self.publisher_can.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    status_broadcaster = StatusBroadcaster()
    
    try:
        rclpy.spin(status_broadcaster)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()
    status_broadcaster.destroy_node()

if __name__ == '__main__':
    main()
