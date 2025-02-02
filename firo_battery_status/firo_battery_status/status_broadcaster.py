import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_socketcan_msgs.msg import FdFrame
from sensor_msgs.msg import BatteryState


class StatusBroadcaster(Node):

    def __init__(self):
        super().__init__('status_broadcaster')
        self.publisher_battery_state = self.create_publisher(BatteryState, 'battery_state', 10)
        self.publisher_can_frame = self.create_publisher(FdFrame, 'can_frame', 10)
        self.timer = self.create_timer(1.0, self.publish_rtr)
        self.i = 0
        self.get_voltage= self.create_subscription(
            FdFrame,
            'can_frame',
            self.battery_callback,
            10)

    def battery_callback(self, msg):
        if msg.id != 0x461:
            return
        data = msg.data
        voltage = float((data[1] << 8) | data[0])
        battery = BatteryState()
        battery.voltage = voltage
        self.publisher_battery_state.publish(battery)

        
    def publish_rtr(self):
        msg = FdFrame()
        msg.id = 0x461
        self.publisher_can_frame.publish(msg)
        self.i += 1


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
