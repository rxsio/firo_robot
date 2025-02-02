import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from can_msgs.msg import Frame
from sensor_msgs.msg import BatteryState


class StatusBroadcaster(Node):

    def __init__(self):
        super().__init__('status_broadcaster')
        self.publisher_battery_state = self.create_publisher(BatteryState, 'battery_state', 10)
        self.publisher_can = self.create_publisher(Frame, 'to_can_bus', 10)
        self.subscriber_can= self.create_subscription(
            Frame,
            'from_can_bus',
            self.battery_callback,
            10)
        
        self.declare_parameter('can_id', 0x461)
        self.can_id = self.get_parameter('can_id').value
        self.declare_parameter('update_period', 1.0)
        self.timer = self.create_timer(self.get_parameter('update_period').value, self.publish_rtr)

    def battery_callback(self, msg):
        if msg.id != self.can_id or msg.is_rtr or msg.dlc != 2:
            return
        data = bytearray(msg.data[0:2])
        # < means little-endian, H means uint16_t
        # we divide by 1000 to convert from mV to V
        voltage = float(struct.unpack('<H', data)[0]) / 1000.0
        battery = BatteryState()
        battery.voltage = voltage
        self.publisher_battery_state.publish(battery)

        
    def publish_rtr(self):
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.is_rtr = True
        msg.id = self.can_id
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
