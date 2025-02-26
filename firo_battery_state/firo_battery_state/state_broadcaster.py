import struct
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from can_msgs.msg import Frame
from sensor_msgs.msg import BatteryState


class StateBroadcaster(Node):

    def __init__(self):
        super().__init__('state_broadcaster')
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
        self.declare_parameter('max_voltage', 40.5)
        self.declare_parameter('min_voltage', 32.0)
        self.max_voltage = self.get_parameter('max_voltage').get_parameter_value().double_value
        self.min_voltage = self.get_parameter('min_voltage').get_parameter_value().double_value

    def battery_callback(self, msg):
        if msg.id != self.can_id or msg.is_rtr or msg.dlc != 2:
            return
        data = bytearray(msg.data[0:2])
        # < means little-endian, H means uint16_t
        # we divide by 1000 to convert from mV to V
        voltage = float(struct.unpack('<H', data)[0]) / 1000.0
        battery = BatteryState()
        battery.voltage = voltage        
        battery.percentage = self.convert_voltage_to_percentage(voltage)
        self.publisher_battery_state.publish(battery)

    def convert_voltage_to_percentage(self, voltage):
        if voltage < self.min_voltage:
            return 0.0
        if voltage > self.max_voltage:
            return 1.0
        return (voltage - self.min_voltage) / (self.max_voltage - self.min_voltage)

        
    def publish_rtr(self):
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.is_rtr = True
        msg.id = self.can_id
        self.publisher_can.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    state_broadcaster = StateBroadcaster()
    executor = MultiThreadedExecutor()
    executor.add_node(state_broadcaster)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()
    state_broadcaster.destroy_node()

if __name__ == '__main__':
    main()
