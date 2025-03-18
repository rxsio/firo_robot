import rclpy
from rclpy.node import Node
from ros2topic.api import get_msg_class
from firo_measurements.configuration import Topic, Influx, validate_configuration
import yaml
from pydantic import ValidationError
from typing import List, Union, Dict



class FieldType:
    name: str
    value: Union[str, int, float, bool]
    tags: Dict[str, str]

class MeasurementType:
    measurement: str
    fields: List[FieldType]

class EvaluationError(Exception):
    pass

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        self.declare_parameter('telemetry_yaml', '/telemetry.yaml')
        self._configuration = None
        self.load_configuration()
        self._topics = {}
        
        

    def shutdown(self, reason=''):
        self.get_logger().error(f"Shutting down: {reason}")
        self.destroy_node()
        rclpy.shutdown()

    def load_configuration(self):
        """
        loading configuration file & getting topics and outputs
        """

        telemetry_yaml = self.get_parameter('telemetry_yaml').get_parameter_value().string_value
        self.get_logger().info(f'Loading configuration from: {telemetry_yaml}')
        
        try:
            with open(telemetry_yaml) as stream:
                try:
                    self.data = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    self.get_logger().error(f'Failed to load configuration: {exc}')
                    print(exc)

            if 'topics' not in self.data:
                self.get_logger().error("Missing 'topics' in configuration file")
                return
                
            if 'outputs' not in self.data:
                self.get_logger().error("Missing 'outputs' in configuration file")
                return

            try:
                self._configuration = validate_configuration(
                    topics=self.data.get("topics", []),
                    outputs=self.data.get("outputs", {})
                )
                self.get_logger().info('Configuration loaded successfully')

                
                self.subscribe_topics()

            except ValidationError as e:
                self.shutdown(f"Configuration validation failed with error: {e}")

        except FileNotFoundError as e:
            self.shutdown(f'Failed to load configuration: {e}')
    

    def subscribe_topics(self):
        if not self._configuration or not self._configuration.topics:
            self.get_logger().warn('No topics defined in configuration')
            return
        
        for topic in self._configuration.topics:
            self.get_logger().info(f'Registering topic: {topic.name}')
            message_type = get_msg_class(self, topic.name, include_hidden_topics=True)
            try:
                self.create_subscription(message_type, topic.name, lambda msg: self.on_message_received(msg, topic), 1)
                
            except Exception as e:
                self.get_logger().error(f'Failed to register topic: {e}')

        self.get_logger().info('Registered topics with known message type')


    def on_message_received(self, message, configuration):
        if not configuration:
            rclpy.shutdown("Invalid topic callback handler")
        
        else:
            self.get_logger().info(f'Received message on topic: {configuration.name}')
            measurements = self.evaluate_measurement(message, configuration)

            for measurement in measurements:
                self.write_measurements(measurement)


    def evaluate_measurement(self, msg, configuration:Topic) -> List[MeasurementType]:
        measurements = []
        try:
            for measurement in configuration.measurements:      
                measurements.append({'measurement': measurement.name, 'fields': self.evaluate_fields(measurement, msg, configuration)})

        except Exception as e:  
            self.get_logger().error(f'Failed to evaluate measurement: {e}')
        return measurements


    def evaluate_fields(self, measurement, msg, configuration):
        """
        separate fields
        """
        fields = []
        for field in measurement.measurement_fields:
            fields.append(self.evaluate_measurement_field(field, msg))
        return fields


    def evaluate_measurement_field(self, field, msg):
        value = self.evaluate_tag(msg, field.value)
        tags = {}

        for tag, tag_value in field.tags.items():
            new_tag_value = self.evaluate_tag(msg, tag_value)
            tags[tag] = new_tag_value
        
        return {'field':field.field,
                'value': value,
                'tags': tags}


    def evaluate_tag(self, msg, tag_value):
        """
        get exact values described in configuration file
        """
        context = {}
        context['$msg'] = msg

        if not tag_value.startswith('$'):
            return tag_value
            
        
        reference, *parts = tag_value.split(".")
        if reference not in context:
            raise EvaluationError(f"Unknown reference to {reference}")
        
        value = context.get(reference)

        for part in parts:
            try:
                # for example: "$msg.motors.[0].id"
                if isinstance(value, (list, tuple)):
                    if not part.startswith("[") or not part.endswith(']'):
                        raise EvaluationError(f"Expected index [], got {part}")
                    
                    index = int(part[1:-1])
                    value = value[index]


                elif isinstance(value, dict):
                    value = value.get(part)

                # for example: "$msg.linear.x"
                else:
                    value = getattr(value, part)

            except AttributeError as e:
                raise EvaluationError(f"Cannot evaluate '{tag_value}': {e}")

        return value


    def write_measurements(self, measurement: MeasurementType):
        name = measurement.get('measurement')
        fields = measurement.get('fields')

        for field in fields:
            name_field = field.get('field')
            value = field.get('value')
            tags = field.get('tags')
            self.get_logger().info(f'Received message on topic: {name}, field:{name_field}, value:{value}, tags:{tags}')

       

def main(args=None):
    rclpy.init(args=args)
    test_subscriber = TelemetryNode()
    rclpy.spin(test_subscriber)
    test_subscriber.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()