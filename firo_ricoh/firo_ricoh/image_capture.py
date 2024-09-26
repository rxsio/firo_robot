#!/usr/bin/env python3
from os import path, makedirs
import gphoto2 as gp
from time import sleep
import rclpy
from std_srvs.srv import Trigger
from rclpy.node import Node


class Theta():
    """
    Define API for multiple exposure
    """

    def __init__(self, output_directory=path.expanduser("~/theta/")):
        if not path.exists(output_directory):
            makedirs(output_directory)
        self.output_directory = output_directory
        self.camera = gp.Camera()

        # Can raise the gp.GPhoto2Error
        # Caller should handle the exception, so we can't catch it here
        self.config = self.camera.get_config()

        while not self.is_ready():
            pass
        self.set_sleep("0")

    def wait_for_event(self, timeout=10, event_type=gp.GP_EVENT_TIMEOUT):
        """
        Wait for event_type to to be triggered.
        :param timeout:
        :param event_type:
        :return: event_data
        """
        while True:
            try:
                _event_type, event_data = self.camera.wait_for_event(timeout)
                if _event_type == gp.GP_EVENT_TIMEOUT:
                    return
                if _event_type == event_type:
                    return event_data
            except gp.GPhoto2Error:
                sleep(timeout)
                return

    def set_config(self, parameter, value):
        while self.get_config(parameter) != value:
            self.wait_for_event()
            gp.gp_widget_get_child_by_name(self.config, parameter)[1].set_value(value)
            self.camera.set_config(self.config)
        self.wait_for_event()

    def get_config(self, parameter):
        self.config = self.camera.get_config()
        self.wait_for_event()
        Ok, value = gp.gp_widget_get_child_by_name(self.config, parameter)
        return value.get_value() if Ok >= gp.GP_OK else None

    def is_ready(self):
        return self.get_config("5002") == "0"

    def set_sleep(self, sleep="1"):
        self.set_config("d80e", sleep)

    def set_capture_mode(self, mode="1"):
        self.set_config("5013", mode)

    def take_picture(self):
        self.set_sleep("0")
        self.set_capture_mode("1")
        gp_jpg_path = self.camera.capture(gp.GP_CAPTURE_IMAGE)
        jpg_path = path.join(gp_jpg_path.folder, gp_jpg_path.name)

        return jpg_path

    def download_file(self, src_path, dst_dir=None, dst_name=None, delete=True):
        """Copy the file from the camera src_path to local dst_path"""
        if dst_dir is None:
            dst_dir = self.output_directory

        src_folder, src_name = path.split(src_path)

        if dst_name is None:
            dst_name = src_name
        dst_path = path.join(dst_dir, dst_name)
        src_file = self.camera.file_get(src_folder, src_name, gp.GP_FILE_TYPE_NORMAL)
        src_file.save(dst_path)
        self.wait_for_event()

        if delete:
            self.camera.file_delete(src_folder, src_name)
            self.wait_for_event()
        return dst_dir, dst_name


class ThetaRos(Node):

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("output_directory", path.expanduser("~/theta/"))
        self.output_directory = self.get_parameter("output_directory").get_parameter_value().string_value
        try:
            self.camera = Theta(self.output_directory)
        except gp.GPhoto2Error as e:
            self.get_logger().error(str(e))
            raise e
        self.capture_image_service = self.create_service(Trigger, 'capture_image', self.handle_capture_image)

    def handle_capture_image(self, request, response):
        output_directory, output_name = self.camera.download_file(self.camera.take_picture(), delete=True)
        response.success = True
        response.message = path.join(output_directory, output_name)
        return response

def main(args=None):
    rclpy.init()
    
    try:
        node = ThetaRos("camera_theta")
    except gp.GPhoto2Error as e:
        rclpy.try_shutdown()
        return
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()
    node.destroy_node()

if __name__ == "__main__":
    main()
