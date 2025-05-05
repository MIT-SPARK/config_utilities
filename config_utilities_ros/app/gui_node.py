#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from time import sleep
import signal
import sys
import os
from threading import Thread
from config_utilities_msgs.srv import SetRequest
from config_utilities_ros.gui import DynamicConfigGUI


class RosDynamicConfigGUI(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        # self.publisher_ = self.create_publisher(String, "topic", 10)

        self.available_servers_and_keys = {}  # {server: [keys]}

        # ROS connection to selected key.
        self.sub = None
        self.srv = None

        # State so the GUI can poll.
        self.alive = False

        # Initialize the GUI.
        self.gui = DynamicConfigGUI()

    def run(self):
        """
        Run the GUI.
        """
        # self.alive = True
        ros_thread = Thread(target=self._spin, daemon=True)
        ros_thread.start()
        # TODO(lschmid): Think about whether this should periodically pull or let the user refresh.
        self.configs_timer = self.create_timer(0.1, self.get_available_configs)
        self.gui.on_setup_complete = self._setup_complete
        self.gui.run(debug=True)
        self.alive = False
        ros_thread.join()

    def _setup_complete(self):
        self.alive = True

    def _spin(self):
        """
        Spin the node in a separate thread.
        """
        rclpy.spin(self)

    def shutdown(self, _, __):
        rclpy.shutdown()
        sys.exit(0)

    def get_available_configs(self):
        # We assume that no other node will use config_utilities messages with the same name.
        configs = [
            t[0][:-4]
            for t in self.get_service_names_and_types()
            if t[0].endswith("/set")
            and t[1][0] == "config_utilities_msgs/srv/SetRequest"
        ]

        new_servers = {}
        for config in configs:
            # We assume that no other node will use config_utilities messages with the same name.
            ind = config.rfind("/")
            server = config[:ind]
            key = config[ind + 1 :]
            if server not in new_servers:
                self.available_servers_and_keys[server] = [key]
            else:
                self.available_servers_and_keys[server].append(key)
        if new_servers != self.available_servers_and_keys and self.alive:
            self.gui.set_available_servers_and_keys(self.available_servers_and_keys)


def main(args=None):
    rclpy.init(args=args)
    gui = RosDynamicConfigGUI()
    signal.signal(signal.SIGINT, gui.shutdown)
    gui.run()
    gui.shutdown()


if __name__ == "__main__":
    main()
