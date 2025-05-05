#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from time import sleep
from config_utilities_msgs.srv import SetRequest


class RosDynamicConfigGUI(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        # self.publisher_ = self.create_publisher(String, "topic", 10)

        self.available_servers_and_keys = {}  # {server: [keys]}
        self.server_namespaces = {}  # {server: namespace}

        # ROS connection to selected key.
        self.sub = None
        self.srv = None
        # TODO(lschmid): Think about whether this should periodically pull or let the user refresh.
        self.configs_timer = self.create_timer(0.2, self.get_available_configs)

        self.setup()

    def setup(self):
        while rclpy.ok():
            self.get_available_configs()
            if self.available_servers_and_keys:
                break
            sleep(0.1)

    def get_available_configs(self):
        # We assume that no other node will use config_utilities messages with the same name.
        configs = [
            t[0][:-4]
            for t in self.get_service_names_and_types()
            if t[0].endswith("/set")
            and t[1][0] == "config_utilities_msgs/srv/SetRequest"
        ]

        # TODO(lschmid): This assumes that node names are unique. Can consider checking for non-uniqueness and including
        # the namespaces until unique.
        self.available_servers_and_keys.clear()
        self.server_namespaces.clear()
        for config in configs:
            # We assume that no other node will use config_utilities messages with the same name.
            names = config.split("/")
            server = names[-2]
            key = names[-1]
            self.server_namespaces[server] = "/".join(names[:-2])

            if server not in self.available_servers_and_keys:
                self.available_servers_and_keys[server] = [key]
            else:
                self.available_servers_and_keys[server].append(key)


def main(args=None):
    rclpy.init(args=args)
    gui = RosDynamicConfigGUI()
    rclpy.spin(gui)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
