#!/usr/bin/env python3

import argparse
import signal
import yaml
from threading import Thread
import sys

import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names
from config_utilities_msgs.srv import SetConfig
from config_utilities_ros.gui import DynamicConfigGUI


class RosDynamicConfigGUI(Node):

    def __init__(self, args=None):
        super().__init__("ros_dynamic_config_gui")

        # Caching of connected config state.
        self._current_server = None
        self._current_key = None
        self._srv = None

        # Initialize the GUI and set the callback functions.
        self._gui = DynamicConfigGUI()
        self._gui.get_available_servers_and_keys_fn = self.get_available_servers_and_keys
        self._gui.set_request_fn = self.set_request
        self._spin_thread = None

    def run(self, **kwargs):
        """
        Run the GUI.
        """
        self._spin_thread = Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        # TODO(lschmid): For now let the GUI handle all interactions. In the future consider also supporting pushing to the GUI, e.g. when multiple clients are connected or configs are updated.
        self._gui.run(**kwargs)
        self.shutdown()

    def get_available_servers_and_keys(self):
        # We assume that no other node will use config_utilities messages with the same name.
        configs = [
            t[0][:-4]
            for t in self.get_service_names_and_types()
            if t[0].endswith("/set")
            and t[1][0] == "config_utilities_msgs/srv/SetConfig"
        ]

        available_nodes = [n[2] for n in get_node_names(
            node=self, include_hidden_nodes=False)]

        servers = {}
        for config in configs:
            for server in available_nodes:
                if not config.startswith(f"{server}/"):
                    continue
                key = config[len(server) + 1:]
                if server not in servers:
                    servers[server] = [key]
                else:
                    servers[server].append(key)
                break
        return servers

    def set_request(self, server, key, data):
        """
        Set the request for the given server and key.
        """
        # Connect to the ROS service.
        if self._current_server != server or self._current_key != key:
            srv_name = f"{server}/{key}/set"
            self._srv = self.create_client(SetConfig, srv_name)
            if not self._srv.wait_for_service(timeout_sec=1.0):
                return {"error": f"Service '{srv_name}' not available."}
            self._current_server = server
            self._current_key = key

        # Send the request.
        request = SetConfig.Request()
        request.data = yaml.dump(data)
        result = self._srv.call(request)
        if not result:
            return {"error": f"Service call to '{srv_name}' failed."}

        return yaml.safe_load(result.data)

    def _spin(self):
        """
        Spin the node in a separate thread.
        """
        rclpy.spin(self)

    def shutdown(self):
        rclpy.shutdown()
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join()
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(
        description="Webserver hosting dynamic configuration GUI."
    )
    parser.add_argument("--debug", "-d", action="store_true")
    parser.add_argument(
        "--host", type=str, default="localhost", help="Host to run the webserver on."
    )
    parser.add_argument(
        "--port", "-p", type=int, default=5000, help="Port to run the webserver on."
    )
    parser.add_argument(
        "--no-open-browser",
        action="store_true",
        help="Do not open the web browser automatically.",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    gui = RosDynamicConfigGUI()
    signal.signal(signal.SIGINT, lambda sig, frame: gui.shutdown())
    print(f"Running Dynamic Config GUI at 'http://{args.host}:{args.port}'.")
    gui.run(debug=args.debug, host=args.host, port=args.port,
            open_browser=not args.no_open_browser)
    gui.shutdown()


if __name__ == "__main__":
    main()
