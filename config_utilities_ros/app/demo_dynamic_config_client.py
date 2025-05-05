#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import yaml
from time import sleep
from config_utilities_ros.config_utilities_ros.app.dynamic_config_gui import (
    DynamicConfigGUI,
)

APP_NAME = "[Config Utilities Dynamic Config Client] "


class DynamicConfigRosClient:
    """
    A ROS client for the dynamic config GUI. This class connects to a dynamic config server
    and allows the user to interact with the server's configuration through YAML.
    """

    def __init__(self):
        # Setup the GUI.
        self.gui = DynamicConfigGUI()
        self.gui.value_changed_cb = self.send_config
        self.gui.key_selected_cb = self.key_selected_cb
        self.gui.server_selected_cb = self.server_selected_cb
        self.gui.server_selection.refresh_cb = self.refresh_servers

        # Variables.
        self.listening_ns = ""
        self.last_values_received = ""
        self.last_info_received = ""

        # ROS.
        self.config_pub = None
        self.config_sub = None
        self.config_info_sub = None
        self.reg_sub = None
        self.dereg_sub = None

        self.initialize()

    def initialize(self):
        servers = self.get_available_servers()
        if len(servers) == 0:
            print(f"{APP_NAME}Waiting for ROS Dynamic Config Servers to register...")
            while len(servers) == 0:
                sleep(0.1)
                servers = self.get_available_servers()

        # This will also selected a server and trigger the connection.
        self.gui.set_servers(servers)
        print(f"{APP_NAME}Connected to server '{self.listening_ns}'.")

    def subscriber_cb(self, msg):
        try:
            values = yaml.load(msg.data, Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(f"{APP_NAME}Error parsing incoming message YAML: {e}")
            return
        self.last_values_received = values
        self.gui.set_config(values)

    def info_sub_cb(self, msg):
        try:
            values = yaml.load(msg.data, Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(f"{APP_NAME}Error parsing incoming message YAML: {e}")
            return
        self.last_info_received = values
        self.gui.set_config_info(values)
        print(values)

    def reg_cb(self, _):
        # Instead of incremental tracking just update the configs.
        self.gui.set_keys(self.get_available_keys())

    def key_selected_cb(self, key):
        if key is not None:
            self.connect_topic(key)

    def server_selected_cb(self, server):
        servers = self.get_available_servers()
        if server not in servers:
            print(f"{APP_NAME}Server '{server}' not available.")
            self.gui.set_servers(servers)
            return
        self.connect_server(server)

    def send_config(self, new_values):
        if self.config_pub is None:
            return
        msg = String()
        msg.data = yaml.dump(new_values, default_flow_style=False)
        self.config_pub.publish(msg)

    def refresh_servers(self):
        previous_server = self.gui.current_server
        previous_key = self.gui.current_key
        servers = self.get_available_servers()
        self.gui.set_servers(servers)
        if (
            previous_server != self.gui.current_server
            or self.gui.current_key != previous_key
        ):
            self.last_values_received = ""
            self.last_info_received = ""
            return
        if self.last_info_received != "":
            self.gui.set_config_info(self.last_info_received)
        elif self.last_values_received != "":
            self.gui.set_config(self.last_values_received)

    def connect_topic(self, key):
        if self.config_pub is not None:
            self.config_pub.unregister()
            self.config_sub.unregister()
            self.config_info_sub.unregister()
        self.config_pub = rospy.Publisher(
            f"{self.listening_ns}/{key}/set", String, queue_size=10
        )
        self.config_sub = rospy.Subscriber(
            f"{self.listening_ns}/{key}/get", String, self.subscriber_cb
        )
        self.config_info_sub = rospy.Subscriber(
            f"{self.listening_ns}/{key}/info", String, self.info_sub_cb
        )

    def connect_server(self, server):
        self.listening_ns = server
        self.reg_sub = rospy.Subscriber(
            f"{self.listening_ns}/registered", String, self.reg_cb
        )
        self.dereg_sub = rospy.Subscriber(
            f"{self.listening_ns}/deregistered", String, self.reg_cb
        )
        self.gui.set_keys(self.get_available_keys())

    def get_available_servers(self):
        topics = rospy.get_published_topics()
        topics = [topic[0] for topic in topics if topic[1] == "std_msgs/String"]
        # We use the queue that all servers advertise these topics.
        reg = [t[:-11] for t in topics if t.endswith("/registered")]
        dereg = [t[:-13] for t in topics if t.endswith("/deregistered")]
        return [t for t in reg if t in dereg]

    def get_available_keys(self):
        topics = rospy.get_published_topics()
        topics = [t[0] for t in topics if t[1] == "std_msgs/String"]
        topics = [
            t[len(self.listening_ns) + 1 :]
            for t in topics
            if t.startswith(self.listening_ns)
        ]
        return [t[:-4] for t in topics if t.endswith("/get")]

    def spin(self):
        self.gui.mainloop()


def on_shutdown(client: DynamicConfigRosClient):
    # Force ctk shutdown.
    client.gui.quit()


def main():
    rospy.init_node("dynamic_config_client")
    client = DynamicConfigRosClient()
    rospy.on_shutdown(lambda: on_shutdown(client))
    client.spin()


if __name__ == "__main__":
    main()
