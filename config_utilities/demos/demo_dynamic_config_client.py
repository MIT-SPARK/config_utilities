#!/usr/bin/env python3
from tkinter import *
import customtkinter as ctk
import rospy
from std_msgs.msg import String
import yaml
from time import sleep

ctk.set_appearance_mode(
    "System")  # Modes: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme(
    "blue")  # Themes: "blue" (standard), "green", "dark-blue"
DEBUG = False  # Disable ROS for debugging.
PAD_X = 10
PAD_Y = 10
APP_NAME = "[Config Utilities Dynamic Config Client] "


class DynamicConfigGUI(ctk.CTk):

    class GUIConfig:
        width = 700
        height = 600

    def __init__(self):
        super().__init__()

        # Callbacks from the GUI. Optionally set by the invoker.
        self.key_selected_cb = None
        self.server_selected_cb = None
        self.value_changed_cb = None

        # GUI configuration.
        self.gui_config = self.GUIConfig()

        # Data.
        self.current_key = None
        self.current_server = None

        self.setup_frame()

    def setup_frame(self):
        # Master.
        self.title("Config Utilities Dynamic Config Client")
        self.geometry(f"{self.gui_config.width}x{self.gui_config.height}")

        # Key selection.
        self.key_selection = self.SelectionDropDown(self, self._key_selected)
        self.key_selection.grid(row=0, column=0, sticky="ew")

        # Config editing.
        self.config_frame = self.PlainTextConfigFrame(self,
                                                      self.value_changed_cb)
        self.config_frame.grid(row=1, column=0, sticky="ns")

        # TODO(lschmid): Consider making this more general, specialized to ROS for now.
        self.server_selection = self.RosStatusBar(self, self._server_selected)
        self.server_selection.send_cb = self._value_changed_cb
        self.server_selection.grid(row=2, column=0, sticky="ew")

        self.rowconfigure([0, 2],
                          minsize=self.key_selection.winfo_reqheight(),
                          pad=PAD_Y,
                          weight=0)
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1, pad=PAD_X)

    # Interfaces for outside interaction with the GUI.
    def set_keys(self, keys):
        self.key_selection.set_keys(keys)

    def set_servers(self, servers):
        self.server_selection.set_keys(servers)

    def set_config(self, new_values):
        self.config_frame.set_config(new_values)

    # Functionality.
    def _key_selected(self, key):
        self.current_key = key
        if self.key_selected_cb is not None:
            self.key_selected_cb(key)

    def _server_selected(self, server):
        self.current_server = server
        if self.server_selected_cb is not None:
            self.server_selected_cb(server)

    def _value_changed_cb(self):
        if self.value_changed_cb is not None:
            self.value_changed_cb(self.config_frame.get_config())

    class SelectionFrame(ctk.CTkFrame):
        """
        Interface class for key selection.
        """

        def __init__(self, master, key_selected_cb):
            super().__init__(master)
            self.key_selected_cb = key_selected_cb

        def set_keys(self, new_keys):
            pass

    class SelectionDropDown(SelectionFrame):

        def __init__(self, master, key_selected_cb):
            super().__init__(master, key_selected_cb)
            self.current_key = None
            self.no_options_text = "No Dynamic Configs Registered."

            self.w_label = ctk.CTkLabel(self, text="Config:")
            self.w_label.grid(row=0,
                              column=0,
                              padx=PAD_X,
                              pady=PAD_Y,
                              sticky="nsw")
            self.w_dropdown = ctk.CTkOptionMenu(self,
                                                dynamic_resizing=True,
                                                command=self._on_change)
            self.w_dropdown.grid(row=0,
                                 column=1,
                                 sticky="nsew",
                                 padx=PAD_X,
                                 pady=PAD_Y)
            self.columnconfigure(1, weight=1)

        def set_keys(self, new_keys):
            if new_keys == []:
                # No keys available.
                if self.current_key is not None:
                    self.key_selected_cb(None)
                self.current_key = None
                self.w_dropdown.set(self.no_options_text)
                self.w_dropdown.configure(state=DISABLED)
                return

            self.w_dropdown.configure(values=new_keys, state=NORMAL)
            if self.current_key in new_keys:
                return
            self.current_key = new_keys[0]
            self.key_selected_cb(self.current_key)
            self.w_dropdown.set(self.current_key)

        def _on_change(self, _):
            key = self.w_dropdown.get()
            if key == self.current_key:
                return
            self.current_key = key
            self.key_selected_cb(key)

    class RosStatusBar(SelectionDropDown):

        def __init__(self, master, key_selected_cb):
            super().__init__(master, key_selected_cb)
            # Callbacks hooks.
            self.refresh_cb = None
            self.send_cb = None

            self.no_options_text = "No RosDynamicConfigServers Registered."
            self.w_label.configure(text="Config Server:")
            self.w_refresh_button = ctk.CTkButton(
                self, text="Refresh", command=self._on_reset_button)
            self.w_refresh_button.grid(row=0,
                                       column=3,
                                       padx=PAD_X,
                                       pady=PAD_Y,
                                       sticky="nse")
            self.w_send_button = ctk.CTkButton(self,
                                               text="Send",
                                               command=self._on_send_button)
            self.w_send_button.grid(row=0,
                                    column=4,
                                    padx=PAD_X,
                                    pady=PAD_Y,
                                    sticky="nse")
            self.columnconfigure(2, weight=1)
            self.columnconfigure([0, 1, 3, 4], weight=0)

        def _on_reset_button(self):
            if self.refresh_cb is not None:
                self.refresh_cb()

        def _on_send_button(self):
            if self.send_cb is not None:
                self.send_cb()

    class ConfigFrame(ctk.CTkFrame):
        """
        Interface class for configuration editing.
        """

        def __init__(self, master, send_update_fn=None):
            super().__init__(master)
            self.send_update_fn = send_update_fn

        def set_config(self, new_config):
            pass

        def get_config(self):
            return {}

        def set_enabled(self, enabled):
            pass

    class PlainTextConfigFrame(ConfigFrame):

        def __init__(self, master, send_update_fn=None):
            super().__init__(master, send_update_fn)
            self.w_text = Text(self, wrap=CHAR, width=1000, undo=True)
            self.w_text.pack(fill=BOTH, expand=True, padx=PAD_X, pady=PAD_Y)
            self.w_text.bind("<KeyRelease>", self._on_key_release)

        def set_config(self, new_config):
            self.w_text.delete("0.0", END)
            self.w_text.insert("0.0", yaml.dump(new_config))

        def get_config(self):
            try:
                return yaml.load(self.w_text.get("1.0", END),
                                 Loader=yaml.FullLoader)
            except yaml.YAMLError as e:
                print(f"{APP_NAME}Error parsing YAML: {e}")
                return None

        def set_enabled(self, enabled):
            new_state = NORMAL if enabled else DISABLED
            self.w_text.configure(new_state)

        def _on_key_release(self, event):
            # Ctrl + Enter.
            if event.keysym == "Return" and event.state == 20:
                self.send_update_fn()


class DynamicConfigRosClient:

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

        # ROS.
        self.config_pub = None
        self.config_sub = None
        self.reg_sub = None
        self.dereg_sub = None

        self.initialize()

    def initialize(self):
        servers = self.get_available_servers()
        if len(servers) == 0:
            print(
                f"{APP_NAME}Waiting for ROS Dynamic Config Servers to register..."
            )
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
        if (previous_server != self.gui.current_server
                or self.gui.current_key != previous_key):
            self.last_values_received = ""
        else:
            self.gui.set_config(self.last_values_received)

    def connect_topic(self, key):
        if self.config_pub is not None:
            self.config_pub.unregister()
            self.config_sub.unregister()
        self.config_pub = rospy.Publisher(f"{self.listening_ns}/{key}/set",
                                          String,
                                          queue_size=10)
        self.config_sub = rospy.Subscriber(f"{self.listening_ns}/{key}/get",
                                           String, self.subscriber_cb)

    def connect_server(self, server):
        self.listening_ns = server
        self.reg_sub = rospy.Subscriber(f"{self.listening_ns}/registered",
                                        String, self.reg_cb)
        self.dereg_sub = rospy.Subscriber(f"{self.listening_ns}/deregistered",
                                          String, self.reg_cb)
        self.gui.set_keys(self.get_available_keys())

    def get_available_servers(self):
        topics = rospy.get_published_topics()
        topics = [
            topic[0] for topic in topics if topic[1] == "std_msgs/String"
        ]
        # We use the queue that all servers advertise these topics.
        reg = [t[:-11] for t in topics if t.endswith("/registered")]
        dereg = [t[:-13] for t in topics if t.endswith("/deregistered")]
        return [t for t in reg if t in dereg]

    def get_available_keys(self):
        topics = rospy.get_published_topics()
        topics = [t[0] for t in topics if t[1] == "std_msgs/String"]
        topics = [
            t[len(self.listening_ns) + 1:] for t in topics
            if t.startswith(self.listening_ns)
        ]
        return [t[:-4] for t in topics if t.endswith("/get")]

    def spin(self):
        self.gui.mainloop()


def on_shutdown(gui: DynamicConfigGUI):
    # Force ctk shutdown.
    gui.quit()


def debug_main():
    gui = DynamicConfigGUI()
    gui.set_keys(["A", "B", "C"])
    gui.mainloop()


def main():
    rospy.init_node("dynamic_config_client")
    client = DynamicConfigRosClient()
    rospy.on_shutdown(lambda: on_shutdown(client.gui))
    client.spin()


if __name__ == "__main__":
    if DEBUG:
        debug_main()
    else:
        main()
