#!/usr/bin/env python3
from tkinter import *
from typing import Any
import customtkinter as ctk
import yaml

ctk.set_appearance_mode(
    "System")  # Modes: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme(
    "blue")  # Themes: "blue" (standard), "green", "dark-blue"
PAD_X = 10
PAD_Y = 10
GUI_NAME = "[Config Utilities Dynamic Config GUI] "


class Settings:
    """
    A class to store settings for the DynamicConfigGUI. This can also be opened as a top-level window.
    """

    def __init__(self) -> None:
        # Settings.
        self.width = 800
        self.height = 600
        self.appearance_mode = "System"
        self.color_theme = "blue"
        self.scale_factor = 1.0

        # GUI.
        self._gui = None

    def gui(self) -> None:
        """
        Open the GUI for changing the settings.
        """
        if self._gui is not None:
            return
        self._gui = ctk.CTkToplevel()
        self._gui.title("Config Utilities Dynamic Config Client Settings")
        self._gui.geometry(f"{400}x{400}")


class DynamicConfigGUI(ctk.CTk):
    """
    A GUI for interacting with dynamic configurations. This GUI allows the user to select a server, a key, and
    edit the configuration for that key. The GUI is designed to be used with a dynamic configuration server
    that can hook into the *_cb functions to send and receive new configurations.
    """

    def __init__(self, settings: Settings = Settings()) -> None:
        super().__init__()

        # Callbacks from the GUI. Optionally set by the invoker.
        self.key_selected_cb = None
        self.server_selected_cb = None
        self.value_changed_cb = None

        # GUI configuration.
        self.settings = settings

        # Data.
        self.current_key = None
        self.current_server = None

        # Initialization.
        self.setup_frame()

    def setup_frame(self):
        # Master.
        self.title("Config Utilities Dynamic Config Client")
        self.geometry(f"{self.settings.width}x{self.settings.height}")

        # Key selection.
        self.key_selection = SelectionDropDown(self, self._key_selected)
        self.key_selection.grid(row=0, column=0, sticky="ew")

        # Settings Button.
        # TODO(lschmid): For now baked into the key selection.
        self.settings_button = ctk.CTkButton(self.key_selection,
                                             text="Settings",
                                             command=self.settings.gui)
        self.settings_button.grid(row=0,
                                  column=3,
                                  sticky="ew",
                                  padx=PAD_X,
                                  pady=PAD_Y)
        self.key_selection.columnconfigure(
            3, weight=0, minsize=self.settings_button.winfo_reqwidth())

        # Config editing.
        self.config_frame = PlainTextConfigFrame(self, self.value_changed_cb)
        self.config_frame.grid(row=1, column=0, sticky="ns", columnspan=2)

        # TODO(lschmid): Consider making this more general, specialized to ROS for now.
        self.server_selection = RosStatusBar(self, self._server_selected)
        self.server_selection.send_cb = self._value_changed_cb
        self.server_selection.grid(row=2, column=0, sticky="ew", columnspan=2)

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

    def set_config_info(self, new_info):
        """
        Update the GUI with new information about the configuration.
        new_info: A dictionary with information about the configuration.
        """
        print(f"Got info: {new_info}")
        pass

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
        self.w_refresh_button = ctk.CTkButton(self,
                                              text="Refresh",
                                              command=self._on_reset_button)
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
        self.w_text = ctk.CTkTextbox(self, wrap=CHAR, width=1000, undo=True)
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
            print(f"{GUI_NAME}Error parsing YAML: {e}")
            return None

    def set_enabled(self, enabled):
        new_state = NORMAL if enabled else DISABLED
        self.w_text.configure(new_state)

    def _on_key_release(self, event):
        # Ctrl + Enter.
        if event.keysym == "Return" and event.state == 20:
            self.send_update_fn()


def main():
    app = DynamicConfigGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
