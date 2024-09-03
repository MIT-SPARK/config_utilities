#!/usr/bin/env python3
from tkinter import *
import customtkinter as ctk
import yaml

ctk.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"
PAD_X = 10
PAD_Y = 10
GUI_NAME = "[Config Utilities Dynamic Config GUI] "

"""
TODO(lschmid): Type Info known limitations:
- Make sure namespaces are handled correctly.
- Enable updating from values without re-building the entire GUI
- Add registered type information for virtual configs.
- Add support for config maps and vectors.
- Find a good interface and visualization for int/float types (consider exteded type information).
"""


class Settings:
    """
    A class to store settings for the DynamicConfigGUI. This can also be opened as a top-level window.
    """

    METHOD_OPTIONS = [ "Plain YAML", "Type Info (Experimental)"]
    APPEARANCE_OPTIONS = ["System", "Light", "Dark"]
    COLOR_THEME_OPTIONS = ["blue", "green", "dark-blue"]
    SACLE_MIN = 0.5
    SACLE_MAX = 2.0

    def __init__(self) -> None:
        # Settings.
        self.width = 800
        self.height = 600
        self.appearance_mode = self.APPEARANCE_OPTIONS[0]
        self.color_theme = self.COLOR_THEME_OPTIONS[0]
        self.method = self.METHOD_OPTIONS[0]
        self.scale_factor = 1.0

        # GUI.
        self._gui = None
        self._master = None

    def validate(self) -> None:
        """
        Validate the settings.
        """
        if not self.appearance_mode in self.APPEARANCE_OPTIONS:
            self.appearance_mode = self.APPEARANCE_OPTIONS[0]
        if not self.color_theme in self.COLOR_THEME_OPTIONS:
            self.color_theme = self.COLOR_THEME_OPTIONS[0]
        if not self.method in self.METHOD_OPTIONS:
            self.method = self.METHOD_OPTIONS[0]
        if self.scale_factor < self.SACLE_MIN:
            self.scale_factor = self.SACLE_MIN
        if self.scale_factor > self.SACLE_MAX:
            self.scale_factor = self.SACLE_MAX

    def apply(self) -> None:
        """
        Apply the settings.
        """
        self.validate()
        ctk.set_appearance_mode(self.appearance_mode)
        ctk.set_default_color_theme(self.color_theme)
        ctk.set_widget_scaling(self.scale_factor)

    def gui(self) -> None:
        """
        Open the GUI for changing the settings.
        """
        self._gui = ctk.CTkToplevel()
        self._gui.title("Config Utilities Dynamic Config Client Settings")
        self._gui.geometry(f"{400}x{400}")

        # Add all settings.
        current_row = 0

        # Entry tool.
        self.w_method_label = ctk.CTkLabel(self._gui, text="UI Method:", anchor="w")
        self.w_method_label.grid(
            row=current_row, column=0, padx=PAD_X, pady=PAD_Y, sticky="nsw"
        )
        self.w_method = ctk.CTkOptionMenu(
            self._gui, values=self.METHOD_OPTIONS, command=self._method_cb
        )
        self.w_method.set(self.method)
        self.w_method.grid(row=current_row, column=1, padx=PAD_X, pady=PAD_Y)
        current_row += 1

        # Appearance.
        self.w_appearance_label = ctk.CTkLabel(
            self._gui, text="Appearance Mode:", anchor="w"
        )
        self.w_appearance = ctk.CTkOptionMenu(
            self._gui, values=self.APPEARANCE_OPTIONS, command=self._appearance_cb
        )
        self.w_appearance.set(self.appearance_mode)
        self.w_appearance_label.grid(
            row=current_row, column=0, padx=PAD_X, pady=PAD_Y, sticky="nsw"
        )
        self.w_appearance.grid(row=current_row, column=1, padx=PAD_X, pady=PAD_Y)
        current_row += 1

        # Color Theme.
        self.w_color_theme_label = ctk.CTkLabel(
            self._gui, text="Color Theme:", anchor="w"
        )
        self.w_color_theme = ctk.CTkOptionMenu(
            self._gui,
            values=self.COLOR_THEME_OPTIONS,
            command=self._color_theme_cb,
        )
        self.w_color_theme.set(self.color_theme)
        self.w_color_theme_label.grid(
            row=current_row, column=0, padx=PAD_X, pady=PAD_Y, sticky="nsw"
        )
        self.w_color_theme.grid(row=current_row, column=1, padx=PAD_X, pady=PAD_Y)
        current_row += 1

        # Scaling.
        self.w_scaling_label = ctk.CTkLabel(self._gui, text="UI Scaling:", anchor="w")
        self.w_scaling_label.grid(
            row=current_row, column=0, padx=PAD_X, pady=PAD_Y, sticky="nsw"
        )
        self.w_scaling = ctk.CTkComboBox(
            self._gui,
            values=["50", "75%", "90%", "100%", "110%", "125%", "150%", "200%"],
            command=self._scaling_cb,
        )
        self.w_scaling.bind("<KeyRelease>", self._scaling_key_cb)
        self.w_scaling.grid(row=current_row, column=1, padx=PAD_X, pady=PAD_Y)
        self.w_scaling.set(f"{int(self.scale_factor * 100)}%")
        current_row += 1

        # Apply and cancel Button.
        self.w_apply_button = ctk.CTkButton(
            self._gui, text="Apply", command=self._gui.destroy, anchor="c"
        )
        self.w_apply_button.grid(
            row=current_row, column=0, columnspan=2, padx=PAD_X, pady=PAD_Y, sticky="se"
        )

        # Formatting,
        self._gui.rowconfigure(current_row, weight=1)
        self._gui.columnconfigure(1, weight=1)

    def _appearance_cb(self, new_appearance_mode):
        self.appearance_mode = new_appearance_mode
        ctk.set_appearance_mode(new_appearance_mode)

    def _scaling_cb(self, new_scaling):
        self.scale_factor = float(new_scaling.replace("%", "")) / 100
        self.scale_factor = min(self.SACLE_MAX, max(self.SACLE_MIN, self.scale_factor))
        self.w_scaling.set(f"{int(self.scale_factor * 100)}%")
        ctk.set_widget_scaling(self.scale_factor)

    def _scaling_key_cb(self, event):
        if event.keysym == "Return":
            self._scaling_cb(self.w_scaling.get())

    def _color_theme_cb(self, new_color_theme):
        self.color_theme = new_color_theme
        ctk.set_default_color_theme(new_color_theme)

    def _method_cb(self, new_method):
        self.method = new_method
        if self._master is not None:
            self._master.setup_config_frame()


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
        self.settings._master = self

        # Data.
        self.current_key = None
        self.current_server = None
        self.current_values = None
        self.current_info = None

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
        self.settings_button = ctk.CTkButton(
            self.key_selection, text="Settings", command=self.settings.gui
        )
        self.settings_button.grid(row=0, column=3, sticky="ew", padx=PAD_X, pady=PAD_Y)
        self.key_selection.columnconfigure(
            3, weight=0, minsize=self.settings_button.winfo_reqwidth()
        )

        # Config editing.
        self.setup_config_frame()

        # TODO(lschmid): Consider making this more general, specialized to ROS for now.
        self.server_selection = RosStatusBar(self, self._server_selected)
        self.server_selection.send_cb = self._value_changed_cb
        self.server_selection.grid(row=2, column=0, sticky="ew", columnspan=2)

        self.rowconfigure(
            [0, 2], minsize=self.key_selection.winfo_reqheight(), pad=PAD_Y, weight=0
        )
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1, pad=PAD_X)

    def setup_config_frame(self):
        if self.settings.method == "Type Info (Experimental)":
            self.config_frame = TypeInfoConfigFrame(self, self._value_changed_cb)
        else:
            # Default to plain text if unsupported.
            self.config_frame = PlainTextConfigFrame(self, self._value_changed_cb)
        self.config_frame.grid(row=1, column=0, sticky="nsew", columnspan=2)

        if self.current_values is not None:
            self.config_frame.set_config(self.current_values)
        if self.current_info is not None:
            self.config_frame.set_config_info(self.current_info)

    # Interfaces for outside interaction with the GUI.
    def set_keys(self, keys):
        self.key_selection.set_keys(keys)

    def set_servers(self, servers):
        self.server_selection.set_keys(servers)

    def set_config(self, new_values):
        self.current_values = new_values
        self.config_frame.set_config(new_values)

    def set_config_info(self, new_info):
        self.current_info = new_info
        self.config_frame.set_config_info(new_info)

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
        # TMP
        print("Sending Values: ")
        print(self.config_frame.get_config())
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
        self.w_label.grid(row=0, column=0, padx=PAD_X, pady=PAD_Y, sticky="nsw")
        self.w_dropdown = ctk.CTkOptionMenu(
            self, dynamic_resizing=True, command=self._on_change
        )
        self.w_dropdown.grid(row=0, column=1, sticky="nsew", padx=PAD_X, pady=PAD_Y)
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
            self, text="Refresh", command=self._on_refresh_button
        )
        self.w_refresh_button.grid(
            row=0, column=3, padx=PAD_X, pady=PAD_Y, sticky="nse"
        )
        self.w_send_button = ctk.CTkButton(
            self, text="Send", command=self._on_send_button
        )
        self.w_send_button.grid(row=0, column=4, padx=PAD_X, pady=PAD_Y, sticky="nse")
        self.columnconfigure(2, weight=1)
        self.columnconfigure([0, 1, 3, 4], weight=0)

    def _on_refresh_button(self):
        if self.refresh_cb is not None:
            self.refresh_cb()

    def _on_send_button(self):
        if self.send_cb is not None:
            self.send_cb()


class ConfigFrame:
    """
    Interface class for configuration editing.
    """

    def __init__(self, master, send_update_fn=None):
        super().__init__(master)
        self.send_update_fn = send_update_fn

    def set_config(self, new_config):
        pass

    def set_config_info(self, new_info):
        pass

    def get_config(self):
        return {}

    def set_enabled(self, enabled):
        pass


class PlainTextConfigFrame(ConfigFrame, ctk.CTkFrame):
    """
    A frame to enable editing a config in plain YAML.
    """

    def __init__(self, master, send_update_fn=None):
        super().__init__(master, send_update_fn)
        self.w_text = ctk.CTkTextbox(
            self, wrap=CHAR, width=1000, undo=True, font=ctk.CTkFont(size=14)
        )
        self.w_text.pack(fill=BOTH, expand=True, padx=PAD_X, pady=PAD_Y)
        self.w_text.bind("<KeyRelease>", self._on_key_release)

    def set_config(self, new_config):
        self.w_text.delete("0.0", END)
        self.w_text.insert("0.0", yaml.dump(new_config))

    def get_config(self):
        try:
            return yaml.load(self.w_text.get("1.0", END), Loader=yaml.FullLoader)
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


class TypeInfoConfigFrame(ConfigFrame, ctk.CTkScrollableFrame):
    """
    A frame to restrict editing a config based on type information.
    """

    INDENT = 4  # Number of spaces per indent level.
    ROW_HEIGHT = 18
    CORNER_RADIUS = 2

    def __init__(self, master, send_update_fn=None):
        super().__init__(master, send_update_fn)

        self.current_row = 0
        self.current_indent = 0
        self.current_ns = []
        self.get_value_fns = {}
        self.set_value_fns = {}
        self.widgets = []

    def set_config_info(self, new_info):
        # TODO(lschmid): Check if only values need to be updated.
        self.clear_config_ui()
        self.build_config_ui(new_info)

    def get_config(self):
        yaml_data = {}
        for key, fn in self.get_value_fns.items():
            ns = key.split("/")
            curr_node = yaml_data
            for n in ns[:-1]:
                if not n:
                    continue
                if not n in curr_node:
                    curr_node[n] = {}
                curr_node = curr_node[n]
            curr_node[ns[-1]] = fn()
        return yaml_data
    
    # Build the UI for a config info.
    def clear_config_ui(self):
        for widget in self.widgets:
            widget.destroy()
        self.current_row = 0
        self.current_indent = 0
        self.current_ns = []
        self.widgets.clear()
        self.get_value_fns.clear()

    def build_config_ui(self, config_info):
        # Header row.
        name = config_info["name"] if "name" in config_info else "Unknown Config"
        self.w_header_name = ctk.CTkLabel(self, text=f"{name}:", anchor="w")
        self.w_header_name.grid(row=self.current_row, column=0, sticky="nsw", padx=PAD_X)
        self.w_header_value = ctk.CTkLabel(
            self,
            text="Value:",
            anchor="w",
        )
        self.w_header_value.grid(row=self.current_row, column=1, sticky="nsw", padx=PAD_X)
        self.w_header_default = ctk.CTkLabel(
            self,
            text="Default:",
            anchor="w",
        )
        self.w_header_default.grid(row=self.current_row, column=2, sticky="nsw", padx=PAD_X)
        self.current_row += 1

        # Build the config.
        if "fields" in config_info:
            for info in config_info["fields"]:
                self.build(info)

        # Configure columns.
        self.rowconfigure([i for i in range(self.current_row + 1)], weight=0, pad=0)
        self.columnconfigure(0, weight=0, pad=0)
        self.columnconfigure(1, weight=1, pad=0)
        self.columnconfigure(2, weight=0, pad=0)

    def build(self, info):
        # Build configs or fields.
        if not "type" in info:
            return
        if info["type"] == "config":
            self.build_config(info)
        elif info["type"] == "field":
            self.build_field(info)

    def build_config(self, info):
        name = info["field_name"] if "field_name" in info else "Unknown Field"
        type = f" [{info['name']}]" if "name" in info else ""
        self.add_label(f"{name}{type}:")
        self.current_row += 1
        if not "fields" in info:
            return

        self.current_indent += 1
        self.current_ns.append(name)
        for field in info["fields"]:
            self.build(field)
        self.current_indent -= 1
        self.current_ns.pop()

    def build_field(self, info):
        name = info["name"] if "name" in info else "Unknown Field"
        unit = f" [{info['unit']}]" if "unit" in info else ""
        default = info["default"] if "default" in info else "Unknown Default"
        self.add_label(f"{name}{unit}:")
        self.add_value_entry(info)
        self.add_default(str(default))
        self.current_row += 1

    def add_label(self, text):
        label = ctk.CTkLabel(
            self,
            text=" " * self.current_indent * self.INDENT + text,
            height=self.ROW_HEIGHT,
            anchor="w",
        )
        label.grid(row=self.current_row, column=0, sticky="nsw", pady=0)
        self.widgets.append(label)

    def add_default(self, text):
        label = ctk.CTkTextbox(
            self,
            fg_color="light gray",
            text_color="gray",
            border_spacing=0,
            corner_radius=self.CORNER_RADIUS,
            height=self.ROW_HEIGHT,
        )
        label.insert("1.0", text)
        label.configure(state=DISABLED)
        label.grid(row=self.current_row, column=2, sticky="nsw", pady=0)
        self.widgets.append(label)

    def add_value_entry(self, info):
        input_type = "yaml"
        param_name = "/".join(self.current_ns) + "/" + info["name"]
        if "input_info" in info and "type" in info["input_info"]:
            input_type = info["input_info"]["type"]
        if input_type == "bool":
            self.add_bool_value_entry(info, param_name)
        elif input_type == "int":
            self.add_numeric_value_entry(info, param_name, True)
        elif input_type == "float":
            self.add_numeric_value_entry(info, param_name, False)
        elif input_type == "string":
            self.add_string_value_entry(info, param_name)
        elif input_type == "options":
            self.add_options_value_entry(info, param_name)
        else:
            self.add_yaml_value_entry(info, param_name)

    def add_yaml_value_entry(self, info, param_name):
        widget = ctk.CTkTextbox(
            self,
            wrap=CHAR,
            undo=True,
            height=self.ROW_HEIGHT,
            border_spacing=0,
            corner_radius=self.CORNER_RADIUS,
        )
        widget.insert("0.0", yaml.dump(info["value"], default_flow_style=True).rstrip())
        widget.grid(row=self.current_row, column=1, sticky="nsew", pady=0, padx=PAD_X)
        self.widgets.append(widget)
        self.get_value_fns[param_name] = lambda: self.get_yaml_value(widget)

    def add_bool_value_entry(self, info, param_name):
        widget = ctk.CTkCheckBox(
            self, height=self.ROW_HEIGHT, corner_radius=self.CORNER_RADIUS, text=""
        )
        widget.grid(row=self.current_row, column=1, sticky="nsew", pady=0, padx=PAD_X)
        if info["value"]:
            widget.select()
        else:
            widget.deselect()
        self.widgets.append(widget)
        self.get_value_fns[param_name] = lambda: "true" if widget.get() else "false"

    def add_numeric_value_entry(self, info, param_name, is_int = True):
        frame = ctk.CTkFrame(self, height=self.ROW_HEIGHT, border_width=0)
        frame.grid(row=self.current_row, column=1, sticky="nsew", pady=0, padx=PAD_X)
        # Value.
        w2 = ctk.CTkTextbox(
            frame,
            height=self.ROW_HEIGHT,
            border_spacing=0,
            corner_radius=self.CORNER_RADIUS,
            width=50,
        )
        w2.insert("0.0", info["value"])
        w2.grid(row=0, column=0, sticky="nsew", pady=0)
        self.widgets.append(w2)
        self.get_value_fns[param_name] = lambda: w2.get("1.0", END)
        frame.columnconfigure(1, weight=0)
        # Constraints.
        min_val = None
        max_val = None
        if "min" in info["input_info"]:
            min_val = info["input_info"]["min"]
        if "max" in info["input_info"]:
            max_val = info["input_info"]["max"]
        if min_val is not None:
            w3 = ctk.CTkLabel(
                frame,
                text=f"{'(' if 'lower_exclusive' in info['input_info'] else '['}{min_val}",
                anchor="w",
                height=self.ROW_HEIGHT,
            )
            w3.grid(row=0, column=1, sticky="nsw", pady=0)
            frame.columnconfigure(1, weight=0)
            self.widgets.append(w3)

        if min_val is not None and max_val is not None:
            slid_min = min_val
            if is_int and 'lower_exclusive' in info['input_info']:
                slid_min += 1
            slid_max = max_val
            if is_int and 'upper_exclusive' in info['input_info']:
                slid_max -= 1
            w4 = ctk.CTkSlider(
                frame,
                from_=slid_min,
                to=slid_max,
                orientation=HORIZONTAL,
                corner_radius=self.CORNER_RADIUS,
                command=lambda _: self.sync_text_to_slider(w2,w4, is_int)
            )
            w2.bind("<KeyRelease>", lambda _: self.sync_slider_to_text(w4, w2))
            if is_int and max_val - slid_min < 100:
                w4.configure(number_of_steps=slid_max - min_val + 1)
            w4.set(info["value"])
            w4.grid(row=0, column=2, sticky="nsew", pady=0)
            self.widgets.append(w4)
            frame.columnconfigure(2, weight=1)

        if max_val is not None:
            w5 = ctk.CTkLabel(
                frame,
                text=f"{', ' if min_val is None else ''}{max_val}{')' if 'upper_exclusive' in info['input_info'] else ']'}",
                anchor="w",
                height=self.ROW_HEIGHT,
            )
            col = 3 if min_val is not None else 1
            w5.grid(row=0, column=col, sticky="nsw", pady=0)
            frame.columnconfigure(col, weight=0)
            self.widgets.append(w5)
        self.widgets.append(frame)

    def sync_slider_to_text(self, slider, text):
        try:
            value = float(text.get("1.0", END))
            slider.set(value)
        except ValueError:
            pass

    def sync_text_to_slider(self, text, slider, is_int):
        text.delete("1.0", END)
        new_text = f"{slider.get():.0f}" if is_int else f"{slider.get():.2f}"
        text.insert("1.0", new_text)

    def add_float_value_entry(self, info, param_name):
        self.add_int_value_entry(info, param_name)

    def add_string_value_entry(self, info, param_name):
        widget = ctk.CTkTextbox(
            self,
            wrap=CHAR,
            undo=True,
            height=self.ROW_HEIGHT,
            border_spacing=0,
            corner_radius=self.CORNER_RADIUS,
        )
        widget.insert("0.0", info["value"])
        widget.grid(row=self.current_row, column=1, sticky="nsew", pady=0, padx=PAD_X)
        self.widgets.append(widget)
        self.get_value_fns[param_name] = lambda: widget.get("1.0")

    def add_options_value_entry(self, info, param_name):
        options = []
        if "options" in info["input_info"]:
            for o in info["input_info"]["options"]:
                options.append(str(o))
        if str(info["value"]) not in options:
            options.append(str(info["value"]))
        widget = ctk.CTkOptionMenu(
            self,
            values=options,
            corner_radius=self.CORNER_RADIUS,
            height=self.ROW_HEIGHT,
        )
        widget.set(info["value"])
        widget.grid(row=self.current_row, column=1, sticky="nsew", pady=0, padx=PAD_X)
        self.widgets.append(widget)
        self.get_value_fns[param_name] = lambda: widget.get()

    def get_yaml_value(self, widget):
        try:
            return yaml.load(widget.get("1.0", END), Loader=yaml.FullLoader)
        except yaml.YAMLError as e:
            print(f"{GUI_NAME}Error parsing YAML: {e}")
            return None


def main():
    app = DynamicConfigGUI()
    # TEST
    data = yaml.load(
        "{'type': 'config', 'name': 'MyConfig', 'fields': [{'type': 'field', 'name': 'i', 'value': 100, 'default': 100, 'input_info': {'type': 'int', 'min': 0, 'max': 2147483647, 'lower_exclusive': True}}, {'type': 'field', 'name': 'distance', 'unit': 'm', 'value': 42, 'default': 42, 'input_info': {'type': 'float', 'min': 0, 'max': 100}}, {'type': 'field', 'name': 'b', 'value': True, 'default': True, 'input_info': {'type': 'bool'}}, {'type': 'field', 'name': 'vec', 'value': [1, 2, 3], 'default': [1, 2, 3], 'input_info': {'type': 'yaml'}}, {'type': 'field', 'name': 'map', 'value': {'a': 1, 'b': 2, 'c': 3}, 'default': {'a': 1, 'b': 2, 'c': 3}, 'input_info': {'type': 'yaml'}}, {'type': 'field', 'name': 'mat', 'value': [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'default': [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 'input_info': {'type': 'yaml'}}, {'type': 'field', 'name': 'my_enum', 'value': 'A', 'default': 'A', 'input_info': {'type': 'options', 'options': ['A', 'B', 'C']}}, {'type': 'config', 'name': 'SubConfig', 'field_name': 'sub_config', 'fields': [{'type': 'field', 'name': 'f', 'value': 1.1, 'default': 1.1, 'input_info': {'type': 'options', 'options': [0, 1.1, 2.2, 3.3]}}, {'type': 'field', 'name': 's', 'value': 'test', 'default': 'test', 'input_info': {'type': 'string'}}]}]}",
        Loader=yaml.FullLoader,
    )
    app.set_config_info(data)
    app.mainloop()


if __name__ == "__main__":
    main()
