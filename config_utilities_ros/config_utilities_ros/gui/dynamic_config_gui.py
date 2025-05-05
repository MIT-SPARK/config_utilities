import yaml
from flask import render_template, request, Flask
import uuid


def to_yaml(data):
    """
    Convert a dictionary to a YAML string for serialization to the flask app.
    """
    return yaml.dump(data, default_flow_style=True).strip()


class DynamicConfigGUI:
    def __init__(self, app_name=__name__):
        # TMP
        with open(
            "/home/lukas/khronos_ws/src/config_utilities/config_utilities_ros/config_utilities_ros/gui/test_cfg.yaml",
            "r",
        ) as f:
            self._config_data = yaml.safe_load(f)

        # For field building.
        self._fields = self._parse_fields(self._config_data)
        self._available_servers_and_keys = {}  # {server: [keys]}
        self._active_server = None
        self._available_keys = []
        self._active_key = None
        self._previously_selected_keys = {}  # {server: key}

        # TMP
        self._available_servers_and_keys = {
            "Server1": ["Key1", "Key2"],
            "Server2": ["Key3"],
        }
        self._update_server_and_key_selected()

        # Callback functions of the GUI.
        self.on_server_key_selecetd = None  # fn(server, key): None

        # The GUI is a Flask app, setup end points.
        self._app = Flask(app_name)
        self._app.secret_key = uuid.uuid4().hex
        self._app.config["TEMPLATES_AUTO_RELOAD"] = True
        self._app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0
        self._app.jinja_env.auto_reload = True
        self._app.add_url_rule("/", "index", self._index, methods=["GET", "POST"])
        self._app.add_url_rule("/refresh", "refresh", self._refresh, methods=["POST"])
        self._app.add_url_rule("/submit", "submit", self._submit, methods=["POST"])
        self._app.add_url_rule(
            "/select", "select", self._select_server_or_key, methods=["POST"]
        )

    def run(self, **kwargs):
        """
        Run the Flask app.
        """
        self._app.run(**kwargs)

    # Interfaces to the GUI for the client.
    def set_config_data(self, data):
        """
        Set the configuration data for the GUI.
        """
        self._config_data = data
        self._fields = self._parse_fields(data)
        return self._render()

    def set_available_servers_and_keys(self, servers_and_keys):
        """
        Set the available servers for the GUI.
        """
        print("Got new servers and keys!")
        self._available_servers_and_keys = servers_and_keys
        self._update_server_and_key_selected()
        return self._render()

    # Implementation of the rendered GUI.
    def _index(self):
        """
        Render the index page of the GUI.
        """
        if request.method == "POST":
            data = request.form
            print("Form submitted with data:", data)

        return self._render()

    def _refresh(self):
        """
        Refresh the GUI.
        """
        print("Refresh button clicked")
        # TODO: Add refresh logic here.
        return self._render()

    def _submit(self):
        """
        Submit the form.
        """
        data = request.form.to_dict()

        print("Submit button clicked:", data)
        return self._render()

    def _select_server_or_key(self):
        """
        Select a server or key.
        """
        # Update the active server and key based on the selection.
        data = request.form.to_dict()
        if "server-pane" in data:
            self._active_server = data["server-pane"]
        if "key-pane" in data:
            self._active_key = data["key-pane"]
        self._update_server_and_key_selected()
        print("Active selection:", self._active_server, self._active_key)

        # Callback to the client if set.
        if self.on_server_key_selecetd is not None:
            self.on_server_key_selecetd(self._active_server, self._active_key)
        return self._render()

    # Processing functions.
    def _update_server_and_key_selected(self):
        # Selection logic for the server.
        if self._active_server not in self._available_servers_and_keys:
            self._active_server = None
            self._active_key = None
        if self._active_server is None and len(self._available_servers_and_keys) > 0:
            self._active_server = list(self._available_servers_and_keys.keys())[0]

        # Selection logic for the key.
        self._available_keys = self._available_servers_and_keys[self._active_server]
        if self._active_key not in self._available_keys:
            self._active_key = None
        if self._active_key is None:
            # Try to lookup previously selected keys.
            previous_key = self._previously_selected_keys.get(self._active_server, None)
            if previous_key in self._available_keys:
                self._active_key = previous_key
            elif len(self._available_keys) > 0:
                self._active_key = self._available_keys[0]

        # Cache the previously selected key.
        if self._active_server is not None and self._active_key is not None:
            self._previously_selected_keys[self._active_server] = self._active_key

    def _render(self):
        """
        Render the GUI, including the form, selectors, and buttons.
        """
        self._app.jinja_env.cache = {}
        return render_template(
            "index.html",
            config_data=self._fields,
            config_name=self._config_data["name"],
            available_servers=list(self._available_servers_and_keys.keys()),
            available_keys=self._available_keys,
            active_server=self._active_server,
            active_key=self._active_key,
        )

    def _parse_fields(self, config_data):
        """
        Parse the fields from the YAML input into rows for the GUI.
        """

        # This function should parse the fields from the YAML file and return them.
        # For now, we will just return a placeholder.
        def parse(config, indent, prefix):
            fields = []
            prefix_str = "".join([f"{p}/" for p in prefix])
            for field in config["fields"]:
                if field["type"] == "field":
                    field["id"] = f"{prefix_str}{field['name']}"
                    field["indent"] = indent
                    # For now no specialization for non-trivial types.
                    if field["input_info"]["type"] == "yaml":
                        field["value"] = to_yaml(field["value"])
                        field["default"] = to_yaml(field["default"])
                    fields.append(field)
                elif field["type"] == "config":
                    fields.append(
                        {
                            "id": f"{prefix_str}{field['field_name']}",
                            "name": field["field_name"],
                            "config_name": field["name"],
                            "indent": indent,
                            "type": "config",
                        }
                    )
                    # Parse all fields in the config.
                    fields.extend(
                        parse(field, indent + 1, prefix + [field["field_name"]])
                    )
                else:
                    raise ValueError(f"Unknown field type: {field['type']}")
            return fields

        return parse(config_data, 0, [])


if __name__ == "__main__":
    gui = DynamicConfigGUI()
    gui.run(debug=True)
