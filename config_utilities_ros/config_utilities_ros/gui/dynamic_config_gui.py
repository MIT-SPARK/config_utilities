import yaml
from flask import render_template, redirect, request, Flask
import uuid
import webbrowser
import re


def to_yaml(data):
    """
    Convert a dictionary to a YAML string for serialization to the flask app.
    """
    return yaml.dump(data, default_flow_style=True).strip()


class DynamicConfigGUI:
    def __init__(self, app_name=__name__):
        self.message = ""
        self.errors = []
        self.warnings = []
        self._config_data = {}
        self._fields = None
        self._available_servers_and_keys = {}  # {server: [keys]}
        self._active_server = None
        self._available_keys = []
        self._active_key = None
        self._previously_selected_keys = {}  # {server: key}
        self._is_setup = False

        # Callback functions to be made available to the GUI.
        self.get_available_servers_and_keys_fn = None  # fn(): {servers: [keys]}
        self.set_request_fn = None  # fn(server, key, data): response

        # The GUI is a Flask app, setup end points.
        self._app = Flask(app_name)
        self._app.secret_key = uuid.uuid4().hex
        self._app.jinja_env.auto_reload = True
        self._app.add_url_rule("/", "index", self._index, methods=["GET", "POST"])
        self._app.add_url_rule("/refresh", "refresh", self._refresh, methods=["POST"])
        self._app.add_url_rule("/submit", "submit", self._submit, methods=["POST"])
        self._app.add_url_rule(
            "/select", "select", self._select_server_or_key, methods=["POST"]
        )

    # TMP: make default for open_browser true
    def run(self, host="localhost", port=5000, debug=False, open_browser=False):
        """
        Run the Flask app.
        """
        self._is_setup = False
        if open_browser:
            # Open the browser to the GUI.
            webbrowser.open(f"http://{host}:{port}", new=2)
        self._app.run(host=host, port=port, debug=debug, threaded=False)

    # Implementation of the rendered GUI.
    def _index(self):
        """
        Render the index page of the GUI.
        """
        # If required, initialize.
        if not self._is_setup:
            return self._setup()

        return self._render()

    def _refresh(self):
        """
        Refresh the GUI.
        """
        # Update the available servers and keys.
        self._available_servers_and_keys = self.get_available_servers_and_keys_fn()
        self._update_server_and_key_selected()

        # Update the content of the selection.
        self._config_data = {}
        self._request_update()
        return redirect("/")

    def _submit(self):
        """
        Submit the form.
        """
        data = request.form.to_dict()

        # Parse the yaml fields.
        for field in self._fields:
            if field["type"] == "field":
                field_name = field["id"]
                if field_name in data:
                    # Convert the YAML string to a dictionary.
                    try:
                        data[field_name] = yaml.safe_load(data[field_name])
                    except yaml.YAMLError as e:
                        pass

        self._request_update(data)
        return redirect("/")

    def _select_server_or_key(self):
        """
        A server or key is elected in the GUI.
        """
        # Update the active server and key based on the selection.
        data = request.form.to_dict()
        if "server-pane" in data:
            self._active_server = data["server-pane"]
        if "key-pane" in data:
            self._active_key = data["key-pane"]
        self._update_server_and_key_selected()
        self._request_update()
        return redirect("/")

    # Processing functions.
    def _request_update(self, data={}):
        """
        Get the data from the server.
        """
        if self._active_server is None or self._active_key is None:
            return

        # Get the new data from the server.
        self._config_data = self.set_request_fn(
            self._active_server, self._active_key, data
        )
        self._parse_fields()
        self._parse_errors()

        # TMP
        self.message = to_yaml(self._fields)

    def _setup(self):
        """
        Setup the GUI.
        """
        # Verify that all required functions are set.
        if self.get_available_servers_and_keys_fn is None:
            raise ValueError("No function to get available servers and keys set.")
        if self.set_request_fn is None:
            raise ValueError("No function to set the request set.")
        self._is_setup = True
        # Initialize the GUI.
        return self._refresh()

    def _update_server_and_key_selected(self):
        """
        Selection logic to validate the selected server and key.
        """
        # Selection logic for the server.
        if self._active_server not in self._available_servers_and_keys:
            # If the server is no longer available, reset the server and key.
            self._active_server = None
            self._active_key = None
        if self._active_server is None and len(self._available_servers_and_keys) > 0:
            # If no server is selected, select the first available server.
            self._active_server = list(self._available_servers_and_keys.keys())[0]

        # Selection logic for the key.
        self._available_keys = self._available_servers_and_keys.get(
            self._active_server, []
        )
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
            config_name=self._config_data.get("name", ""),
            available_servers=list(self._available_servers_and_keys.keys()),
            available_keys=self._available_keys,
            active_server=self._active_server,
            active_key=self._active_key,
            message=self.message,
            error_message=self.errors,
            warning_message=self.warnings,
        )

    def _parse_fields(self):
        """
        Parse the fields from the YAML input into rows for the GUI.
        """

        def parse_rec(config, indent, prefix):
            fields = []
            prefix_str = "".join([f"{p}/" for p in prefix])
            for field in config["fields"]:
                if field["type"] == "field":
                    # Data for each field (leaves of the config).
                    field["id"] = f"{prefix_str}{field['name']}"
                    field["indent"] = indent
                    if field["input_info"]["type"] == "yaml":
                        field["value"] = to_yaml(field["value"])
                        field["default"] = to_yaml(field["default"])
                    fields.append(field)
                elif field["type"] == "config":
                    # Sub configs.
                    new_prefix = prefix + [field["field_name"]]
                    conf_data = {
                        "id": f"{prefix_str}{field['field_name']}",
                        "name": field["field_name"],
                        "config_name": field["name"],
                        "indent": indent,
                        "type": "config",
                    }
                    if "array_index" in field:
                        conf_data["array_index"] = field["array_index"]
                        conf_data["id"] += f"/{field['array_index']}"
                        new_prefix.append(field["array_index"])
                    if "map_config_key" in field:
                        conf_data["map_config_key"] = field["map_config_key"]
                        conf_data["id"] += f"/{field['map_config_key']}"
                        new_prefix.append(field["map_config_key"])
                    fields.append(conf_data)

                    # Parse all fields in the config.
                    fields.extend(parse_rec(field, indent + 1, new_prefix))
                else:
                    raise ValueError(f"Unknown field type: {field['type']}")
            return fields

        self._fields = parse_rec(self._config_data, 0, [])

    def _parse_errors(self):
        """
        Parse the error and warning messages from the config data.
        """
        self.errors = []
        self.warnings = []
        if "error" not in self._config_data:
            return

        data = self._config_data["error"]
        parts = re.split(r"(Warning: |Error: )", data)
        # parts will be like ['', 'Error: ', 'some error', 'Warning: ', 'some warning']
        for i in range(1, len(parts), 2):
            label = parts[i]
            value = parts[i + 1].strip() if i + 1 < len(parts) else ""
            if label == "Error: ":
                self.errors.append(label + value)
            elif label == "Warning: ":
                self.warnings.append(label + value)


if __name__ == "__main__":
    gui = DynamicConfigGUI()
    gui.run(debug=True, open_browser=False)
