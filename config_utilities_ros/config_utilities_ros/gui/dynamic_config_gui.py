import yaml
from flask import render_template, redirect, request, Flask
import uuid
import webbrowser
import re
import copy
import logging


FACTORY_TYPE_PAPRAM_NAME = "type"
NS_SEP = "/"
UNINITIALIZED_VIRTUAL_CONFIG_NAME = (
    "Uninitialized Virtual Config"  # Reserved token for empty virtual configs.
)


def to_yaml(data):
    """
    Convert a dictionary to a YAML string for serialization to the flask app.
    """
    return yaml.dump(data, default_flow_style=True).strip()


def from_yaml(data):
    """
    Convert a YAML string to a dictionary for deserialization from the flask app.
    """
    return yaml.safe_load(data)


class DynamicConfigGUI:
    def __init__(self, app_name=__name__):
        # Additional display containers.
        self.message = ""
        self.errors = []
        self.warnings = []

        # Config data containers
        self._config_data = {}  # The underlying config data, this is the data that is received from the server.
        self._fields = None  # The linearized fields to render in the GUI.

        # Server and key containers.
        self._available_servers_and_keys = {}  # {server: [keys]}
        self._active_server = None
        self._available_keys = []
        self._active_key = None
        self._previously_selected_keys = {}  # {server: key}
        self._is_setup = False

        # Callback functions to be made available to the GUI.
        self.get_available_servers_and_keys_fn = None  # fn(): {servers: [keys]}
        self.set_request_fn = None  # fn(server, key, request_data): response

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
        self._app.add_url_rule(
            "/add_delete", "add_delete", self._add_delete_field, methods=["POST"]
        )
        self._app.add_url_rule("/msg", "msg", self._msg, methods=["POST"])

    def run(self, host="localhost", port=5000, debug=False, open_browser=True):
        """
        Run the Flask app.
        """
        self._is_setup = False
        if not debug:
            log = logging.getLogger("werkzeug")
            log.setLevel(logging.ERROR)
        if open_browser:
            # Open the browser to the GUI.
            webbrowser.open(f"http://{host}:{port}", new=2)
        self._app.run(host=host, port=port, debug=debug, threaded=False)

    # Messaging route for debugging.
    def _msg(self):
        self.message = request.form.to_dict()
        return redirect("/")

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
        # Clear messages.
        self.message = ""
        self.errors = []
        self.warnings = []

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
        self.errors.clear()
        raw_data = request.form.to_dict()
        data, _ = self._parse_form_data(raw_data)
        if not self.errors:
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

    def _add_delete_field(self):
        data = request.form.to_dict()
        action = data.pop("_action")
        field_id = data.pop("_id")

        # TODO(lschmid): In the future first apply all unsaved changes to the config data for a better user experience. This will require updaintg the namespace of the field to delete or add, too.
        # _, changed_keys = self._parse_form_data(data)

        # Find the parent container and final index of the ns.
        ns = field_id.split(NS_SEP)
        curr_data = self._config_data["fields"]
        curr_index = None
        while ns:
            curr_ns = ns.pop(0)
            for i, field in enumerate(curr_data):
                if field["type"] == "field":
                    if field["name"] == curr_ns:
                        if ns:
                            curr_data = field
                        else:
                            curr_index = i
                        break
                elif field["type"] == "config":
                    if field["field_name"] == curr_ns:
                        # If the field is an array or map check the index.
                        if "array_index" in field and int(field["array_index"]) != int(
                            ns[0]
                        ):
                            continue
                        if (
                            "map_config_key" in field
                            and field["map_config_key"] != ns[0]
                        ):
                            continue

                        if "array_index" in field or "map_config_key" in field:
                            ns.pop(0)
                        if ns:
                            curr_data = field["fields"]
                        else:
                            curr_index = i
                        break
                else:
                    raise ValueError(f"Unknown field type: {field['type']}")

        # Backup error handling.
        if curr_index is None:
            self.errors.append(
                f"Internal Error: Could not find field '{field_id}' in config data."
            )
            return redirect("/")

        if action == "add":
            # Add a new field by copying the last field.
            curr_data.insert(curr_index + 1, copy.deepcopy(curr_data[curr_index]))
            # Adjust for new keys.
            if "array_index" in curr_data[curr_index]:
                i = curr_index + 1
                while i < len(curr_data):
                    if (
                        "array_index" in curr_data[i]
                        and curr_data[i]["field_name"]
                        == curr_data[curr_index]["field_name"]
                    ):
                        curr_data[i]["array_index"] += 1
                    i += 1
            if "map_config_key" in curr_data[curr_index]:
                curr_data[curr_index + 1]["map_config_key"] += "-copy"
        elif action == "delete":
            # Adjust for new keys.
            if "array_index" in curr_data[curr_index]:
                i = curr_index + 1
                while i < len(curr_data):
                    if (
                        "array_index" in curr_data[i]
                        and curr_data[i]["field_name"]
                        == curr_data[curr_index]["field_name"]
                    ):
                        curr_data[i]["array_index"] -= 1
                    i += 1
            # Delete the field.
            curr_data.pop(curr_index)

        # Apply the changes from the config to the fields.
        self._parse_fields(replace_yaml=False)
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
        
        # Parse the config data into fields for the GUI.
        self._parse_fields(replace_yaml=True)
        self._parse_errors()

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

    def _parse_fields(self, replace_yaml=True):
        """
        Parse the fields from the YAML input into rows for the GUI.
        """

        def parse_rec(config, indent, prefix):
            fields = []
            prefix_str = "".join([f"{p}{NS_SEP}" for p in prefix])
            for field in config["fields"]:
                if field["type"] == "field":
                    # Default the input info to just strings if no details are present.
                    if "input_info" not in field:
                        field["input_info"] = {"type": "yaml"}

                    # Data for each field (leaves of the config).
                    field["id"] = f"{prefix_str}{field['name']}"
                    field["indent"] = indent
                    if field["input_info"]["type"] == "yaml" and replace_yaml:
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
                    has_subfields = True
                    if "available_types" in field:
                        # Virtual configs
                        conf_data["available_types"] = field["available_types"]
                        if field["name"] == UNINITIALIZED_VIRTUAL_CONFIG_NAME:
                            # Special case for the not set config.
                            has_subfields = False
                    if "array_index" in field:
                        # Array configs
                        conf_data["array_index"] = field["array_index"]
                        conf_data["id"] += f"{NS_SEP}{field['array_index']}"
                        new_prefix.append(field["array_index"])
                    if "map_config_key" in field:
                        # Map configs
                        conf_data["map_config_key"] = field["map_config_key"]
                        conf_data["id"] += f"{NS_SEP}{field['map_config_key']}"
                        new_prefix.append(field["map_config_key"])
                    fields.append(conf_data)

                    # Parse all fields in the config.
                    if has_subfields:
                        fields.extend(parse_rec(field, indent + 1, new_prefix))
                else:
                    raise ValueError(f"Unknown field type: {field['type']}")
            return fields

        self._fields = parse_rec(self._config_data, 0, [])

    def _parse_form_data(self, data):
        """
        Reverse parsing the read form data into yaml to send to the client.
        :param data: The data from the config table in the GUI.
        """

        changed_keys = {}

        def parse_rec(config, prefix, values):
            prefix_str = "".join([f"{p}{NS_SEP}" for p in prefix])
            for field in config["fields"]:
                if field["type"] == "field":
                    # Data for each field (leaves of the config).
                    id = f"{prefix_str}{field['name']}"
                    if id not in data:
                        self.errors.append(
                            f"Internal Error: field '{'.'.join(prefix + [field['name']])}' not found in form data."
                        )
                        continue
                    val = data[id]
                    if field["input_info"]["type"] == "yaml":
                        try:
                            val = from_yaml(val)
                        except Exception as e:
                            self.errors.append(
                                f"Error parsing YAML for field '{'.'.join(prefix + [field['name']])}': {e}"
                            )
                            continue
                    values[field["name"]] = val
                    field["value"] = val
                elif field["type"] == "config":
                    # Sub configs.
                    new_prefix = prefix + [field["field_name"]]
                    val = {}
                    has_subfields = True
                    if "available_types" in field:
                        # Virtual configs
                        # Fix naming if arrays or maps are used.
                        id = f"{prefix_str}{field['field_name']}"
                        if "array_index" in field:
                            id += f"{NS_SEP}{field['array_index']}"
                        elif "map_config_key" in field:
                            id += f"{NS_SEP}{field['map_config_key']}"
                        val[FACTORY_TYPE_PAPRAM_NAME] = data[f"{id}-type"]
                        if field["name"] == UNINITIALIZED_VIRTUAL_CONFIG_NAME:
                            # Special case for the not set config.
                            has_subfields = False
                    if "array_index" in field:
                        # NOTE(lschmid): This assumes that the arrays arrive and are sent ordered.
                        idx = field["array_index"]
                        new_prefix.append(idx)
                        if idx == 0:
                            values[field["field_name"]] = []
                        if has_subfields:
                            values[field["field_name"]].append(
                                parse_rec(field, new_prefix, val)
                            )
                    elif "map_config_key" in field:
                        key = field["map_config_key"]
                        name = field["field_name"]
                        new_prefix.append(key)  # Keep the old key to look up IDs.
                        new_key = str(data[f"{prefix_str}{name}{NS_SEP}{key}-key"])
                        if new_key == "":
                            self.errors.append(
                                f"Error: map key for '{'.'.join(prefix + [name])}' is empty."
                            )
                            continue
                        if new_key != key:
                            ns = prefix_str + name + NS_SEP
                            changed_keys[f"{ns}{key}"] = f"{ns}{new_key}"
                        if name not in values:
                            values[name] = {}
                        if has_subfields:
                            values[name][new_key] = parse_rec(field, new_prefix, val)
                        field["map_config_key"] = new_key
                    else:
                        # Parse all fields in a regular config.
                        values[field["field_name"]] = (
                            parse_rec(field, new_prefix, val) if has_subfields else val
                        )
                else:
                    raise ValueError(f"Unknown field type: {field['type']}")
            return values

        return parse_rec(self._config_data, [], {}), changed_keys

    def _parse_errors(self):
        """
        Parse the error and warning messages from the received config data.
        """
        self.errors = []
        self.warnings = []
        if "error" not in self._config_data:
            return

        data = self._config_data["error"]
        parts = re.split(r"(Warning: |Error: )", data)
        for i in range(1, len(parts), 2):
            severity = parts[i]
            value = parts[i + 1].strip() if i + 1 < len(parts) else ""
            if severity == "Error: ":
                self.errors.append(severity + value)
            elif severity == "Warning: ":
                self.warnings.append(severity + value)


if __name__ == "__main__":
    gui = DynamicConfigGUI()
    gui.run(debug=True, open_browser=True)
