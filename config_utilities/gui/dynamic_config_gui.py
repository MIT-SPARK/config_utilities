import yaml
from flask import render_template

def to_yaml(data):
    """
    Convert a dictionary to a YAML string.
    """
    return yaml.dump(data, default_flow_style=True).strip()
    

class DynamicConfigGUI:
    def __init__(self):
        # TMP
        with open("/home/lukas/khronos_ws/src/config_utilities/config_utilities/gui/test_cfg.yaml", "r") as f:
            self.config_data = yaml.safe_load(f) 
        # For field building.
        self.fields = self.parse_fields(self.config_data)
        self.available_servers = []
        self.active_server = None
        self.available_keys = []
        self.active_key = None

    # Interfaces to the GUI for the client.
    def set_config_data(self, config):
        """
        Set the configuration data for the GUI.
        """
        self.config_data = config
        self.fields = self.parse_fields(config)
        return self.render()
    
    def set_available_servers(self, servers):
        """
        Set the available servers for the GUI.
        """
        self.available_servers = servers
        return self.render()
    
    def set_available_keys(self, keys):
        """
        Set the available keys for the GUI.
        """
        self.available_keys = keys
        return self.render()
    
    # Interfaces to the GUI from the rendered HTML.
    def refresh(self):
        """
        Refresh the GUI.
        """
        # Refresh available servers, keys, and config data.
        return self.render()


    # Processing functions.
    def parse_fields(self, config_data):
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
                    fields.append( {
                        "id": f"{prefix_str}{field['field_name']}",
                        "name": field["field_name"] ,
                        "config_name": field["name"],
                        "indent": indent,
                        "type": "config",
                    })
                    # Parse all fields in the config.
                    fields.extend(parse(field, indent +1, prefix + [field["field_name"]]))
                else:
                    raise ValueError(f"Unknown field type: {field['type']}")
            return fields

        return parse(config_data, 0, [])

    def render(self):
        """
        Render the GUI, including the form, selectors, and buttons.
        """
        return render_template("index.html", config_data=self.fields, config_name=self.config_data["name"])