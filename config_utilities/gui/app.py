from flask import render_template, request, Flask
import uuid
import yaml
import json
import argparse

def to_yaml(data):
    """
    Convert a dictionary to a YAML string.
    """
    return yaml.dump(data, default_flow_style=False).strip()


class DynamicConfigGUI:
    def __init__(self):
        with open("/home/lukas/khronos_ws/src/config_utilities/config_utilities/gui/test_cfg.yaml", "r") as f:
            self.config_data = yaml.safe_load(f) 
        # For field building.
        self.fields = self.parse_fields(self.config_data)
       
        
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
        print(self.fields)
        return render_template("index.html", config_data=self.fields, config_name=self.config_data["name"])


app = Flask(__name__)
app.secret_key = uuid.uuid4().hex
gui = DynamicConfigGUI()



@app.route("/", methods=['POST', 'GET'])
def index():
    if request.method == 'POST':
        # Handle form submission
        data = request.form
        # Process the data as needed
        print("Form submitted with data:", data)
        # You can also update the GUI with new data if needed
        # gui.config_data.update(data)
        # Render the updated GUI
    return gui.render()

@app.route("/refresh", methods=['POST'])
def refresh():
    """Refresh the form."""
    # This function should be called when the refresh button is clicked.
    # It should update the form with new data.
    print("Refresh button clicked")
    return gui.render()

@app.route("/submit", methods=['POST'])
def submit():
    """Submit the form."""
    data = request.form.to_dict()

    print("Submit button clicked:", data)
    return gui.render()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', '-d', action='store_true')
    parser.add_argument('--port', '-p', default=5000, type=int)
    parser.add_argument('--host', default='0.0.0.0')

    args = parser.parse_args()
    # app.run(args.host, args.port, debug=args.debug)
    app.run(debug=True)