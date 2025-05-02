from flask import request, Flask
import uuid
import argparse
from dynamic_config_gui import DynamicConfigGUI


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
    """ Refresh button clicked. """
    gui.refresh()
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