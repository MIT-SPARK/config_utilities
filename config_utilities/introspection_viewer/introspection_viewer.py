#!/usr/bin/env python3
import os
import json
from flask import Flask, render_template
import uuid
import webbrowser
import logging
import argparse


class IntrospectionViewer:
    def __init__(self, data, app_name=__name__):
        self._data = data  # Introspection data as json.
        # The GUI is a Flask app, setup end points.
        self._app = Flask(app_name)
        self._app.secret_key = uuid.uuid4().hex
        self._app.add_url_rule("/", "/", self._index, methods=["GET", "POST"])

    def run(self, host="localhost", port=5000, debug=False, open_browser=True):
        """
        Run the Flask app.
        """
        if not debug:
            log = logging.getLogger("werkzeug")
            log.setLevel(logging.ERROR)
        if open_browser:
            # Open the browser to the GUI.
            webbrowser.open(f"http://{host}:{port}", new=2)
        self._app.run(host=host, port=port, debug=debug, threaded=False)

    def _index(self):
        return render_template("index.html", data=self._data)


def main():
    parser = argparse.ArgumentParser(
        description="Viewer for introspection data logged by config_utilities. To log introspection, run your executable using config_utilities with the --config-utilities-introspect/-i option.")
    parser.add_argument("--data", default=None,
                        help="Path to the introspection data.json file to view")
    parser.add_argument("--host", default="localhost",
                        help="Host to run the Flask app on")
    parser.add_argument("--port", type=int, default=5000,
                        help="Port to run the Flask app on")
    parser.add_argument("--debug", action="store_true",
                        help="Run the Flask app in debug mode")
    parser.add_argument("--open-browser", action="store_true",
                        help="Open the browser to the GUI")
    args = parser.parse_args()

    # Load data
    if args.data is None:
        candidate_paths = [
            os.path.join(os.path.curdir, "data.json"),
            os.path.join(os.path.curdir,
                         "config_introspection_output", "data.json"),
            os.path.join(os.path.dirname(__file__), 'data.json')]
        for path in candidate_paths:
            if os.path.exists(path):
                args.data = path
                break
        if args.data is None:
            raise FileNotFoundError(
                "No introspection data file specified and no default found. Please specify the path to the introspection data.json file using the --data argument.")

    if not os.path.exists(args.data):
        raise FileNotFoundError(
            f"Introspection data file not found: '{args.data}'")
    with open(args.data, 'r') as f:
        data = json.load(f)

    viewer = IntrospectionViewer(data)
    print(
        f"Running Introspection Viewer for '{os.path.abspath(args.data)}' at 'http://{args.host}:{args.port}'.")
    viewer.run(host=args.host, port=args.port,
               debug=args.debug, open_browser=args.open_browser)


if __name__ == '__main__':
    main()
