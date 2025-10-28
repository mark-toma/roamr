# Should be called with arguments like:
#   freecadcmd load_config.py --pass \
#     config_file=path/to/roamr_geometry.yaml \
#     freecad_file=path/to/yaml.FCStd

import os
import FreeCAD
import yaml


args = { arg: value for arg, value in 
         (arg.split('=') for arg in sys.argv[1:] if '=' in arg) }

FreeCAD.Console.PrintMessage("\nReceived arguments:\n")
for k, v in args.items():
    FreeCAD.Console.PrintMessage(f"arg: {k} = {v}\n")

# TODO: Validate arguments, show usage, etc.

# Load and display the YAML configuration
with open(args['config_file'], 'r') as f:
  config = yaml.safe_load(f)

FreeCAD.Console.PrintMessage("\nLoaded configuration:\n")
for k, v in config.items():
    FreeCAD.Console.PrintMessage(f"{k}: {v}\n")

# Load the FreeCAD document
# Validate FreeCAD document contents (e.g. Spreadsheet exists)

# For each YAML config item
# - Validate value, if not numeric
#   - Error out
# - Calculate full context key
#   - This is concatenation of all keys, joined by dunderbars
#   - e.g. config['key1']['key2'] -> key1__key2
# - Search for existing key in Spreadsheet
# - If key exists, compare values
#   - If values differ
#     - Notify
#     - Update value
# - If key does not exist, create entry
#   - Notify creation
#   - Add key and value
#   - Add alias for use in FreeCAD expressions
#
# Note: This does not maintain deprecated keys.

