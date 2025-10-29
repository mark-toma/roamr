# Should be called with arguments like:
#   freecadcmd load_config.py --pass \
#     config_file=path/to/roamr_geometry.yaml \
#     freecad_file=path/to/yaml.FCStd

import os
import FreeCAD
import yaml


DEFAULT_UNITS = 'm'

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

FreeCAD.openDocument(args['freecad_file'])
document_name = os.path.splitext(os.path.basename(args['freecad_file']))[0]
doc = App.getDocument(document_name)
sheet = doc.getObjectsByLabel('Spreadsheet')[0] # Get the first one

# TODO: Nuke existing contents

sheet.set('A1', 'Key')
sheet.set('B1', 'Value')
for idx, (k, v) in enumerate(config.items()):
  units = DEFAULT_UNITS
  sheet.set(f"A{idx+2}", f"\'{k}")
  sheet.set(f"B{idx+2}", f"{v} {units}")
  sheet.setAlias(f"B{idx+2}", f"{k}")
  FreeCAD.Console.PrintMessage(
    f"Row {idx+2} updated with {k} = {v} [{units}]\n"
  )
  readback_value = sheet.getContents(f"B{idx+2}")
  FreeCAD.Console.PrintMessage(
    f"  Readback value: {readback_value}\n"
  )
  
sheet.recompute()
doc.save()