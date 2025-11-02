# Should be called with arguments like:
#   freecadcmd load_config.py --pass \
#     config_file=path/to/roamr_geometry.yaml \
#     freecad_file=path/to/yaml.FCStd

import os
import FreeCAD as App
import yaml

log_info = App.Console.PrintMessage
log_error = App.Console.PrintError

DEFAULT_UNITS = 'm'
YAML_DIMS_SPREADSHEET_LABEL = 'YamlDims'
GLOBAL_DIMS_SPREADSHEET_LABEL = 'GlobalDims'

def get_last_row(sheet):
  return int(sheet.getUsedRange()[1][1:]) # e.g., 'B10' -> 10
def data_map(sheet): 
  dm = {}
  last_row = get_last_row(sheet) + 1
  for row in range(2, last_row):
    key = sheet.get(F"A{row}")
    dm[key] = row 
  return dm

this_file = f"{os.path.splitext(os.path.basename(__file__))[0]}"
log_info(f"Running {this_file} as {__name__}\n")

if __name__ == this_file:
  log_info(
    "Whoops, we're not running as __main__!\n"
    "Exiting as a workaround...\n"
  )
  sys.exit(0)
elif __name__ == "__main__":

  log_info("\n--- FreeCAD Macro load_config.py starting ---\n")
  
  # Parse command-line arguments and validate
  log_info("\nReceived arguments:\n")
  args = { arg: value for arg, value in 
           (arg.split('=') for arg in sys.argv[1:] if '=' in arg) }
  for k, v in args.items():
      log_info(f"arg: {k} = {v}\n")
  if 'config_file' not in args or 'freecad_file' not in args:
      log_error("\nArguments 'config_file' and 'freecad_file' are required.\n")
      sys.exit(1)
  
  # Load and display the YAML configuration
  with open(args['config_file'], 'r') as f:
    config = yaml.safe_load(f)
  log_info("\nLoaded configuration:\n")
  for k, v in config.items():
      log_info(f"{k}: {v}\n")
 
  # Open the FreeCAD document and get the relevant spreadsheets
  App.openDocument(args['freecad_file'])
  document_name = os.path.splitext(os.path.basename(args['freecad_file']))[0]
  doc = App.getDocument(document_name)
  yaml_sheet = doc.getObjectsByLabel(YAML_DIMS_SPREADSHEET_LABEL)[0]
  global_sheet = doc.getObjectsByLabel(GLOBAL_DIMS_SPREADSHEET_LABEL)[0]
 
  # Nuke existing contents oh YAML sheet to support reducing the number of keys
  yaml_data_map = data_map(yaml_sheet)
  for key in yaml_data_map.keys():
    if key not in config.keys():
      log_info(
        f"\nWARNING: Key '{key}' from {YAML_DIMS_SPREADSHEET_LABEL} not found in "
        f"{config_file}.\n"
      )
      log_info(
        f"It will be removed from {YAML_DIMS_SPREADSHEET_LABEL} "
        f"and will not be unlinked from {GLOBAL_DIMS_SPREADSHEET_LABEL}\n"
      )

  # Get data from valid range in global sheet
  global_data_map = data_map(global_sheet)  
   
  # Write headers 
  yaml_sheet.set('A1', 'Key')
  yaml_sheet.set('B1', 'Value')
  # Write data
  for idx, (k, v) in enumerate(config.items()):
    units = DEFAULT_UNITS
    yaml_sheet.set(f"A{idx+2}", f"\'{k}")
    yaml_sheet.set(f"B{idx+2}", f"{v} {units}")
    yaml_sheet.setAlias(f"B{idx+2}", f"{k}")
    log_info(
      f"Row {idx+2} updated with {k} = {v} [{units}]\n"
    )
    readback_value = yaml_sheet.getContents(f"B{idx+2}")
    log_info(
      f"  Readback value: {readback_value}\n"
    )
    yaml_sheet.recompute()

    # Link in global dims sheet
    if k in global_data_map:
      log_info(f"Updating link to {k} in {GLOBAL_DIMS_SPREADSHEET_LABEL}\n")
      global_row = global_data_map[k]
      global_sheet.set(f"B{global_row}", f"=<<{YAML_DIMS_SPREADSHEET_LABEL}>>.{k}")
      global_sheet.setAlias(f"B{global_row}", f"{k}")
    else:
      log_info(f"Creating new link to {k} in {GLOBAL_DIMS_SPREADSHEET_LABEL}\n")
      global_row = get_last_row(global_sheet) + 1
      global_sheet.set(f"A{global_row}", f"'{k}")
      global_sheet.set(f"B{global_row}", f"=<<{YAML_DIMS_SPREADSHEET_LABEL}>>.{k}")
      global_sheet.setAlias(f"B{global_row}", f"{k}")
    global_sheet.recompute()

  doc.save()
  
  log_info("\n--- FreeCAD Macro load_config.py finished ---\n")
  