#!/usr/bin/env bash

FREECAD_CMD="freecad"

$FREECAD_CMD freecadcmd "$(dirname $0)/macros/load_config.py" --pass \
  "config_file="$(dirname $0)/geo.yaml"" \
  "freecad_file="$(dirname $0)/yaml.FCStd""
