#!/usr/bin/env bash

FREECAD_CMD="freecad"
FREECAD_ROAMR_PATH="$(dirname $0)/../"
FREECAD_FILE="cad/roamr_dims.FCStd"
YAML_FILE="geo.yaml"

cd $FREECAD_ROAMR_PATH

$FREECAD_CMD freecadcmd "macros/load_config.py" --pass \
  "config_file=$YAML_FILE" \
  "freecad_file=$FREECAD_FILE"
