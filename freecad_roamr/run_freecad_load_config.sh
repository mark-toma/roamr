#!/usr/bin/env bash

FREECAD_CMD="FreeCAD_1.0.0RC2-conda-Linux-x86_64-py311.AppImage"

$FREECAD_CMD freecadcmd "$(dirname $0)/macros/load_config.py" --pass \
  config_file="$(dirname $0)/geo.yaml" \
  freecad_file="$(dirname $0)/yaml.FCStd"
