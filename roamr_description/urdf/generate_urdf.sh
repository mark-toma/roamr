#!/usr/bin/env bash
#
# Copyright 2025 Mark Tomaszewski
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Use this script to generate the URDF from XACRO and check its validity.

cd "$(dirname "$0")"
echo "Generating URDF from XACRO..."
ros2 run xacro xacro ./roamr_robot.urdf.xacro \
  geom_file:="$(pwd)/../config/roamr_geometry.yaml" > roamr_robot.urdf

echo ""
echo "Checking URDF validity..."
check_urdf roamr_robot.urdf > roamr_robot.urdf.check
res=$?

echo ""
echo "See roamr_robot.urdf[.check] for details"
if [[ 0 == $res ]]; then
    echo "SUCCESS: URDF is valid"
else
    echo "FAILURE: URDF is not valid"
fi