# ROAMeR Description

Robot description package for ROAMeR including URDF and demo with RViz and joint publisher.

## Usage

Launch the demo. This will bring up RViz and a joint state publisher GUI to visualize and move the joints of the robot.

```bash
ros2 run roamr_description demo.launch.py
```

See [demo.launch.py](./launch/demo.launch.py) for optional launch arguments if you'd like to disable default behavior.

For use in a robotic application, you're expected to create a `robot_state_publisher` using `xacro` as performed in this demo.

For URDF development, you can use the [generate_urdf.sh](./urdf/generate_urdf.sh) script to generate and check the URDF.

## Development Constraints

The URDF consumes geometry from [roamr_geometry.yaml](./config/roamr_geometry.yaml) which must be kept in sync with the following:

- Kinematics calculations
  - Forward and inverse kinematics
  - Behavior controllers (e.g. for obstable avoidance)
- CAD geometry
  - Design data
  - Visualization data

At present time, this YAML file is the source of truth for this data. All other ROS packages and CAD solutions that relate this data should consume this file for specification of this data. Any future geometry data needs should be evaluated for extension of this file or refactoring of the concept of source of truth.
