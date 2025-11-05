# TODO:

## Spec

- Research:
  - Typical \<robot\>_sim package capability and interdependencies with description and other packages (control, driver, etc)
  - Develop conceptual sim architecture against physical robot
  - Detail the types of additions to be made to "sim-ify" the URDF, e.g. mass props, actuation/sensing elements, etc
  - Detail the list of package nodes/launch files required to meet the package capability with integration to other packages

## Impl

- Macro-ify the current top level XACRO in description for inclusion as a macro here
- Implement top level XACRO URDF files in both description and sim packages
- Develop the sim-ified XACRO to extend the sim with actuation/sensing capability
  - This should be direct prescription of kinematic states with zero-order dynamics with idealized kinematic behavior
  - This also includes mass properties to make system ODEs nonsingular for integration, but not to be physically valid
- Research typical capability for a sim support package
- Create a launch file for demo or otherwise basic functionality
- Spawn the sim-ified URDF in Gazebo with configured environment simulation 
- Add some mechanism to command controllable robot states
  - Hopefully this doesn't need to be provided by a roamr_control package as this isn't the intended solution
  - This should be part of the initial research before starting

## Test

- Do we do those now? Maybe. Maybe not yet.