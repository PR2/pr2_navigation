## Overview

This package contains configuration files for the
[move_base](https://index.ros.org/p/move_base) and
[amcl](https://index.ros.org/p/amcl) nodes meant to be run in an
application that requires global navigation with a pre-specified static
map. This package also includes launch files that bring up
[rviz](https://index.ros.org/p/rviz) and [nav_view](nav_view) with
global navigation specific configurations.

## Applications

- **`rviz/rviz_move_base.launch`**: Starts
  [rviz](https://index.ros.org/p/rviz) with a global navigation friendly
  setup.
- **`nav_view/nav_view_move_base.launch`**: Starts
  [nav_view](nav_view) with a global navigation friendly setup.

## Building Blocks

- **`move_base.xml`**: Holds a PR2 specific configuration of the
  [move_base](https://index.ros.org/p/move_base) node designed for
  navigation with a pre-specified static map. Should be run in
  conjunction with the [amcl](https://index.ros.org/p/amcl) and
  [map_server](https://index.ros.org/p/map_server) nodes.
- **`amcl_node.xml`**: Holds a PR2 specific configuration of the
  [amcl](https://index.ros.org/p/amcl) node. Designed to be included by
  applications that wish to run [amcl](https://index.ros.org/p/amcl) on
  the PR2. Assumes a map is available from an instance of the
  [map_server](https://index.ros.org/p/map_server) node.

## Configuration Files

- **`config/base_local_planner_params.yaml`**: Holds parameters for
  the [base_local_planner](https://index.ros.org/p/base_local_planner)
  specific to global navigation on the PR2.
- **`config/global_costmap_params.yaml`**: Holds parameters for the
  global [costmap](https://index.ros.org/p/costmap_2d) used in the
  `move_base.xml` configuration file. This is the costmap used by the
  [navfn](https://index.ros.org/p/navfn) planner, and in this case, is
  initialized by a static map offered by an instance of the
  [map_server](https://index.ros.org/p/map_server) node.
- **`config/local_costmap_params.yaml`**: Holds parameters for the
  local [costmap](https://index.ros.org/p/costmap_2d) used in the
  `move_base.xml` configuration file. This is the costmap used by the
  [base_local_planner](https://index.ros.org/p/base_local_planner)
  planner, which operates in an odometric frame.
