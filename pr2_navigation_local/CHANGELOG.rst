^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_navigation_local
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.18 (2014-08-25)
-------------------
* catkinize packages
* Add ignores.
* Add missing manifest dependencies.
* Update rviz configurations.
* Updating the rviz configuration files to subscribe to PointCloud2
* Adding a missing dependency on topic tools to pr2_navigation _global _local and _slam
* Making rviz nodes anonymous so that multiple people can launch them on different machines
* Changing rviz configuration files for new pr2_navigation
* Testing on the robot with some updates to make things work with the new local planner
* Switching to use new dwa_local_planner, also removing nav_view launch files
* Running the navigation stack on c2, removing some throttles
* Adding mux to pr2_navigation stack to coordinate with teleop... needs testing on robot
* Modifying pr2_navigation to use the pr2_move_base node to manage the tilt laser instead of the laser_tilt_action script
* Changing wiki markup to html links in manifest and stack xml files.... hopefully that will work with the wiki macros, we'll see
* Updating manifest descriptions
* Adding descriptions of each package to manifest files
* Adding some missing dependencies, also updating the stack description
* Copying pr2_navigation from staging area to stacks section of the repoistory
* Contributors: Austin Hendrix, Eitan Marder-Eppstein, Kei Okada, eitan
