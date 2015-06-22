^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_navigation_global
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.27 (2015-06-22)
-------------------
* Merge pull request `#16 <https://github.com/pr2/pr2_navigation/issues/16>`_ from k-okada/remove_build_depend
  we do not need any package during build process
* we do not need any package during build process
* Contributors: Devon Ash, Kei Okada

0.1.26 (2015-02-10)
-------------------

0.1.25 (2015-02-06)
-------------------

0.1.24 (2014-10-15)
-------------------
* Remove mainpage.dox
* Contributors: TheDash

0.1.23 (2014-10-15)
-------------------
* Updated maintainership
* Contributors: TheDash

0.1.22 (2014-09-08)
-------------------

0.1.21 (2014-09-06)
-------------------

0.1.20 (2014-09-06)
-------------------
* 0.1.19
* Added changelogs
* 0.1.18
* Generating changelogs; preparing for release
* Contributors: TheDash

0.1.19 (2014-09-06)
-------------------
* 0.1.18
* Generating changelogs; preparing for release
* Contributors: TheDash

0.1.18 (2014-09-06)
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
