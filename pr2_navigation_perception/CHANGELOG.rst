^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_navigation_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.27 (2015-06-22)
-------------------

0.1.26 (2015-02-10)
-------------------
* Updated maintanership
* Contributors: TheDash

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
* Removed unnecesary dependency on deprecated pcl
* Contributors: TheDash

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
* Fixes to keep costmap clean of artifacts in voxel grid.
  Added an interpolation filter to the tilt laser scan used for clearing.  This lets the laser clear
  voxels which are no longer filled but which are in front of transparent objects which don't give a
  proper laser reading.
  Changed the timing of when in the tilt scan the laser data is used for marking.  Previously it was
  prone to marking voxels at the top edge of the tilt scan which would never get cleared out.  Now
  the filter stops accepting tilt scan data .05 seconds before it gets to the top of the tilt range,
  and the problem does not seem to happen.  This throws away a very thin wedge of scan data which does
  not seem to hurt performance.
* The last touches for moving pr2_navigation over to PointCloud2
* Fixing some dependencies
* Removing unused laser footprint filter
* Since the footprint extractor is a filter, the ROS message types must be used rather than the pcl subscriber
* Porting the footprint filter from a PointCloud to PointCloud2
* Switching to the un-deprecated pluginlib macro
* Fixes for fuerte
* Running the navigation stack on c2, removing some throttles
* Removing a package dependency on pr2_tilt_laser_profile that wasn't needed
* Adding the laser_tilt_controller_filter package as a dependency
* Adding the downscan filter to the tilt laser filters
* Changing throttles to operate at 1.0 hz
* Modifying pr2_navigation to use the pr2_move_base node to manage the tilt laser instead of the laser_tilt_action script
* Adding descriptions of each package to manifest files
* Copying pr2_navigation from staging area to stacks section of the repoistory
* Contributors: Austin Hendrix, Dave Hershberger, Eitan Marder-Eppstein, Kei Okada, eitan
