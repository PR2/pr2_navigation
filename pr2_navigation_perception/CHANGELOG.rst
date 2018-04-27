^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_navigation_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.28 (2018-04-27)
-------------------
* Merge pull request `#36 <https://github.com/pr2/pr2_navigation/issues/36>`_ from k-okada/fix_catkin_depends
  fixed CMake files for compile in kinetic
* updated maintainer
* Merge pull request `#27 <https://github.com/pr2/pr2_navigation/issues/27>`_ from v4hn/pr-hydro-fix-broken-filters-plugin
  specify prefix for filter plugins
* Merge pull request `#35 <https://github.com/pr2/pr2_navigation/issues/35>`_ from k-okada/hydro-devel
  use `arg` for all parameters for ground filtering
* fix layout
* enable to use args for all params
* Merge pull request `#28 <https://github.com/pr2/pr2_navigation/issues/28>`_ from v4hn/pr-hydro-filter-machine
  spawn laser filter on c2 (where the rest of the pipeline runs)
* Merge pull request `#31 <https://github.com/pr2/pr2_navigation/issues/31>`_ from mikaelarguedas/update_pluginlib_macros
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* spawn laser filter on c2 (where the rest of the pipeline runs)
  Without this, all tilt laserscans are sent back and forth between
  c1 and c2 for no reason, adding to the general delays...
* specify prefix for filter plugins
  The missing prefix has hazardous consequences due
  to a series of unfortunate events involving the filters and the
  laser_filters package..
* Contributors: Austin, David Feil-Seifer, Kei Okada, Mikael Arguedas, v4hn

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
