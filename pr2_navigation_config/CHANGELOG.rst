^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_navigation_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.27 (2015-06-22)
-------------------

0.1.26 (2015-02-10)
-------------------

0.1.25 (2015-02-06)
-------------------

0.1.24 (2014-10-15)
-------------------

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
* Fixes to keep costmap clean of artifacts in voxel grid.
  Added an interpolation filter to the tilt laser scan used for clearing.  This lets the laser clear
  voxels which are no longer filled but which are in front of transparent objects which don't give a
  proper laser reading.
  Changed the timing of when in the tilt scan the laser data is used for marking.  Previously it was
  prone to marking voxels at the top edge of the tilt scan which would never get cleared out.  Now
  the filter stops accepting tilt scan data .05 seconds before it gets to the top of the tilt range,
  and the problem does not seem to happen.  This throws away a very thin wedge of scan data which does
  not seem to hurt performance.
* pr2_navigation_config: Lowered min_rot_vel from 0.4 to 0.3 to fix an interaction between min_rot_vel and (acc_lim_theta * sim_period) which prevents it from turning in place from a stop.
* The last touches for moving pr2_navigation over to PointCloud2
* Adding an oscillation timeout of 10 seconds by default to pr2_navigation
* Changing the size of the voxel grid to allow the robot to navigate when the spine is all the way up
* Switching to use new dwa_local_planner, also removing nav_view launch files
* Changing acc_limit to acc_lim
* Changing wiki markup to html links in manifest and stack xml files.... hopefully that will work with the wiki macros, we'll see
* Adding descriptions of each package to manifest files
* Copying pr2_navigation from staging area to stacks section of the repoistory
* Contributors: Austin Hendrix, Dave Hershberger, Eitan Marder-Eppstein, Kei Okada, eitan
