^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_navigation_self_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.28 (2018-04-27)
-------------------
* Merge pull request `#36 <https://github.com/pr2/pr2_navigation/issues/36>`_ from k-okada/fix_catkin_depends
  fixed CMake files for compile in kinetic
* updated maintainer
* fixed CMake files for compile in kinetic
* Merge pull request `#24 <https://github.com/pr2/pr2_navigation/issues/24>`_ from wkentaro/self_filter-timestamp
  Set correct timestamp for self filtered cloud
* Set correct timestamp for self filtered cloud
  This is needed because pcl drops some value of timestamp.
  So pcl::fromROSMsg and pcl::toROSMsg does not work to get correct timestamp.
* Contributors: David Feil-Seifer, Devon Ash, Kei Okada, Kentaro Wada

0.1.27 (2015-06-22)
-------------------
* Support collada dae mesh file as well as stl files
* Install self_filter binary to the package_bin directory
* Contributors: Ryohei Ueda, aginika

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
* use less bullet and more tf
* Navigation is just going to use XYZ points from now on
* Porting pr2_navigation_self filter from PointCloud to PointCloud2. Still need to test.
* Fixing linking issue on oneiric
* Fixes for fuerte
* Remove deprecated API usage
* Adding descriptions of each package to manifest files
* Copying pr2_navigation from staging area to stacks section of the repoistory
* Contributors: Austin Hendrix, Brian Gerkey, Eitan Marder-Eppstein, Kei Okada, Vincent Rabaud, eitan
