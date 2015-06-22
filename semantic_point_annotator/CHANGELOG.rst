^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semantic_point_annotator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Removed unnecessary package fix
* Contributors: TheDash

0.1.22 (2014-09-08)
-------------------
* Removed dependency on pcl
* Contributors: TheDash

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
* Removing some launch files that we don't need anymore
* Porting the ground removal stuff away from PointCloud to PointCloud2. Still needs testing though.
* Remove deprecated API usage
* eigen updates
* port to eigen3 and remove deprecated use of message size
* Unreal, the code actually depended on reading the parameters every cycle to work because it reset the value of z_threshold_ which got transformed to a different frame every cycle
* Removing reading of parameters from the cloud callback
* Changing in all three files instead of just one for empty cloud... this package is a mess... really should switch to new pcl stufff
* Empty clouds are sometimes expected and should be passed on... not errored on and dropped
* Copying pr2_navigation from staging area to stacks section of the repoistory
* Contributors: Austin Hendrix, Brian Gerkey, Eitan Marder-Eppstein, Kei Okada, Ken Conley, Wim Meeussen, eitan
