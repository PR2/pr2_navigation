^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_navigation_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* 0.1.18
* Generating changelogs; preparing for release
* Contributors: TheDash

0.1.18 (2014-09-06)
-------------------
* catkinize packages
* Add ignores.
* Adding / to topics so that things work with a mux implementation that doesn't resolve topic names fully. These slashes should be removed once mux is fixed so that this node can be pushed into a namespace.
* Adding mux to pr2_navigation stack to coordinate with teleop... needs testing on robot
* Adding descriptions of each package to manifest files
* Copying pr2_navigation from staging area to stacks section of the repoistory
* Contributors: Austin Hendrix, Kei Okada, eitan
