^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eband_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2013-08-12)
------------------
* stopped publishing empty bubble heading message. closes `#6 <https://github.com/utexas-bwi/eband_local_planner/issues/6>`_
* reenabled holonomic drive control. still untested since groovy/hydro migration. closes `#2 <https://github.com/utexas-bwi/eband_local_planner/issues/2>`_

0.1.0 (2013-07-10)
------------------
* add include directory to installation
* removed redundant file
* introduced some hysteresis between turning to final orientation and
  turning towards the next bubble. closes `#3 <https://github.com/utexas-bwi/eband_local_planner/issues/3>`_
* returning trajectory controller's opinion on whether the goal has been reached or not. closes `#4 <https://github.com/utexas-bwi/eband_local_planner/issues/4>`_
* removed unnecesary print statements
* some debug statements to help find issues with hydro-devel costmap
* fixed all runtime loading errors in process of catkinization
* removing unneeded/untested launch files. I'll add working examples later as the package sees a public release.
* catkinizing eband_local_planner against the hydro-devel branch of navigation
* removing standalone follow trajectory ndoe
* promoting eband local planner to separate repository in preparation for catkinization
* some improvements to navigation
* fixed a bug in the eband trajectory controller
* Merge branch 'master' of github.com:utexas-bwi/segbot_apps
* in-place rotation at goal now supported
* merged goal tolerance parameters between local planner and trajectory controller.
* add launch for e-band navigation
* changed distance check to not include orienation. I am not even sure why orientation was being used to check overlap.
* a couple more parameters + better velocity limit checking
* fixed for the regular nav stack launch file as well. closes `#1 <https://github.com/utexas-bwi/eband_local_planner/issues/1>`_
* hmm not sure why this file was here
* fix for the eband costmap having an incorrect topic. `#1 <https://github.com/utexas-bwi/eband_local_planner/issues/1>`_
* updated launch file to use any visualization configuration + reorganized eband configuration file
* fixed a problem with requesting obstacle distances at the edge of the costmap - these regions are now assumed to be free (optimism) - removes problem with the global plan being regenerated over and over again as new parts of the global plan were converted to the band
* hack for global planner not catching up in case of an unexpected obstacle snapping band
* changed INFO statements to DEBUG
* checking in new parameters for the eband local planner
* inital differential drive trajectory controller - looks pretty good. needs a bit more code improvent, dynamic reconfigure and stricter obstacle testing
* Revert "checking in Paul's changes to get eband local planner to work with our differential drive robot". I am seeing some problems with his changes, will apply by hand.
  This reverts commit c925f7e1d5bebbf8c102a2292429749da84562a9.
* checking in Paul's changes to get eband local planner to work with our differential drive robot
* added git ignore settings
* changes to make eband_local_planner compile with navigation 1.9 + groovy
* copy of eband_local_planner from navigation_experimental.
* removed some unnecessary launch files and added an rviz configuration + launch file for testing autonomous navigation
* basic amcl + move base demo works (but is not very good)
* removed old ens basement maps from the repo
* removed joy gmapping file - joystick control not directly supported
* removed redundant sensor files (moved to segbot_sensors)
* initial port of of navigation and controller code from the svn repository
* first commit
