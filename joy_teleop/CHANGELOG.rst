^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joy_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2023-05-04)
------------------
* When allow_multiple_commands is True, Use AND operation with match_command instead of OR operation (`#82 <https://github.com/ros-teleop/teleop_tools/issues/82>`_)
  * Use AND operation with match_command instead of OR operation
  * added allow_multiple_commands parameter to have some control over the joystick combinations
* Contributors: Sai Kishor Kothakota

0.4.1 (2021-05-23)
------------------
* Fix missing exec_depend rosservice
* Contributors: Jeremie Deray

0.4.0 (2020-06-04)
------------------
* Fix iteritems and dependency in python3
* Bump CMake version to avoid CMP0048
* Fix install of python scripts for Noetic
* Python3 compatibility for noetic (`#52 <https://github.com/ros-teleop/teleop_tools/issues/52>`_)
* Contributors: Bence Magyar, Tim Clephas

0.3.1 (2020-02-10)
------------------
* fix python3 compatibility (`#42 <https://github.com/ros-teleop/teleop_tools/issues/42>`_)
* Contributors: Yutaka Kondo

0.3.0 (2019-01-03)
------------------
* Fill in the timestamp of outgoing messages, if applicable.
* add service example
* Add option for persistent service, defaulted false
* Contributors: AndyZe, Jeremie Deray, Bence Magyar

0.2.6 (2018-04-06)
------------------
* Support using buttons and axis in the same message
* Contributors: Tim Clephas

0.2.5 (2017-04-21)
------------------
* Remove duplicate examples, add list ones
* Contributors: Bence Magyar

0.2.4 (2016-11-30)
------------------
* Replace joy_teleop.fill_msg with genpy.message.fill_message_args
* Contributors: Stephen Street

0.2.3 (2016-07-18)
------------------
* Add hello publish to example
* Rename to fix example launch file
* Added example of feature to config file
* Added message_value parameter to specify message content on topics
* PEP8 style stuff
* Fixes bug when keep asking for increments
  would make the goal position grow infinitely instead of be of maximum 'current joint position' + 'increment quantity'
* Contributors: Bence Magyar, Sam Pfeiffer, SomeshDaga

0.2.2 (2016-03-24)
------------------
* Add install rules for example files
* gracefully handle missing joy axes
* Contributors: Bence Magyar, Kopias Peter

0.2.1 (2016-01-29)
------------------
* Add support for services
  it is now possible to asynchronously send service requests on button presses
* Adds queue_size keyword
* Contributors: Bence Magyar, Nils Berg, Enrique Fernandez

0.2.0 (2015-08-03)
------------------
* Add example for incrementer
* Update package.xmls
* Add incrementer_server
* Contributors: Bence Magyar

0.1.2 (2015-02-15)
------------------
* joy_teleop: fix minor typo
* Contributors: G.A. vd. Hoorn

0.1.1 (2014-11-17)
------------------
* Change maintainer
* checks for index out of bounds in buttons list
  `buttons` is a list, not a dict
  Filter out buttons not available
* Check for b in buttons
* Check for IndexError
* joy_teleop: add action server auto-refresh
* Move everything to joy_teleop subfolder
* Contributors: Bence Magyar, Enrique Fernández Perdomo, Paul Mathieu

0.1.0 (2013-11-28)
------------------
* joy_teleop: nice, generic joystick control for ROS
