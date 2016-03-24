^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joy_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
