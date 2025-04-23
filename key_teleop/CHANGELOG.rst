^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package key_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2025-04-16)
------------------
* Default to TwistStamped (`#90 <https://github.com/ros-teleop/teleop_tools/issues/90>`_)
* Contributors: Bence Magyar

1.7.0 (2024-11-06)
------------------

1.6.0 (2024-10-01)
------------------

1.5.1 (2024-09-02)
------------------

1.5.0 (2023-11-01)
------------------
* replace deprecated dash by underscore (`#85 <https://github.com/ros-teleop/teleop_tools/issues/85>`_)
* Contributors: Noel Jiménez García

1.4.0 (2023-03-28)
------------------

1.3.0 (2022-11-23)
------------------
* Fix some warnings from tests.
  In here are some flake8 fixes and fixes to the joy_teleop tests
  now that some of the error messages have changed.
* added ability to use twiststamped
* add ci & lint
* Add QoS profile to key_teleop publisher
* Contributors: Andreas Klintberg, Chris Lalancette, Kazunari Tanaka, nfry321

1.2.1 (2020-10-29)
------------------

1.2.0 (2020-10-16)
------------------

1.1.0 (2020-04-21)
------------------

1.0.2 (2020-02-10)
------------------

1.0.1 (2019-09-18)
------------------
* Fix install rules and dashing changes (`#38 <https://github.com/ros-teleop/teleop_tools/issues/38>`_)
  * fix ament indexing
  * fix package resource files
  * add tk depenndency
  * add check for param index-ability
  * data files are now package agnostic
  Signed-off-by: Ted Kern <ted.kern@canonical.com>
* Contributors: Ted Kern

1.0.0 (2019-09-10)
------------------
* ROS2 port (`#35 <https://github.com/ros-teleop/teleop_tools/issues/35>`_)
  * ROS2 port
  * key_teleop pkg format 3
  * port teleop_tools_msgs
  * key_teleop catch KeyboardInterrupt
  * port mouse_teleop
  * add key_teleop.yaml
  * prepare tests
  * add xmllint test
  * fix xmllint tests
  * simplify joy_teleop retrieve_config
  * remove useless class KeyTeleop
  * Fixes for dynamic topic joy publishers
  - match_command() now compares button array length to the max
  deadman button index (apples to apples)
  - match_command function now checks if any of the deadman buttons
  are depressed before returning a match
  - properly handle a std_msgs/msg/Empty 'message_value' by not
  attempting to access its value
  - utilizes iter-items to correctly index into the config dict
  for 'axis_mappings''s 'axis' and 'button' values
  - set_member() now splits according to a dash (-) rather than a
  periond (.) to be consistent with ros2 param parsing & example yaml
  - adds the correct name to setup.py for test_key_teleop.py test
  * reduce copy/pasta
* Contributors: Jeremie Deray

0.3.0 (2019-01-03)
------------------

0.2.6 (2018-04-06)
------------------

0.2.5 (2017-04-21)
------------------

0.2.4 (2016-11-30)
------------------

0.2.3 (2016-07-18)
------------------

0.2.2 (2016-03-24)
------------------

0.2.1 (2016-01-29)
------------------

0.2.0 (2015-08-03)
------------------
* Update package.xmls
* Contributors: Bence Magyar

0.1.2 (2015-02-15)
------------------

0.1.1 (2014-11-17)
------------------
* Change maintainer
* Remove rosbuild legacy
* Merge key_teleop into teleop_tools
* Contributors: Bence Magyar, Paul Mathieu
