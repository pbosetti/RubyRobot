Controller programs
===================

At the moment a stub class for IK testing is implemented.

Available Libraries
-------------------

1. lib/ik.rb implements the inverse kinematic of a planar 3R robot and of a Puma 560 4R robot.
2. lib/viewer.rb implements an OpneGL viewer for robot configuration. The viewer can be initialized with an array of Body objects, opportunely configured in conformance with the Denavit-Hartenberg convention. See viewer_test.rb for an usage example

To Do
-----

* Direct kinematics
* Perhaps rethink it on a more general basis
* Testing for speed and convert to C++