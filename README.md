# TerpCopter [![Build Status](https://travis-ci.org/UMD-AMAV/TerpCopter.svg?branch=master)](https://travis-ci.org/UMD-AMAV/TerpCopter)
University of Maryland's Autopilot Software for Autonomous-Micro Aerial Vehicles (A-MAV) Competition hosted by American Helicopter Society (AHS)

## Reference Guides

Have a look at [*C++ Cheat Sheet*](https://www.pa.msu.edu/~duxbury/courses/phy480/Cpp_refcard.pdf) , [*ROS Cheat Sheet*](https://mirror.umd.edu/roswiki/attachments/de/ROScheatsheet.pdf) and [*Git Cheat Sheet*](https://services.github.com/kit/downloads/github-git-cheat-sheet.pdf) for quick reference

## Contribution Rules

Kindly follow the conventions listed in the [*C++ Style Guide*](https://google.github.io/styleguide/cppguide.html), [*ROS Style Guide*](https://github.com/ethz-asl/ros_best_practices/wiki) and [*Git Style Guide*](https://github.com/agis-/git-style-guide/blob/master/README.md) when contributing to this repository

The master branch has been protected and so you cannot push commits to this branch directly and the only way to contribute to the master branch is via pull requests. Whenever you are planning to contribute, create a new branch in this repository ( eg. feature/obstacle_avoidance ) and push all your commits to this branch. When the feature is complete and has been tested , create a pull request to merge your branch with the master and make sure to write a good description that explains how your code works and what the results of those tests are. Once this is done, some of the team members and/or the code owners of the file you modified should review your code and suggest modifications to improve functionality/ enforce style guide/ remove bugs etc. Once all the appropriate changes have been made and they have been approved by the reviewers, tag @lkumar93 in your pull request to merge your branch with the master.

# Using TerpCopter

## Start TerpCopter
Run `roslaunch terpcopter_controller controller.launch`

To start with simulator, add in `sitl:=1` argument
When starting with simulator, PX4 console will pop up in an xterm
window. You can send commands to copter from PX4 console and do
PX4 specific debugging there.

## Creating a node
There are two types of nodes, the TCNode and the TCManagerNode.
The base classes for both are in the `terpcopter_common` package.
A TCNode is used for any TerpCopter computing that doesn't require
starting another node. A TCManagerNode is a specific example of a
TCNode. It is capable of starting further nodes. This creates a
node tree. Each TCManagerNode and corresponding child nodes forms
what is often referred to as a `system` in the code.

To use the TCNode and TCManagerNode, simply add the following include
statements in your C++ code:
    #include <terpcopter_common/common.h>
    #include <terpcopter_common/system.h>
    // To include TCNode
    #include <terpcopter_common/tc_node.h>
    // To include TCManagerNode
    #include <terpcopter_common/tc_manager_node.h>

See the `terpcopter_sensor_processor/terpcopter_localizer.<cpp|h>`
files for a good example of a TCNode implementation. To implement a
TCNode, you must define the method:
    virtual int8_t check_health();

See the `terpcopter_controller/terpcopter_manager.<cpp|h>` and
`terpcopter_simulator/px4_sitl_manager.<cpp|h>` for two examples
of a TCManagerNode implementation. To implement TCManagerNode, you
must define the method:
    int initialize_systems(std::vector<std::string> &sys_list);
    
## Start Vision Node
Run `rosrun terpcopter_vision terpcopter_vision_server`

Vision node currently operates as a server so that the function/service can be called over the network whenever required and not continuously running so as to reduce CPU usage. It currently advertises two services `detect_boundary` and `detect_home_base` which perform boundary detection and home base detection respectively. An example client node that calls these services and prints their output can be run using the command `rosrun terpcopter_vision terpcopter_vision_client`.

## Dependencies
Check `terpcopter_description/config/dependencies.txt` for
dependencies. This list is incomplete and should be updated every time
you add a new dependency.

## License

[GPL-3.0](https://github.com/UMD-AMAV/TerpCopter/blob/master/LICENSE)
