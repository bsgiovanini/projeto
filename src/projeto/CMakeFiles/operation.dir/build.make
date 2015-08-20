# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bruno/ros_workspace/src/projeto

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bruno/ros_workspace/src/projeto

# Include any dependencies generated for this target.
include CMakeFiles/operation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/operation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/operation.dir/flags.make

CMakeFiles/operation.dir/src/operation.o: CMakeFiles/operation.dir/flags.make
CMakeFiles/operation.dir/src/operation.o: src/operation.cpp
CMakeFiles/operation.dir/src/operation.o: manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /home/bruno/workshop/ardrone_autonomy/manifest.xml
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/operation.dir/src/operation.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/operation.dir/src/operation.o: /home/bruno/workshop/ardrone_autonomy/msg_gen/generated
CMakeFiles/operation.dir/src/operation.o: /home/bruno/workshop/ardrone_autonomy/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/projeto/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/operation.dir/src/operation.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/operation.dir/src/operation.o -c /home/bruno/ros_workspace/src/projeto/src/operation.cpp

CMakeFiles/operation.dir/src/operation.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/operation.dir/src/operation.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/projeto/src/operation.cpp > CMakeFiles/operation.dir/src/operation.i

CMakeFiles/operation.dir/src/operation.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/operation.dir/src/operation.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/projeto/src/operation.cpp -o CMakeFiles/operation.dir/src/operation.s

CMakeFiles/operation.dir/src/operation.o.requires:
.PHONY : CMakeFiles/operation.dir/src/operation.o.requires

CMakeFiles/operation.dir/src/operation.o.provides: CMakeFiles/operation.dir/src/operation.o.requires
	$(MAKE) -f CMakeFiles/operation.dir/build.make CMakeFiles/operation.dir/src/operation.o.provides.build
.PHONY : CMakeFiles/operation.dir/src/operation.o.provides

CMakeFiles/operation.dir/src/operation.o.provides.build: CMakeFiles/operation.dir/src/operation.o

# Object files for target operation
operation_OBJECTS = \
"CMakeFiles/operation.dir/src/operation.o"

# External object files for target operation
operation_EXTERNAL_OBJECTS =

bin/operation: CMakeFiles/operation.dir/src/operation.o
bin/operation: /usr/local/lib/liboctomap.so
bin/operation: /usr/local/lib/liboctomath.so
bin/operation: CMakeFiles/operation.dir/build.make
bin/operation: CMakeFiles/operation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/operation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/operation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/operation.dir/build: bin/operation
.PHONY : CMakeFiles/operation.dir/build

CMakeFiles/operation.dir/requires: CMakeFiles/operation.dir/src/operation.o.requires
.PHONY : CMakeFiles/operation.dir/requires

CMakeFiles/operation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/operation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/operation.dir/clean

CMakeFiles/operation.dir/depend:
	cd /home/bruno/ros_workspace/src/projeto && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/ros_workspace/src/projeto /home/bruno/ros_workspace/src/projeto /home/bruno/ros_workspace/src/projeto /home/bruno/ros_workspace/src/projeto /home/bruno/ros_workspace/src/projeto/CMakeFiles/operation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/operation.dir/depend

