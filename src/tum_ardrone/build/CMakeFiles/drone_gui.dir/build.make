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
CMAKE_SOURCE_DIR = /home/bruno/ros_workspace/src/tum_ardrone

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bruno/ros_workspace/src/tum_ardrone/build

# Include any dependencies generated for this target.
include CMakeFiles/drone_gui.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/drone_gui.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/drone_gui.dir/flags.make

CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: CMakeFiles/drone_gui.dir/flags.make
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: ../src/UINode/main_GUI.cpp
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: ../manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /home/bruno/workshop/ardrone_autonomy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/rosservice/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /home/bruno/workshop/ardrone_autonomy/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /home/bruno/workshop/ardrone_autonomy/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o -c /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/main_GUI.cpp

CMakeFiles/drone_gui.dir/src/UINode/main_GUI.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_gui.dir/src/UINode/main_GUI.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/main_GUI.cpp > CMakeFiles/drone_gui.dir/src/UINode/main_GUI.i

CMakeFiles/drone_gui.dir/src/UINode/main_GUI.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_gui.dir/src/UINode/main_GUI.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/main_GUI.cpp -o CMakeFiles/drone_gui.dir/src/UINode/main_GUI.s

CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.requires:
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.requires

CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.provides: CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.requires
	$(MAKE) -f CMakeFiles/drone_gui.dir/build.make CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.provides.build
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.provides

CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.provides.build: CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o

CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: CMakeFiles/drone_gui.dir/flags.make
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: ../src/UINode/tum_ardrone_gui.cpp
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: ../manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /home/bruno/workshop/ardrone_autonomy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/rosservice/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /home/bruno/workshop/ardrone_autonomy/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /home/bruno/workshop/ardrone_autonomy/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o -c /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/tum_ardrone_gui.cpp

CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/tum_ardrone_gui.cpp > CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.i

CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/tum_ardrone_gui.cpp -o CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.s

CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.requires:
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.requires

CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.provides: CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.requires
	$(MAKE) -f CMakeFiles/drone_gui.dir/build.make CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.provides.build
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.provides

CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.provides.build: CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o

CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: CMakeFiles/drone_gui.dir/flags.make
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: ../src/UINode/RosThread.cpp
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: ../manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /home/bruno/workshop/ardrone_autonomy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/rosservice/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /home/bruno/workshop/ardrone_autonomy/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /home/bruno/workshop/ardrone_autonomy/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/RosThread.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/drone_gui.dir/src/UINode/RosThread.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/drone_gui.dir/src/UINode/RosThread.o -c /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/RosThread.cpp

CMakeFiles/drone_gui.dir/src/UINode/RosThread.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_gui.dir/src/UINode/RosThread.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/RosThread.cpp > CMakeFiles/drone_gui.dir/src/UINode/RosThread.i

CMakeFiles/drone_gui.dir/src/UINode/RosThread.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_gui.dir/src/UINode/RosThread.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/RosThread.cpp -o CMakeFiles/drone_gui.dir/src/UINode/RosThread.s

CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.requires:
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.requires

CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.provides: CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.requires
	$(MAKE) -f CMakeFiles/drone_gui.dir/build.make CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.provides.build
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.provides

CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.provides.build: CMakeFiles/drone_gui.dir/src/UINode/RosThread.o

CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: CMakeFiles/drone_gui.dir/flags.make
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: ../src/UINode/PingThread.cpp
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: ../manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/std_srvs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /home/bruno/workshop/ardrone_autonomy/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/rosservice/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /home/bruno/workshop/ardrone_autonomy/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /home/bruno/workshop/ardrone_autonomy/srv_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
CMakeFiles/drone_gui.dir/src/UINode/PingThread.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/drone_gui.dir/src/UINode/PingThread.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/drone_gui.dir/src/UINode/PingThread.o -c /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/PingThread.cpp

CMakeFiles/drone_gui.dir/src/UINode/PingThread.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_gui.dir/src/UINode/PingThread.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/PingThread.cpp > CMakeFiles/drone_gui.dir/src/UINode/PingThread.i

CMakeFiles/drone_gui.dir/src/UINode/PingThread.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_gui.dir/src/UINode/PingThread.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/PingThread.cpp -o CMakeFiles/drone_gui.dir/src/UINode/PingThread.s

CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.requires:
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.requires

CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.provides: CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.requires
	$(MAKE) -f CMakeFiles/drone_gui.dir/build.make CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.provides.build
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.provides

CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.provides.build: CMakeFiles/drone_gui.dir/src/UINode/PingThread.o

CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o: CMakeFiles/drone_gui.dir/flags.make
CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o: src/UINode/moc_tum_ardrone_gui.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o -c /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_tum_ardrone_gui.cxx

CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_tum_ardrone_gui.cxx > CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.i

CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_tum_ardrone_gui.cxx -o CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.s

CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.requires:
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.requires

CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.provides: CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.requires
	$(MAKE) -f CMakeFiles/drone_gui.dir/build.make CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.provides.build
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.provides

CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.provides.build: CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o

CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o: CMakeFiles/drone_gui.dir/flags.make
CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o: src/UINode/moc_RosThread.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o -c /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_RosThread.cxx

CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_RosThread.cxx > CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.i

CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_RosThread.cxx -o CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.s

CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.requires:
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.requires

CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.provides: CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.requires
	$(MAKE) -f CMakeFiles/drone_gui.dir/build.make CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.provides.build
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.provides

CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.provides.build: CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o

CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o: CMakeFiles/drone_gui.dir/flags.make
CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o: src/UINode/moc_PingThread.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o -c /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_PingThread.cxx

CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_PingThread.cxx > CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.i

CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_PingThread.cxx -o CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.s

CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.requires:
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.requires

CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.provides: CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.requires
	$(MAKE) -f CMakeFiles/drone_gui.dir/build.make CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.provides.build
.PHONY : CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.provides

CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.provides.build: CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o

ui_tum_ardrone_gui.h: ../src/UINode/tum_ardrone_gui.ui
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ui_tum_ardrone_gui.h"
	/usr/bin/uic-qt4 -o /home/bruno/ros_workspace/src/tum_ardrone/build/ui_tum_ardrone_gui.h /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/tum_ardrone_gui.ui

src/UINode/moc_tum_ardrone_gui.cxx: ../src/UINode/tum_ardrone_gui.h
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/UINode/moc_tum_ardrone_gui.cxx"
	/usr/bin/moc-qt4 -I/home/bruno/ros_workspace/src/tum_ardrone/include -I/home/bruno/workshop/ardrone_autonomy/msg_gen/cpp/include -I/home/bruno/workshop/ardrone_autonomy/srv_gen/cpp/include -I/opt/ros/fuerte/include -I/opt/ros/fuerte/stacks/image_common/image_transport/include -I/opt/ros/fuerte/stacks/pluginlib/include -I/opt/ros/fuerte/stacks/pluginlib -I/opt/ros/fuerte/stacks/geometry/tf/include -I/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/bullet/include -I/opt/ros/fuerte/stacks/geometry/angles/include -I/opt/ros/fuerte/stacks/image_common/camera_info_manager/include -I/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/include -I/opt/ros/fuerte/include/opencv -I/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/cpp/include -I/home/bruno/ros_workspace/src/tum_ardrone/cfg/cpp -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/TooN/include -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/libcvd/include -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/gvars3/include -I/usr/include/qt4 -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtCore -DKF_REPROJ -DQT_GUI_LIB -DQT_CORE_LIB -o /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_tum_ardrone_gui.cxx /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/tum_ardrone_gui.h

src/UINode/moc_RosThread.cxx: ../src/UINode/RosThread.h
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/UINode/moc_RosThread.cxx"
	/usr/bin/moc-qt4 -I/home/bruno/ros_workspace/src/tum_ardrone/include -I/home/bruno/workshop/ardrone_autonomy/msg_gen/cpp/include -I/home/bruno/workshop/ardrone_autonomy/srv_gen/cpp/include -I/opt/ros/fuerte/include -I/opt/ros/fuerte/stacks/image_common/image_transport/include -I/opt/ros/fuerte/stacks/pluginlib/include -I/opt/ros/fuerte/stacks/pluginlib -I/opt/ros/fuerte/stacks/geometry/tf/include -I/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/bullet/include -I/opt/ros/fuerte/stacks/geometry/angles/include -I/opt/ros/fuerte/stacks/image_common/camera_info_manager/include -I/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/include -I/opt/ros/fuerte/include/opencv -I/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/cpp/include -I/home/bruno/ros_workspace/src/tum_ardrone/cfg/cpp -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/TooN/include -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/libcvd/include -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/gvars3/include -I/usr/include/qt4 -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtCore -DKF_REPROJ -DQT_GUI_LIB -DQT_CORE_LIB -o /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_RosThread.cxx /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/RosThread.h

src/UINode/moc_PingThread.cxx: ../src/UINode/PingThread.h
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/UINode/moc_PingThread.cxx"
	/usr/bin/moc-qt4 -I/home/bruno/ros_workspace/src/tum_ardrone/include -I/home/bruno/workshop/ardrone_autonomy/msg_gen/cpp/include -I/home/bruno/workshop/ardrone_autonomy/srv_gen/cpp/include -I/opt/ros/fuerte/include -I/opt/ros/fuerte/stacks/image_common/image_transport/include -I/opt/ros/fuerte/stacks/pluginlib/include -I/opt/ros/fuerte/stacks/pluginlib -I/opt/ros/fuerte/stacks/geometry/tf/include -I/opt/ros/fuerte/stacks/geometry/tf/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/geometry/tf/srv_gen/cpp/include -I/opt/ros/fuerte/stacks/bullet/include -I/opt/ros/fuerte/stacks/geometry/angles/include -I/opt/ros/fuerte/stacks/image_common/camera_info_manager/include -I/opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/include -I/opt/ros/fuerte/include/opencv -I/opt/ros/fuerte/stacks/vision_opencv/cv_bridge/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv/cpp -I/opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/cpp/include -I/opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/cpp/include -I/home/bruno/ros_workspace/src/tum_ardrone/cfg/cpp -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/TooN/include -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/libcvd/include -I/home/bruno/ros_workspace/src/tum_ardrone/thirdparty/gvars3/include -I/usr/include/qt4 -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtCore -DKF_REPROJ -DQT_GUI_LIB -DQT_CORE_LIB -o /home/bruno/ros_workspace/src/tum_ardrone/build/src/UINode/moc_PingThread.cxx /home/bruno/ros_workspace/src/tum_ardrone/src/UINode/PingThread.h

# Object files for target drone_gui
drone_gui_OBJECTS = \
"CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o" \
"CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o" \
"CMakeFiles/drone_gui.dir/src/UINode/RosThread.o" \
"CMakeFiles/drone_gui.dir/src/UINode/PingThread.o" \
"CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o" \
"CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o" \
"CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o"

# External object files for target drone_gui
drone_gui_EXTERNAL_OBJECTS =

../bin/drone_gui: CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o
../bin/drone_gui: CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o
../bin/drone_gui: CMakeFiles/drone_gui.dir/src/UINode/RosThread.o
../bin/drone_gui: CMakeFiles/drone_gui.dir/src/UINode/PingThread.o
../bin/drone_gui: CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o
../bin/drone_gui: CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o
../bin/drone_gui: CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o
../bin/drone_gui: /usr/lib/i386-linux-gnu/libQtGui.so
../bin/drone_gui: /usr/lib/i386-linux-gnu/libQtCore.so
../bin/drone_gui: CMakeFiles/drone_gui.dir/build.make
../bin/drone_gui: CMakeFiles/drone_gui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/drone_gui"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drone_gui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/drone_gui.dir/build: ../bin/drone_gui
.PHONY : CMakeFiles/drone_gui.dir/build

CMakeFiles/drone_gui.dir/requires: CMakeFiles/drone_gui.dir/src/UINode/main_GUI.o.requires
CMakeFiles/drone_gui.dir/requires: CMakeFiles/drone_gui.dir/src/UINode/tum_ardrone_gui.o.requires
CMakeFiles/drone_gui.dir/requires: CMakeFiles/drone_gui.dir/src/UINode/RosThread.o.requires
CMakeFiles/drone_gui.dir/requires: CMakeFiles/drone_gui.dir/src/UINode/PingThread.o.requires
CMakeFiles/drone_gui.dir/requires: CMakeFiles/drone_gui.dir/src/UINode/moc_tum_ardrone_gui.o.requires
CMakeFiles/drone_gui.dir/requires: CMakeFiles/drone_gui.dir/src/UINode/moc_RosThread.o.requires
CMakeFiles/drone_gui.dir/requires: CMakeFiles/drone_gui.dir/src/UINode/moc_PingThread.o.requires
.PHONY : CMakeFiles/drone_gui.dir/requires

CMakeFiles/drone_gui.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/drone_gui.dir/cmake_clean.cmake
.PHONY : CMakeFiles/drone_gui.dir/clean

CMakeFiles/drone_gui.dir/depend: ui_tum_ardrone_gui.h
CMakeFiles/drone_gui.dir/depend: src/UINode/moc_tum_ardrone_gui.cxx
CMakeFiles/drone_gui.dir/depend: src/UINode/moc_RosThread.cxx
CMakeFiles/drone_gui.dir/depend: src/UINode/moc_PingThread.cxx
	cd /home/bruno/ros_workspace/src/tum_ardrone/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/ros_workspace/src/tum_ardrone /home/bruno/ros_workspace/src/tum_ardrone /home/bruno/ros_workspace/src/tum_ardrone/build /home/bruno/ros_workspace/src/tum_ardrone/build /home/bruno/ros_workspace/src/tum_ardrone/build/CMakeFiles/drone_gui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/drone_gui.dir/depend
