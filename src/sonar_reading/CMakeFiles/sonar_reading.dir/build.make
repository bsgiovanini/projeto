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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/projeto/src/sonar_reading

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/projeto/src/sonar_reading

# Include any dependencies generated for this target.
include CMakeFiles/sonar_reading.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sonar_reading.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sonar_reading.dir/flags.make

CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o: CMakeFiles/sonar_reading.dir/flags.make
CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o: src/sonar_reading.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/projeto/src/sonar_reading/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o -c /home/pi/projeto/src/sonar_reading/src/sonar_reading.cpp

CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pi/projeto/src/sonar_reading/src/sonar_reading.cpp > CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.i

CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pi/projeto/src/sonar_reading/src/sonar_reading.cpp -o CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.s

CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.requires:
.PHONY : CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.requires

CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.provides: CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.requires
	$(MAKE) -f CMakeFiles/sonar_reading.dir/build.make CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.provides.build
.PHONY : CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.provides

CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.provides.build: CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o

# Object files for target sonar_reading
sonar_reading_OBJECTS = \
"CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o"

# External object files for target sonar_reading
sonar_reading_EXTERNAL_OBJECTS =

devel/lib/sonar_reading/sonar_reading: CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o
devel/lib/sonar_reading/sonar_reading: CMakeFiles/sonar_reading.dir/build.make
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/libroscpp.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/libboost_signals-mt.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/libboost_filesystem-mt.so
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/librosconsole.so
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/librosconsole_log4cxx.so
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/librosconsole_backend_interface.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/liblog4cxx.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/libboost_regex-mt.so
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/libxmlrpcpp.so
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/libroscpp_serialization.so
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/librostime.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/libboost_date_time-mt.so
devel/lib/sonar_reading/sonar_reading: /opt/ros/indigo/lib/libcpp_common.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/libboost_system-mt.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/libboost_thread-mt.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/arm-linux-gnueabihf/libpthread.so
devel/lib/sonar_reading/sonar_reading: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
devel/lib/sonar_reading/sonar_reading: CMakeFiles/sonar_reading.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable devel/lib/sonar_reading/sonar_reading"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sonar_reading.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sonar_reading.dir/build: devel/lib/sonar_reading/sonar_reading
.PHONY : CMakeFiles/sonar_reading.dir/build

CMakeFiles/sonar_reading.dir/requires: CMakeFiles/sonar_reading.dir/src/sonar_reading.cpp.o.requires
.PHONY : CMakeFiles/sonar_reading.dir/requires

CMakeFiles/sonar_reading.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sonar_reading.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sonar_reading.dir/clean

CMakeFiles/sonar_reading.dir/depend:
	cd /home/pi/projeto/src/sonar_reading && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading/CMakeFiles/sonar_reading.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sonar_reading.dir/depend

