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

# Utility rule file for clean_test_results_sonar_reading.

# Include the progress variables for this target.
include CMakeFiles/clean_test_results_sonar_reading.dir/progress.make

CMakeFiles/clean_test_results_sonar_reading:
	/usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/remove_test_results.py /home/pi/projeto/src/sonar_reading/test_results/sonar_reading

clean_test_results_sonar_reading: CMakeFiles/clean_test_results_sonar_reading
clean_test_results_sonar_reading: CMakeFiles/clean_test_results_sonar_reading.dir/build.make
.PHONY : clean_test_results_sonar_reading

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results_sonar_reading.dir/build: clean_test_results_sonar_reading
.PHONY : CMakeFiles/clean_test_results_sonar_reading.dir/build

CMakeFiles/clean_test_results_sonar_reading.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_sonar_reading.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results_sonar_reading.dir/clean

CMakeFiles/clean_test_results_sonar_reading.dir/depend:
	cd /home/pi/projeto/src/sonar_reading && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading /home/pi/projeto/src/sonar_reading/CMakeFiles/clean_test_results_sonar_reading.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clean_test_results_sonar_reading.dir/depend

