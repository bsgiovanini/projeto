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
CMAKE_SOURCE_DIR = /home/bruno/octomap

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bruno/octomap/build

# Include any dependencies generated for this target.
include octomap/src/CMakeFiles/intersection_example.dir/depend.make

# Include the progress variables for this target.
include octomap/src/CMakeFiles/intersection_example.dir/progress.make

# Include the compile flags for this target's objects.
include octomap/src/CMakeFiles/intersection_example.dir/flags.make

octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o: octomap/src/CMakeFiles/intersection_example.dir/flags.make
octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o: ../octomap/src/intersection_example.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/intersection_example.dir/intersection_example.cpp.o -c /home/bruno/octomap/octomap/src/intersection_example.cpp

octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/intersection_example.dir/intersection_example.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/intersection_example.cpp > CMakeFiles/intersection_example.dir/intersection_example.cpp.i

octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/intersection_example.dir/intersection_example.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/intersection_example.cpp -o CMakeFiles/intersection_example.dir/intersection_example.cpp.s

octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.requires

octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.provides: octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/intersection_example.dir/build.make octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.provides

octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.provides.build: octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o

# Object files for target intersection_example
intersection_example_OBJECTS = \
"CMakeFiles/intersection_example.dir/intersection_example.cpp.o"

# External object files for target intersection_example
intersection_example_EXTERNAL_OBJECTS =

../bin/intersection_example: octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o
../bin/intersection_example: ../lib/liboctomap.so.1.6.8
../bin/intersection_example: ../lib/liboctomath.so.1.6.8
../bin/intersection_example: octomap/src/CMakeFiles/intersection_example.dir/build.make
../bin/intersection_example: octomap/src/CMakeFiles/intersection_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../bin/intersection_example"
	cd /home/bruno/octomap/build/octomap/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/intersection_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
octomap/src/CMakeFiles/intersection_example.dir/build: ../bin/intersection_example
.PHONY : octomap/src/CMakeFiles/intersection_example.dir/build

octomap/src/CMakeFiles/intersection_example.dir/requires: octomap/src/CMakeFiles/intersection_example.dir/intersection_example.cpp.o.requires
.PHONY : octomap/src/CMakeFiles/intersection_example.dir/requires

octomap/src/CMakeFiles/intersection_example.dir/clean:
	cd /home/bruno/octomap/build/octomap/src && $(CMAKE_COMMAND) -P CMakeFiles/intersection_example.dir/cmake_clean.cmake
.PHONY : octomap/src/CMakeFiles/intersection_example.dir/clean

octomap/src/CMakeFiles/intersection_example.dir/depend:
	cd /home/bruno/octomap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/octomap /home/bruno/octomap/octomap/src /home/bruno/octomap/build /home/bruno/octomap/build/octomap/src /home/bruno/octomap/build/octomap/src/CMakeFiles/intersection_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : octomap/src/CMakeFiles/intersection_example.dir/depend

