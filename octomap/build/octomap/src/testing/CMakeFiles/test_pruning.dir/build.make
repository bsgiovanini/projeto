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
include octomap/src/testing/CMakeFiles/test_pruning.dir/depend.make

# Include the progress variables for this target.
include octomap/src/testing/CMakeFiles/test_pruning.dir/progress.make

# Include the compile flags for this target's objects.
include octomap/src/testing/CMakeFiles/test_pruning.dir/flags.make

octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o: octomap/src/testing/CMakeFiles/test_pruning.dir/flags.make
octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o: ../octomap/src/testing/test_pruning.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o"
	cd /home/bruno/octomap/build/octomap/src/testing && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_pruning.dir/test_pruning.cpp.o -c /home/bruno/octomap/octomap/src/testing/test_pruning.cpp

octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_pruning.dir/test_pruning.cpp.i"
	cd /home/bruno/octomap/build/octomap/src/testing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/testing/test_pruning.cpp > CMakeFiles/test_pruning.dir/test_pruning.cpp.i

octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_pruning.dir/test_pruning.cpp.s"
	cd /home/bruno/octomap/build/octomap/src/testing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/testing/test_pruning.cpp -o CMakeFiles/test_pruning.dir/test_pruning.cpp.s

octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.requires:
.PHONY : octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.requires

octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.provides: octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.requires
	$(MAKE) -f octomap/src/testing/CMakeFiles/test_pruning.dir/build.make octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.provides.build
.PHONY : octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.provides

octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.provides.build: octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o

# Object files for target test_pruning
test_pruning_OBJECTS = \
"CMakeFiles/test_pruning.dir/test_pruning.cpp.o"

# External object files for target test_pruning
test_pruning_EXTERNAL_OBJECTS =

../bin/test_pruning: octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o
../bin/test_pruning: ../lib/liboctomap.so.1.6.8
../bin/test_pruning: ../lib/liboctomath.so.1.6.8
../bin/test_pruning: octomap/src/testing/CMakeFiles/test_pruning.dir/build.make
../bin/test_pruning: octomap/src/testing/CMakeFiles/test_pruning.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../../bin/test_pruning"
	cd /home/bruno/octomap/build/octomap/src/testing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_pruning.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
octomap/src/testing/CMakeFiles/test_pruning.dir/build: ../bin/test_pruning
.PHONY : octomap/src/testing/CMakeFiles/test_pruning.dir/build

octomap/src/testing/CMakeFiles/test_pruning.dir/requires: octomap/src/testing/CMakeFiles/test_pruning.dir/test_pruning.cpp.o.requires
.PHONY : octomap/src/testing/CMakeFiles/test_pruning.dir/requires

octomap/src/testing/CMakeFiles/test_pruning.dir/clean:
	cd /home/bruno/octomap/build/octomap/src/testing && $(CMAKE_COMMAND) -P CMakeFiles/test_pruning.dir/cmake_clean.cmake
.PHONY : octomap/src/testing/CMakeFiles/test_pruning.dir/clean

octomap/src/testing/CMakeFiles/test_pruning.dir/depend:
	cd /home/bruno/octomap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/octomap /home/bruno/octomap/octomap/src/testing /home/bruno/octomap/build /home/bruno/octomap/build/octomap/src/testing /home/bruno/octomap/build/octomap/src/testing/CMakeFiles/test_pruning.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : octomap/src/testing/CMakeFiles/test_pruning.dir/depend

