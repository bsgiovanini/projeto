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
include octomap/src/testing/CMakeFiles/test_color_tree.dir/depend.make

# Include the progress variables for this target.
include octomap/src/testing/CMakeFiles/test_color_tree.dir/progress.make

# Include the compile flags for this target's objects.
include octomap/src/testing/CMakeFiles/test_color_tree.dir/flags.make

octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o: octomap/src/testing/CMakeFiles/test_color_tree.dir/flags.make
octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o: ../octomap/src/testing/test_color_tree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o"
	cd /home/bruno/octomap/build/octomap/src/testing && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o -c /home/bruno/octomap/octomap/src/testing/test_color_tree.cpp

octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_color_tree.dir/test_color_tree.cpp.i"
	cd /home/bruno/octomap/build/octomap/src/testing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/testing/test_color_tree.cpp > CMakeFiles/test_color_tree.dir/test_color_tree.cpp.i

octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_color_tree.dir/test_color_tree.cpp.s"
	cd /home/bruno/octomap/build/octomap/src/testing && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/testing/test_color_tree.cpp -o CMakeFiles/test_color_tree.dir/test_color_tree.cpp.s

octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.requires:
.PHONY : octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.requires

octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.provides: octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.requires
	$(MAKE) -f octomap/src/testing/CMakeFiles/test_color_tree.dir/build.make octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.provides.build
.PHONY : octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.provides

octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.provides.build: octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o

# Object files for target test_color_tree
test_color_tree_OBJECTS = \
"CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o"

# External object files for target test_color_tree
test_color_tree_EXTERNAL_OBJECTS =

../bin/test_color_tree: octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o
../bin/test_color_tree: ../lib/liboctomap.so.1.6.8
../bin/test_color_tree: ../lib/liboctomath.so.1.6.8
../bin/test_color_tree: octomap/src/testing/CMakeFiles/test_color_tree.dir/build.make
../bin/test_color_tree: octomap/src/testing/CMakeFiles/test_color_tree.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../../../bin/test_color_tree"
	cd /home/bruno/octomap/build/octomap/src/testing && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_color_tree.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
octomap/src/testing/CMakeFiles/test_color_tree.dir/build: ../bin/test_color_tree
.PHONY : octomap/src/testing/CMakeFiles/test_color_tree.dir/build

octomap/src/testing/CMakeFiles/test_color_tree.dir/requires: octomap/src/testing/CMakeFiles/test_color_tree.dir/test_color_tree.cpp.o.requires
.PHONY : octomap/src/testing/CMakeFiles/test_color_tree.dir/requires

octomap/src/testing/CMakeFiles/test_color_tree.dir/clean:
	cd /home/bruno/octomap/build/octomap/src/testing && $(CMAKE_COMMAND) -P CMakeFiles/test_color_tree.dir/cmake_clean.cmake
.PHONY : octomap/src/testing/CMakeFiles/test_color_tree.dir/clean

octomap/src/testing/CMakeFiles/test_color_tree.dir/depend:
	cd /home/bruno/octomap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/octomap /home/bruno/octomap/octomap/src/testing /home/bruno/octomap/build /home/bruno/octomap/build/octomap/src/testing /home/bruno/octomap/build/octomap/src/testing/CMakeFiles/test_color_tree.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : octomap/src/testing/CMakeFiles/test_color_tree.dir/depend

