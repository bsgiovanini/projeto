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
include dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/depend.make

# Include the progress variables for this target.
include dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/progress.make

# Include the compile flags for this target's objects.
include dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/flags.make

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/flags.make
dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o: ../dynamicEDT3D/src/dynamicEDT3D.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o"
	cd /home/bruno/octomap/build/dynamicEDT3D/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o -c /home/bruno/octomap/dynamicEDT3D/src/dynamicEDT3D.cpp

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.i"
	cd /home/bruno/octomap/build/dynamicEDT3D/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/dynamicEDT3D/src/dynamicEDT3D.cpp > CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.i

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.s"
	cd /home/bruno/octomap/build/dynamicEDT3D/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/dynamicEDT3D/src/dynamicEDT3D.cpp -o CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.s

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.requires:
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.requires

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.provides: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.requires
	$(MAKE) -f dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/build.make dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.provides.build
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.provides

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.provides.build: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/flags.make
dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o: ../dynamicEDT3D/src/dynamicEDTOctomap.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o"
	cd /home/bruno/octomap/build/dynamicEDT3D/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o -c /home/bruno/octomap/dynamicEDT3D/src/dynamicEDTOctomap.cpp

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.i"
	cd /home/bruno/octomap/build/dynamicEDT3D/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/dynamicEDT3D/src/dynamicEDTOctomap.cpp > CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.i

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.s"
	cd /home/bruno/octomap/build/dynamicEDT3D/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/dynamicEDT3D/src/dynamicEDTOctomap.cpp -o CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.s

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.requires:
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.requires

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.provides: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.requires
	$(MAKE) -f dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/build.make dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.provides.build
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.provides

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.provides.build: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o

# Object files for target dynamicedt3d-static
dynamicedt3d__static_OBJECTS = \
"CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o" \
"CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o"

# External object files for target dynamicedt3d-static
dynamicedt3d__static_EXTERNAL_OBJECTS =

../lib/libdynamicedt3d.a: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o
../lib/libdynamicedt3d.a: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o
../lib/libdynamicedt3d.a: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/build.make
../lib/libdynamicedt3d.a: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../../../lib/libdynamicedt3d.a"
	cd /home/bruno/octomap/build/dynamicEDT3D/src && $(CMAKE_COMMAND) -P CMakeFiles/dynamicedt3d-static.dir/cmake_clean_target.cmake
	cd /home/bruno/octomap/build/dynamicEDT3D/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dynamicedt3d-static.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/build: ../lib/libdynamicedt3d.a
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/build

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/requires: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDT3D.cpp.o.requires
dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/requires: dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/dynamicEDTOctomap.cpp.o.requires
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/requires

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/clean:
	cd /home/bruno/octomap/build/dynamicEDT3D/src && $(CMAKE_COMMAND) -P CMakeFiles/dynamicedt3d-static.dir/cmake_clean.cmake
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/clean

dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/depend:
	cd /home/bruno/octomap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/octomap /home/bruno/octomap/dynamicEDT3D/src /home/bruno/octomap/build /home/bruno/octomap/build/dynamicEDT3D/src /home/bruno/octomap/build/dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dynamicEDT3D/src/CMakeFiles/dynamicedt3d-static.dir/depend

