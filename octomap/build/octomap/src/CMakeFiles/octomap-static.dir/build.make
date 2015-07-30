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
include octomap/src/CMakeFiles/octomap-static.dir/depend.make

# Include the progress variables for this target.
include octomap/src/CMakeFiles/octomap-static.dir/progress.make

# Include the compile flags for this target's objects.
include octomap/src/CMakeFiles/octomap-static.dir/flags.make

octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o: ../octomap/src/AbstractOcTree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o -c /home/bruno/octomap/octomap/src/AbstractOcTree.cpp

octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/AbstractOcTree.cpp > CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/AbstractOcTree.cpp -o CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o: ../octomap/src/AbstractOccupancyOcTree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o -c /home/bruno/octomap/octomap/src/AbstractOccupancyOcTree.cpp

octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/AbstractOccupancyOcTree.cpp > CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/AbstractOccupancyOcTree.cpp -o CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o: ../octomap/src/Pointcloud.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/Pointcloud.cpp.o -c /home/bruno/octomap/octomap/src/Pointcloud.cpp

octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/Pointcloud.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/Pointcloud.cpp > CMakeFiles/octomap-static.dir/Pointcloud.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/Pointcloud.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/Pointcloud.cpp -o CMakeFiles/octomap-static.dir/Pointcloud.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o: ../octomap/src/ScanGraph.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/ScanGraph.cpp.o -c /home/bruno/octomap/octomap/src/ScanGraph.cpp

octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/ScanGraph.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/ScanGraph.cpp > CMakeFiles/octomap-static.dir/ScanGraph.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/ScanGraph.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/ScanGraph.cpp -o CMakeFiles/octomap-static.dir/ScanGraph.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o: ../octomap/src/CountingOcTree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o -c /home/bruno/octomap/octomap/src/CountingOcTree.cpp

octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/CountingOcTree.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/CountingOcTree.cpp > CMakeFiles/octomap-static.dir/CountingOcTree.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/CountingOcTree.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/CountingOcTree.cpp -o CMakeFiles/octomap-static.dir/CountingOcTree.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o: ../octomap/src/OcTree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/OcTree.cpp.o -c /home/bruno/octomap/octomap/src/OcTree.cpp

octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/OcTree.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/OcTree.cpp > CMakeFiles/octomap-static.dir/OcTree.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/OcTree.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/OcTree.cpp -o CMakeFiles/octomap-static.dir/OcTree.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o: ../octomap/src/OcTreeNode.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o -c /home/bruno/octomap/octomap/src/OcTreeNode.cpp

octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/OcTreeNode.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/OcTreeNode.cpp > CMakeFiles/octomap-static.dir/OcTreeNode.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/OcTreeNode.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/OcTreeNode.cpp -o CMakeFiles/octomap-static.dir/OcTreeNode.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o: ../octomap/src/OcTreeStamped.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o -c /home/bruno/octomap/octomap/src/OcTreeStamped.cpp

octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/OcTreeStamped.cpp > CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/OcTreeStamped.cpp -o CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o: ../octomap/src/OcTreeLUT.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o -c /home/bruno/octomap/octomap/src/OcTreeLUT.cpp

octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/OcTreeLUT.cpp > CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/OcTreeLUT.cpp -o CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o

octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o: octomap/src/CMakeFiles/octomap-static.dir/flags.make
octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o: ../octomap/src/ColorOcTree.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/bruno/octomap/build/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o -c /home/bruno/octomap/octomap/src/ColorOcTree.cpp

octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octomap-static.dir/ColorOcTree.cpp.i"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/bruno/octomap/octomap/src/ColorOcTree.cpp > CMakeFiles/octomap-static.dir/ColorOcTree.cpp.i

octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octomap-static.dir/ColorOcTree.cpp.s"
	cd /home/bruno/octomap/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/bruno/octomap/octomap/src/ColorOcTree.cpp -o CMakeFiles/octomap-static.dir/ColorOcTree.cpp.s

octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.requires:
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.requires

octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.provides: octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.requires
	$(MAKE) -f octomap/src/CMakeFiles/octomap-static.dir/build.make octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.provides.build
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.provides

octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.provides.build: octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o

# Object files for target octomap-static
octomap__static_OBJECTS = \
"CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o" \
"CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o" \
"CMakeFiles/octomap-static.dir/Pointcloud.cpp.o" \
"CMakeFiles/octomap-static.dir/ScanGraph.cpp.o" \
"CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o" \
"CMakeFiles/octomap-static.dir/OcTree.cpp.o" \
"CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o" \
"CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o" \
"CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o" \
"CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o"

# External object files for target octomap-static
octomap__static_EXTERNAL_OBJECTS =

../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/build.make
../lib/liboctomap.a: octomap/src/CMakeFiles/octomap-static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../../../lib/liboctomap.a"
	cd /home/bruno/octomap/build/octomap/src && $(CMAKE_COMMAND) -P CMakeFiles/octomap-static.dir/cmake_clean_target.cmake
	cd /home/bruno/octomap/build/octomap/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/octomap-static.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
octomap/src/CMakeFiles/octomap-static.dir/build: ../lib/liboctomap.a
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/build

octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/AbstractOcTree.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/AbstractOccupancyOcTree.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/Pointcloud.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/ScanGraph.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/CountingOcTree.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/OcTree.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/OcTreeNode.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/OcTreeStamped.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/OcTreeLUT.cpp.o.requires
octomap/src/CMakeFiles/octomap-static.dir/requires: octomap/src/CMakeFiles/octomap-static.dir/ColorOcTree.cpp.o.requires
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/requires

octomap/src/CMakeFiles/octomap-static.dir/clean:
	cd /home/bruno/octomap/build/octomap/src && $(CMAKE_COMMAND) -P CMakeFiles/octomap-static.dir/cmake_clean.cmake
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/clean

octomap/src/CMakeFiles/octomap-static.dir/depend:
	cd /home/bruno/octomap/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bruno/octomap /home/bruno/octomap/octomap/src /home/bruno/octomap/build /home/bruno/octomap/build/octomap/src /home/bruno/octomap/build/octomap/src/CMakeFiles/octomap-static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : octomap/src/CMakeFiles/octomap-static.dir/depend

