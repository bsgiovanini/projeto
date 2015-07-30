# - Config file for the OctoMap package
# (example from http://www.vtk.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file)
# It defines the following variables
#  OCTOVIS_INCLUDE_DIRS - include directories for OctoMap viewer
#  OCTOVIS_LIBRARY_DIRS - library directories for OctoMap viewer 
#  OCTOVIS_LIBRARIES    - libraries to link against
 
set(OCTOVIS_INCLUDE_DIRS "/usr/include/QGLViewer" "/home/robotica/bruno_workspace/src/projeto/octomap/octovis/include")
set(OCTOVIS_LIBRARY_DIRS "/usr/lib/i386-linux-gnu" "/usr/local/lib")


# Set library names as absolute paths:
set(OCTOVIS_LIBRARIES
  "qglviewer-qt4"
  "optimized;/usr/lib/i386-linux-gnu/libQtOpenGL.so;debug;/usr/lib/i386-linux-gnu/libQtOpenGL.so;optimized;/usr/lib/i386-linux-gnu/libQtGui.so;debug;/usr/lib/i386-linux-gnu/libQtGui.so;optimized;/usr/lib/i386-linux-gnu/libQtXml.so;debug;/usr/lib/i386-linux-gnu/libQtXml.so;optimized;/usr/lib/i386-linux-gnu/libQtCore.so;debug;/usr/lib/i386-linux-gnu/libQtCore.so"
  "/usr/local/lib/liboctovis.so"
)
