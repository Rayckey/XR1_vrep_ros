# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/rocky/qtXR1/XR1IMU/XR1IMU

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rocky/catkin_ws/src/vrep_test/IMU-build

# Include any dependencies generated for this target.
include CMakeFiles/XR1IMU.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/XR1IMU.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/XR1IMU.dir/flags.make

CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o: CMakeFiles/XR1IMU.dir/flags.make
CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o: /home/rocky/qtXR1/XR1IMU/XR1IMU/XR1IMUmethods.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rocky/catkin_ws/src/vrep_test/IMU-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o -c /home/rocky/qtXR1/XR1IMU/XR1IMU/XR1IMUmethods.cpp

CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rocky/qtXR1/XR1IMU/XR1IMU/XR1IMUmethods.cpp > CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.i

CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rocky/qtXR1/XR1IMU/XR1IMU/XR1IMUmethods.cpp -o CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.s

CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.requires:

.PHONY : CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.requires

CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.provides: CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.requires
	$(MAKE) -f CMakeFiles/XR1IMU.dir/build.make CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.provides.build
.PHONY : CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.provides

CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.provides.build: CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o


# Object files for target XR1IMU
XR1IMU_OBJECTS = \
"CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o"

# External object files for target XR1IMU
XR1IMU_EXTERNAL_OBJECTS =

libXR1IMU.so: CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o
libXR1IMU.so: CMakeFiles/XR1IMU.dir/build.make
libXR1IMU.so: CMakeFiles/XR1IMU.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rocky/catkin_ws/src/vrep_test/IMU-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libXR1IMU.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/XR1IMU.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/XR1IMU.dir/build: libXR1IMU.so

.PHONY : CMakeFiles/XR1IMU.dir/build

CMakeFiles/XR1IMU.dir/requires: CMakeFiles/XR1IMU.dir/XR1IMUmethods.cpp.o.requires

.PHONY : CMakeFiles/XR1IMU.dir/requires

CMakeFiles/XR1IMU.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/XR1IMU.dir/cmake_clean.cmake
.PHONY : CMakeFiles/XR1IMU.dir/clean

CMakeFiles/XR1IMU.dir/depend:
	cd /home/rocky/catkin_ws/src/vrep_test/IMU-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rocky/qtXR1/XR1IMU/XR1IMU /home/rocky/qtXR1/XR1IMU/XR1IMU /home/rocky/catkin_ws/src/vrep_test/IMU-build /home/rocky/catkin_ws/src/vrep_test/IMU-build /home/rocky/catkin_ws/src/vrep_test/IMU-build/CMakeFiles/XR1IMU.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/XR1IMU.dir/depend

