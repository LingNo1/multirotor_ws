# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/pi/multirotor_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/multirotor_ws/build

# Include any dependencies generated for this target.
include init_control/CMakeFiles/offb_node.dir/depend.make

# Include the progress variables for this target.
include init_control/CMakeFiles/offb_node.dir/progress.make

# Include the compile flags for this target's objects.
include init_control/CMakeFiles/offb_node.dir/flags.make

init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o: init_control/CMakeFiles/offb_node.dir/flags.make
init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o: /home/pi/multirotor_ws/src/init_control/src/offboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/multirotor_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o"
	cd /home/pi/multirotor_ws/build/init_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/offb_node.dir/src/offboard.cpp.o -c /home/pi/multirotor_ws/src/init_control/src/offboard.cpp

init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/offb_node.dir/src/offboard.cpp.i"
	cd /home/pi/multirotor_ws/build/init_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/multirotor_ws/src/init_control/src/offboard.cpp > CMakeFiles/offb_node.dir/src/offboard.cpp.i

init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/offb_node.dir/src/offboard.cpp.s"
	cd /home/pi/multirotor_ws/build/init_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/multirotor_ws/src/init_control/src/offboard.cpp -o CMakeFiles/offb_node.dir/src/offboard.cpp.s

init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.requires:

.PHONY : init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.requires

init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.provides: init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.requires
	$(MAKE) -f init_control/CMakeFiles/offb_node.dir/build.make init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.provides.build
.PHONY : init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.provides

init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.provides.build: init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o


# Object files for target offb_node
offb_node_OBJECTS = \
"CMakeFiles/offb_node.dir/src/offboard.cpp.o"

# External object files for target offb_node
offb_node_EXTERNAL_OBJECTS =

/home/pi/multirotor_ws/devel/lib/init_control/offb_node: init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: init_control/CMakeFiles/offb_node.dir/build.make
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/libroscpp.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/librosconsole.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/librostime.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /opt/ros/melodic/lib/libcpp_common.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/pi/multirotor_ws/devel/lib/init_control/offb_node: init_control/CMakeFiles/offb_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/multirotor_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/multirotor_ws/devel/lib/init_control/offb_node"
	cd /home/pi/multirotor_ws/build/init_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/offb_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
init_control/CMakeFiles/offb_node.dir/build: /home/pi/multirotor_ws/devel/lib/init_control/offb_node

.PHONY : init_control/CMakeFiles/offb_node.dir/build

init_control/CMakeFiles/offb_node.dir/requires: init_control/CMakeFiles/offb_node.dir/src/offboard.cpp.o.requires

.PHONY : init_control/CMakeFiles/offb_node.dir/requires

init_control/CMakeFiles/offb_node.dir/clean:
	cd /home/pi/multirotor_ws/build/init_control && $(CMAKE_COMMAND) -P CMakeFiles/offb_node.dir/cmake_clean.cmake
.PHONY : init_control/CMakeFiles/offb_node.dir/clean

init_control/CMakeFiles/offb_node.dir/depend:
	cd /home/pi/multirotor_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/multirotor_ws/src /home/pi/multirotor_ws/src/init_control /home/pi/multirotor_ws/build /home/pi/multirotor_ws/build/init_control /home/pi/multirotor_ws/build/init_control/CMakeFiles/offb_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : init_control/CMakeFiles/offb_node.dir/depend

