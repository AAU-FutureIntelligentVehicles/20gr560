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
CMAKE_SOURCE_DIR = /home/frederik/Documents/20gr560/workspaces/melodic/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/frederik/Documents/20gr560/workspaces/melodic/build

# Include any dependencies generated for this target.
include tpod_teleop/CMakeFiles/watchdog.dir/depend.make

# Include the progress variables for this target.
include tpod_teleop/CMakeFiles/watchdog.dir/progress.make

# Include the compile flags for this target's objects.
include tpod_teleop/CMakeFiles/watchdog.dir/flags.make

tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o: tpod_teleop/CMakeFiles/watchdog.dir/flags.make
tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o: /home/frederik/Documents/20gr560/workspaces/melodic/src/tpod_teleop/src/watchdog.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frederik/Documents/20gr560/workspaces/melodic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o"
	cd /home/frederik/Documents/20gr560/workspaces/melodic/build/tpod_teleop && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/watchdog.dir/src/watchdog.cpp.o -c /home/frederik/Documents/20gr560/workspaces/melodic/src/tpod_teleop/src/watchdog.cpp

tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/watchdog.dir/src/watchdog.cpp.i"
	cd /home/frederik/Documents/20gr560/workspaces/melodic/build/tpod_teleop && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frederik/Documents/20gr560/workspaces/melodic/src/tpod_teleop/src/watchdog.cpp > CMakeFiles/watchdog.dir/src/watchdog.cpp.i

tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/watchdog.dir/src/watchdog.cpp.s"
	cd /home/frederik/Documents/20gr560/workspaces/melodic/build/tpod_teleop && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frederik/Documents/20gr560/workspaces/melodic/src/tpod_teleop/src/watchdog.cpp -o CMakeFiles/watchdog.dir/src/watchdog.cpp.s

tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.requires:

.PHONY : tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.requires

tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.provides: tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.requires
	$(MAKE) -f tpod_teleop/CMakeFiles/watchdog.dir/build.make tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.provides.build
.PHONY : tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.provides

tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.provides.build: tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o


# Object files for target watchdog
watchdog_OBJECTS = \
"CMakeFiles/watchdog.dir/src/watchdog.cpp.o"

# External object files for target watchdog
watchdog_EXTERNAL_OBJECTS =

/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: tpod_teleop/CMakeFiles/watchdog.dir/build.make
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/libroscpp.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/librosconsole.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/librostime.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /opt/ros/melodic/lib/libcpp_common.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog: tpod_teleop/CMakeFiles/watchdog.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/frederik/Documents/20gr560/workspaces/melodic/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog"
	cd /home/frederik/Documents/20gr560/workspaces/melodic/build/tpod_teleop && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/watchdog.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tpod_teleop/CMakeFiles/watchdog.dir/build: /home/frederik/Documents/20gr560/workspaces/melodic/devel/lib/tpod_teleop/watchdog

.PHONY : tpod_teleop/CMakeFiles/watchdog.dir/build

tpod_teleop/CMakeFiles/watchdog.dir/requires: tpod_teleop/CMakeFiles/watchdog.dir/src/watchdog.cpp.o.requires

.PHONY : tpod_teleop/CMakeFiles/watchdog.dir/requires

tpod_teleop/CMakeFiles/watchdog.dir/clean:
	cd /home/frederik/Documents/20gr560/workspaces/melodic/build/tpod_teleop && $(CMAKE_COMMAND) -P CMakeFiles/watchdog.dir/cmake_clean.cmake
.PHONY : tpod_teleop/CMakeFiles/watchdog.dir/clean

tpod_teleop/CMakeFiles/watchdog.dir/depend:
	cd /home/frederik/Documents/20gr560/workspaces/melodic/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/frederik/Documents/20gr560/workspaces/melodic/src /home/frederik/Documents/20gr560/workspaces/melodic/src/tpod_teleop /home/frederik/Documents/20gr560/workspaces/melodic/build /home/frederik/Documents/20gr560/workspaces/melodic/build/tpod_teleop /home/frederik/Documents/20gr560/workspaces/melodic/build/tpod_teleop/CMakeFiles/watchdog.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tpod_teleop/CMakeFiles/watchdog.dir/depend

