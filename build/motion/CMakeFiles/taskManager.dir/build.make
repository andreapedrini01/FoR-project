# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/lorenzo/FoR-project/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lorenzo/FoR-project/build

# Include any dependencies generated for this target.
include motion/CMakeFiles/taskManager.dir/depend.make

# Include the progress variables for this target.
include motion/CMakeFiles/taskManager.dir/progress.make

# Include the compile flags for this target's objects.
include motion/CMakeFiles/taskManager.dir/flags.make

motion/CMakeFiles/taskManager.dir/src/taskManager.cpp.o: motion/CMakeFiles/taskManager.dir/flags.make
motion/CMakeFiles/taskManager.dir/src/taskManager.cpp.o: /home/lorenzo/FoR-project/src/motion/src/taskManager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lorenzo/FoR-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object motion/CMakeFiles/taskManager.dir/src/taskManager.cpp.o"
	cd /home/lorenzo/FoR-project/build/motion && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/taskManager.dir/src/taskManager.cpp.o -c /home/lorenzo/FoR-project/src/motion/src/taskManager.cpp

motion/CMakeFiles/taskManager.dir/src/taskManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/taskManager.dir/src/taskManager.cpp.i"
	cd /home/lorenzo/FoR-project/build/motion && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lorenzo/FoR-project/src/motion/src/taskManager.cpp > CMakeFiles/taskManager.dir/src/taskManager.cpp.i

motion/CMakeFiles/taskManager.dir/src/taskManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/taskManager.dir/src/taskManager.cpp.s"
	cd /home/lorenzo/FoR-project/build/motion && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lorenzo/FoR-project/src/motion/src/taskManager.cpp -o CMakeFiles/taskManager.dir/src/taskManager.cpp.s

# Object files for target taskManager
taskManager_OBJECTS = \
"CMakeFiles/taskManager.dir/src/taskManager.cpp.o"

# External object files for target taskManager
taskManager_EXTERNAL_OBJECTS =

/home/lorenzo/FoR-project/devel/lib/motion/taskManager: motion/CMakeFiles/taskManager.dir/src/taskManager.cpp.o
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: motion/CMakeFiles/taskManager.dir/build.make
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/libroscpp.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/librosconsole.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/librostime.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /opt/ros/noetic/lib/libcpp_common.so
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/lorenzo/FoR-project/devel/lib/motion/taskManager: motion/CMakeFiles/taskManager.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lorenzo/FoR-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/lorenzo/FoR-project/devel/lib/motion/taskManager"
	cd /home/lorenzo/FoR-project/build/motion && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/taskManager.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
motion/CMakeFiles/taskManager.dir/build: /home/lorenzo/FoR-project/devel/lib/motion/taskManager

.PHONY : motion/CMakeFiles/taskManager.dir/build

motion/CMakeFiles/taskManager.dir/clean:
	cd /home/lorenzo/FoR-project/build/motion && $(CMAKE_COMMAND) -P CMakeFiles/taskManager.dir/cmake_clean.cmake
.PHONY : motion/CMakeFiles/taskManager.dir/clean

motion/CMakeFiles/taskManager.dir/depend:
	cd /home/lorenzo/FoR-project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lorenzo/FoR-project/src /home/lorenzo/FoR-project/src/motion /home/lorenzo/FoR-project/build /home/lorenzo/FoR-project/build/motion /home/lorenzo/FoR-project/build/motion/CMakeFiles/taskManager.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : motion/CMakeFiles/taskManager.dir/depend

