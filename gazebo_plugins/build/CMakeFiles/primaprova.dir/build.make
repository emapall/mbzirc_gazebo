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
CMAKE_SOURCE_DIR = /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/build

# Include any dependencies generated for this target.
include CMakeFiles/primaprova.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/primaprova.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/primaprova.dir/flags.make

CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o: CMakeFiles/primaprova.dir/flags.make
CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o: ../prova_modelplugin/primaprova.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o -c /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/prova_modelplugin/primaprova.cc

CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/prova_modelplugin/primaprova.cc > CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.i

CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/prova_modelplugin/primaprova.cc -o CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.s

CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.requires:

.PHONY : CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.requires

CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.provides: CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.requires
	$(MAKE) -f CMakeFiles/primaprova.dir/build.make CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.provides.build
.PHONY : CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.provides

CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.provides.build: CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o


# Object files for target primaprova
primaprova_OBJECTS = \
"CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o"

# External object files for target primaprova
primaprova_EXTERNAL_OBJECTS =

libprimaprova.so: CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o
libprimaprova.so: CMakeFiles/primaprova.dir/build.make
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libblas.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libblas.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libignition-transport4.so.4.0.0
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libignition-msgs1.so.1.0.0
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libignition-common1.so.1.1.1
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools1.so.1.2.0
libprimaprova.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libignition-math4.so.4.0.0
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libprimaprova.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libprimaprova.so: CMakeFiles/primaprova.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libprimaprova.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/primaprova.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/primaprova.dir/build: libprimaprova.so

.PHONY : CMakeFiles/primaprova.dir/build

CMakeFiles/primaprova.dir/requires: CMakeFiles/primaprova.dir/prova_modelplugin/primaprova.cc.o.requires

.PHONY : CMakeFiles/primaprova.dir/requires

CMakeFiles/primaprova.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/primaprova.dir/cmake_clean.cmake
.PHONY : CMakeFiles/primaprova.dir/clean

CMakeFiles/primaprova.dir/depend:
	cd /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/build /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/build /home/ema/MBZIRC/drone_ws/src/mbzirc_gazebo/gazebo_plugins/build/CMakeFiles/primaprova.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/primaprova.dir/depend
