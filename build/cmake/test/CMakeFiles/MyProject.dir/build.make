# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\work\g431_foc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\work\g431_foc\build

# Include any dependencies generated for this target.
include cmake/test/CMakeFiles/MyProject.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include cmake/test/CMakeFiles/MyProject.dir/compiler_depend.make

# Include the progress variables for this target.
include cmake/test/CMakeFiles/MyProject.dir/progress.make

# Include the compile flags for this target's objects.
include cmake/test/CMakeFiles/MyProject.dir/flags.make

cmake/test/CMakeFiles/MyProject.dir/main.cpp.obj: cmake/test/CMakeFiles/MyProject.dir/flags.make
cmake/test/CMakeFiles/MyProject.dir/main.cpp.obj: cmake/test/CMakeFiles/MyProject.dir/includes_CXX.rsp
cmake/test/CMakeFiles/MyProject.dir/main.cpp.obj: C:/work/g431_foc/cmake/test/main.cpp
cmake/test/CMakeFiles/MyProject.dir/main.cpp.obj: cmake/test/CMakeFiles/MyProject.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\work\g431_foc\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cmake/test/CMakeFiles/MyProject.dir/main.cpp.obj"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT cmake/test/CMakeFiles/MyProject.dir/main.cpp.obj -MF CMakeFiles\MyProject.dir\main.cpp.obj.d -o CMakeFiles\MyProject.dir\main.cpp.obj -c C:\work\g431_foc\cmake\test\main.cpp

cmake/test/CMakeFiles/MyProject.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MyProject.dir/main.cpp.i"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\work\g431_foc\cmake\test\main.cpp > CMakeFiles\MyProject.dir\main.cpp.i

cmake/test/CMakeFiles/MyProject.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MyProject.dir/main.cpp.s"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\work\g431_foc\cmake\test\main.cpp -o CMakeFiles\MyProject.dir\main.cpp.s

cmake/test/CMakeFiles/MyProject.dir/sin.cpp.obj: cmake/test/CMakeFiles/MyProject.dir/flags.make
cmake/test/CMakeFiles/MyProject.dir/sin.cpp.obj: cmake/test/CMakeFiles/MyProject.dir/includes_CXX.rsp
cmake/test/CMakeFiles/MyProject.dir/sin.cpp.obj: C:/work/g431_foc/cmake/test/sin.cpp
cmake/test/CMakeFiles/MyProject.dir/sin.cpp.obj: cmake/test/CMakeFiles/MyProject.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\work\g431_foc\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cmake/test/CMakeFiles/MyProject.dir/sin.cpp.obj"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT cmake/test/CMakeFiles/MyProject.dir/sin.cpp.obj -MF CMakeFiles\MyProject.dir\sin.cpp.obj.d -o CMakeFiles\MyProject.dir\sin.cpp.obj -c C:\work\g431_foc\cmake\test\sin.cpp

cmake/test/CMakeFiles/MyProject.dir/sin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MyProject.dir/sin.cpp.i"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\work\g431_foc\cmake\test\sin.cpp > CMakeFiles\MyProject.dir\sin.cpp.i

cmake/test/CMakeFiles/MyProject.dir/sin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MyProject.dir/sin.cpp.s"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\work\g431_foc\cmake\test\sin.cpp -o CMakeFiles\MyProject.dir\sin.cpp.s

# Object files for target MyProject
MyProject_OBJECTS = \
"CMakeFiles/MyProject.dir/main.cpp.obj" \
"CMakeFiles/MyProject.dir/sin.cpp.obj"

# External object files for target MyProject
MyProject_EXTERNAL_OBJECTS =

cmake/test/MyProject.exe: cmake/test/CMakeFiles/MyProject.dir/main.cpp.obj
cmake/test/MyProject.exe: cmake/test/CMakeFiles/MyProject.dir/sin.cpp.obj
cmake/test/MyProject.exe: cmake/test/CMakeFiles/MyProject.dir/build.make
cmake/test/MyProject.exe: C:/googletest-main/build/lib/libgtest.a
cmake/test/MyProject.exe: cmake/test/CMakeFiles/MyProject.dir/linkLibs.rsp
cmake/test/MyProject.exe: cmake/test/CMakeFiles/MyProject.dir/objects1.rsp
cmake/test/MyProject.exe: cmake/test/CMakeFiles/MyProject.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=C:\work\g431_foc\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable MyProject.exe"
	cd /d C:\work\g431_foc\build\cmake\test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\MyProject.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cmake/test/CMakeFiles/MyProject.dir/build: cmake/test/MyProject.exe
.PHONY : cmake/test/CMakeFiles/MyProject.dir/build

cmake/test/CMakeFiles/MyProject.dir/clean:
	cd /d C:\work\g431_foc\build\cmake\test && $(CMAKE_COMMAND) -P CMakeFiles\MyProject.dir\cmake_clean.cmake
.PHONY : cmake/test/CMakeFiles/MyProject.dir/clean

cmake/test/CMakeFiles/MyProject.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\work\g431_foc C:\work\g431_foc\cmake\test C:\work\g431_foc\build C:\work\g431_foc\build\cmake\test C:\work\g431_foc\build\cmake\test\CMakeFiles\MyProject.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : cmake/test/CMakeFiles/MyProject.dir/depend
