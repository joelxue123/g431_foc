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
include cmake/test/CMakeFiles/MyProject1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include cmake/test/CMakeFiles/MyProject1.dir/compiler_depend.make

# Include the progress variables for this target.
include cmake/test/CMakeFiles/MyProject1.dir/progress.make

# Include the compile flags for this target's objects.
include cmake/test/CMakeFiles/MyProject1.dir/flags.make

cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.obj: cmake/test/CMakeFiles/MyProject1.dir/flags.make
cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.obj: cmake/test/CMakeFiles/MyProject1.dir/includes_CXX.rsp
cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.obj: C:/work/g431_foc/cmake/test/main1.cpp
cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.obj: cmake/test/CMakeFiles/MyProject1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\work\g431_foc\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.obj"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.obj -MF CMakeFiles\MyProject1.dir\main1.cpp.obj.d -o CMakeFiles\MyProject1.dir\main1.cpp.obj -c C:\work\g431_foc\cmake\test\main1.cpp

cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MyProject1.dir/main1.cpp.i"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\work\g431_foc\cmake\test\main1.cpp > CMakeFiles\MyProject1.dir\main1.cpp.i

cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MyProject1.dir/main1.cpp.s"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\work\g431_foc\cmake\test\main1.cpp -o CMakeFiles\MyProject1.dir\main1.cpp.s

cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.obj: cmake/test/CMakeFiles/MyProject1.dir/flags.make
cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.obj: cmake/test/CMakeFiles/MyProject1.dir/includes_CXX.rsp
cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.obj: C:/work/g431_foc/cmake/test/sin.cpp
cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.obj: cmake/test/CMakeFiles/MyProject1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=C:\work\g431_foc\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.obj"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.obj -MF CMakeFiles\MyProject1.dir\sin.cpp.obj.d -o CMakeFiles\MyProject1.dir\sin.cpp.obj -c C:\work\g431_foc\cmake\test\sin.cpp

cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/MyProject1.dir/sin.cpp.i"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\work\g431_foc\cmake\test\sin.cpp > CMakeFiles\MyProject1.dir\sin.cpp.i

cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/MyProject1.dir/sin.cpp.s"
	cd /d C:\work\g431_foc\build\cmake\test && C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\work\g431_foc\cmake\test\sin.cpp -o CMakeFiles\MyProject1.dir\sin.cpp.s

# Object files for target MyProject1
MyProject1_OBJECTS = \
"CMakeFiles/MyProject1.dir/main1.cpp.obj" \
"CMakeFiles/MyProject1.dir/sin.cpp.obj"

# External object files for target MyProject1
MyProject1_EXTERNAL_OBJECTS =

cmake/test/MyProject1.exe: cmake/test/CMakeFiles/MyProject1.dir/main1.cpp.obj
cmake/test/MyProject1.exe: cmake/test/CMakeFiles/MyProject1.dir/sin.cpp.obj
cmake/test/MyProject1.exe: cmake/test/CMakeFiles/MyProject1.dir/build.make
cmake/test/MyProject1.exe: C:/Users/82057/Downloads/googletest-main/build/lib/libgmock.a
cmake/test/MyProject1.exe: C:/googletest-main/build/lib/libgtest_main.a
cmake/test/MyProject1.exe: C:/googletest-main/build/lib/libgtest.a
cmake/test/MyProject1.exe: cmake/test/CMakeFiles/MyProject1.dir/linkLibs.rsp
cmake/test/MyProject1.exe: cmake/test/CMakeFiles/MyProject1.dir/objects1.rsp
cmake/test/MyProject1.exe: cmake/test/CMakeFiles/MyProject1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=C:\work\g431_foc\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable MyProject1.exe"
	cd /d C:\work\g431_foc\build\cmake\test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\MyProject1.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cmake/test/CMakeFiles/MyProject1.dir/build: cmake/test/MyProject1.exe
.PHONY : cmake/test/CMakeFiles/MyProject1.dir/build

cmake/test/CMakeFiles/MyProject1.dir/clean:
	cd /d C:\work\g431_foc\build\cmake\test && $(CMAKE_COMMAND) -P CMakeFiles\MyProject1.dir\cmake_clean.cmake
.PHONY : cmake/test/CMakeFiles/MyProject1.dir/clean

cmake/test/CMakeFiles/MyProject1.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\work\g431_foc C:\work\g431_foc\cmake\test C:\work\g431_foc\build C:\work\g431_foc\build\cmake\test C:\work\g431_foc\build\cmake\test\CMakeFiles\MyProject1.dir\DependInfo.cmake "--color=$(COLOR)"
.PHONY : cmake/test/CMakeFiles/MyProject1.dir/depend
