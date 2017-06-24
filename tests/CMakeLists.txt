# Include the directory itself as a path to include directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${PROJECT_SOURCE_DIR}/src)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()



set(test_SOURCES RunAllTests.cpp 
	test_FusionEKF.cpp ../src/FusionEKF.cpp 
	test_kalman_filter.cpp ../src/kalman_filter.cpp 
	test_tools.cpp ../src/tools.cpp)
add_executable(RunAllTests ${test_SOURCES})
target_link_libraries(RunAllTests gtest_main)

add_test(NAME RunAllTests COMMAND RunAllTests)
