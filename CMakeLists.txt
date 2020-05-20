project(soap_bar_rrt)
cmake_minimum_required(VERSION 3.0.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroInstallLib)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")


find_package(catkin REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()





INCLUDE_DIRECTORIES(include ${EIGEN3_INCLUDE_DIR}
                            )

catkin_package(
   INCLUDE_DIRS include)

SET(SOURCE_FILES src/Configuration.cpp  
                 src/Contact.cpp
                 #src/Planner.cpp
                 src/Stance.cpp
                 src/Tree.cpp
                 src/Vertex.cpp
                 )

ADD_LIBRARY(soap_bar_rrt SHARED
                       ${SOURCE_FILES}
                       )

#TARGET_LINK_LIBRARIES(soap_bar_rrt PUBLIC )

INSTALL(TARGETS soap_bar_rrt 
        DESTINATION lib)

