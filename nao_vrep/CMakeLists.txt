cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)
project (nao_vrep)

####################################
# Settings
####################################
set (CMAKE_SKIP_ASSEMBLY_SOURCE_RULES ON)
set (CMAKE_SKIP_PREPROCESSED_SOURCE_RULES ON)
set (CMAKE_VERBOSE_MAKEFILE ON) # print full commands during compilation
set (CMAKE_RULE_MESSAGES OFF CACHE BOOL "")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib/")

set(VREP_DIR "." CACHE PATH "v-rep directory")

set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")

find_package(Boost REQUIRED system thread)

####################################
# Other parameters
####################################

# set path to directories containing headers
find_path(EIGEN_INCLUDE_DIRS "Eigen/Core" PATHS "/usr/include/" PATH_SUFFIXES "eigen3")
if (EIGEN_INCLUDE_DIRS)
    include_directories ("${EIGEN_INCLUDE_DIRS}")
endif (EIGEN_INCLUDE_DIRS)

INCLUDE_DIRECTORIES(
    ${Boost_INCLUDE_DIR}
)

include_directories (
    include
    /usr/local/include
    ${VREP_DIR}/programming/common
    ${VREP_DIR}/programming/include
    ${VREP_DIR}/programming/remoteApi
) 

link_directories(
    ${VREP_DIR}/programming/v_repExtRemoteApi/lib
)

EXECUTE_PROCESS(COMMAND make WORKING_DIRECTORY ${VREP_DIR}/programming/v_repExtRemoteApi )

file (GLOB NAO_VREP_SRC "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_library(nao_vrep 
    ${NAO_VREP_SRC}
    src/vrep/VRepRobotInterface.cpp
    ${VREP_DIR}/programming/remoteApi/extApi.c
    ${VREP_DIR}/programming/remoteApi/extApiPlatform.c
)

# link to libraries
target_link_libraries(nao_vrep
    v_repExtRemoteApi 
    pthread
    ${Boost_LIBRARIES}
)

