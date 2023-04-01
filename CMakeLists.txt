cmake_minimum_required(VERSION 3.10.1)
project(cerberus2)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)          
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find Eigen3
find_package(Eigen3)

# find casadi
find_package(casadi REQUIRED)

# add pinocchio 
find_package(urdfdom REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

# find catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        tf
        )

catkin_package(
        CATKIN_DEPENDS
        roscpp
        DEPENDS
        pinocchio
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${CASADI_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIRS}
)

# add libtorch - add /home/EstimationUser/support_ws/libtorch to CMAKE_PREFIX_PATH
# look at Dockerfile to understand why the path is like this
message("${CMAKE_PREFIX_PATH}")
list(APPEND CMAKE_PREFIX_PATH "/home/EstimationUser/support_ws/libtorch")
message("${CMAKE_PREFIX_PATH}")
find_package(Torch REQUIRED)
message("${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
message("${CMAKE_CXX_FLAGS}")

set(FUSION_LIB fusion) # set library name
set(${FUSION_LIB}_SRC
        src/utils/ROSFusion.cpp
        src/mipo/MIPOEstimator.cpp
)
add_library(${FUSION_LIB} SHARED ${${FUSION_LIB}_SRC})
target_link_libraries(${FUSION_LIB}
        PRIVATE
        casadi
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)
target_compile_options(${FUSION_LIB} PRIVATE -std=c++14)
target_include_directories(${FUSION_LIB} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(mipo_main src/program/mipo_main.cpp)
target_link_libraries(mipo_main
        ${catkin_LIBRARIES}
        ${FUSION_LIB}
)


add_executable(cerberus2_main src/program/cerberus2_main.cpp)
target_link_libraries(cerberus2_main
        ${catkin_LIBRARIES}
)


