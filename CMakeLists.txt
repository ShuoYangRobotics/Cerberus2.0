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

find_package(OpenCV REQUIRED)
# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)
#message(WARNING "CERES_LIBRARIES: ${CERES_LIBRARIES}")

# find catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        cv_bridge
        camera_models
        image_transport
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
    ${CERES_INCLUDE_DIRS}
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
        src/fusion/ROSFusion.cpp
        src/mipo/MIPOEstimator.cpp
        src/utils/parameters.cpp
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


set(VILO_LIB vilo_fusion) # set library name
set(${VILO_LIB}_SRC
        src/fusion/VILOFusion.cpp
        src/mipo/MIPOEstimator.cpp
        src/sipo/SIPOEstimator.cpp
        src/vilo/VILOEstimator.cpp
        src/featureTracker/feature_tracker.cpp
        src/featureTracker/feature_manager.cpp
        src/utils/parameters.cpp
        src/utils/visualization.cpp
        src/utils/CameraPoseVisualization.cpp
        src/utils/vins_utility.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/projectionTwoFrameOneCamFactor.cpp
        src/factor/projectionTwoFrameTwoCamFactor.cpp
        src/factor/projectionOneFrameTwoCamFactor.cpp
        src/factor/marginalization_factor.cpp

        src/vilo/initial_aligment.cpp
)
add_library(${VILO_LIB} SHARED ${${VILO_LIB}_SRC})
target_link_libraries(${VILO_LIB}
        casadi
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES} 
        ${OpenCV_LIBS}
)
target_compile_options(${VILO_LIB} PRIVATE -std=c++14)
target_include_directories(${VILO_LIB} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(cerberus2_main src/program/cerberus2_main.cpp)
target_link_libraries(cerberus2_main
        ${catkin_LIBRARIES}
        ${VILO_LIB}
)

# ##
# ## tests
# ##
add_executable(test_LOTightFactor test/test_LOTightFactor.cpp
        src/utils/parameters.cpp)
target_link_libraries(test_LOTightFactor
        PRIVATE
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        casadi
)

add_executable(test_LOTightIntegration 
        test/test_LOTightIntegration.cpp
        src/utils/parameters.cpp
)
target_link_libraries(test_LOTightIntegration
        PRIVATE
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        casadi
)
add_custom_target(run_test_LOTightIntegration
        ALL
        COMMAND $<TARGET_FILE:test_LOTightIntegration>
        DEPENDS test_LOTightIntegration
        WORKING_DIRECTORY "/home/EstimationUser/estimation_ws/devel/lib/cerberus2"
        COMMENT "Executing target: my_executable"
        VERBATIM)
