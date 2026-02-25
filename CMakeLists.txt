cmake_minimum_required(VERSION 3.5)
project(lvi_sam)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -g -pthread")

######################
### Packages
######################
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system timer)
find_package(Eigen3 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

######################
### visual odometry
######################
file(GLOB visual_feature_files
    "src/visual_odometry/visual_feature/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
file(GLOB visual_odometry_files
    "src/visual_odometry/visual_estimator/*.cpp"
    "src/visual_odometry/visual_estimator/factor/*.cpp"
    "src/visual_odometry/visual_estimator/initial/*.cpp"
    "src/visual_odometry/visual_estimator/utility/*.cpp"
)
file(GLOB visual_loop_files
    "src/visual_odometry/visual_loop/*.cpp"
    "src/visual_odometry/visual_loop/utility/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)

# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature ${visual_feature_files})
target_link_libraries(${PROJECT_NAME}_visual_feature
    ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

# Visual Odometry
add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
target_link_libraries(${PROJECT_NAME}_visual_odometry
    ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

# Visual Loop
add_executable(${PROJECT_NAME}_visual_loop ${visual_loop_files})
target_link_libraries(${PROJECT_NAME}_visual_loop
    ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

######################
### lidar odometry
######################

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration
    ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Boost::timer)

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/lidar_odometry/imageProjection.cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection
    ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/lidar_odometry/featureExtraction.cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction
    ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/lidar_odometry/mapOptmization.cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization
    ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam Boost::timer)

######################
### Combined single-process node
######################
# All pipeline components compiled into one executable.
# Internal messages use the in-process broker (ros_compat.h); no middleware.
# Sensor data is injected via the NodeInterface (ros::NodeHandle/Publisher/Subscriber).
#
# Source files are the same as the individual targets above, plus the combined
# entry point.  The LVI_SAM_COMBINED_NODE preprocessor flag suppresses the
# individual main() functions so only the combined main in lvi_sam_node.cpp
# is compiled.
file(GLOB visual_loop_files_no_cam_models
    "src/visual_odometry/visual_loop/*.cpp"
    "src/visual_odometry/visual_loop/utility/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp"
)

add_executable(${PROJECT_NAME}
    src/lvi_sam_node.cpp
    # lidar pipeline
    src/lidar_odometry/imageProjection.cpp
    src/lidar_odometry/featureExtraction.cpp
    src/lidar_odometry/imuPreintegration.cpp
    src/lidar_odometry/mapOptmization.cpp
    # visual pipeline (camera models included once via visual_feature_files)
    ${visual_feature_files}
    ${visual_odometry_files}
    ${visual_loop_files_no_cam_models}
)
target_compile_definitions(${PROJECT_NAME} PRIVATE LVI_SAM_COMBINED_NODE)
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}
    ${OpenMP_CXX_FLAGS} gtsam Boost::timer)
