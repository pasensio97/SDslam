cmake_minimum_required(VERSION 2.8)
project(SD_SLAM)
option(USE_ANDROID "Android Cross Compilation" OFF)
set(USE_PANGOLIN ON)
set(DEBUG OFF)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

if(USE_ANDROID)
  MESSAGE(STATUS "Compiling for Android")

  ADD_DEFINITIONS(-DANDROID)

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -std=c++11")

  set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
  set(OPENCV_SDK /opt/OpenCV-android-sdk)

  # Configure OpenCV
  include_directories(${OPENCV_SDK}/sdk/native/jni/include)
  add_library(lib_opencv SHARED IMPORTED)
  set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION ${OPENCV_SDK}/sdk/native/libs/${ANDROID_ABI}/libopencv_java3.so)
else()
  MESSAGE(STATUS "Compiling on Unix")

  if(USE_PANGOLIN)
    ADD_DEFINITIONS(-DPANGOLIN)
  endif()

  if(DEBUG)
    ADD_DEFINITIONS(-g)
  endif()

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -std=c++17")

  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
     find_package(OpenCV 2.4.3 REQUIRED)
  endif()
  find_package(Eigen3 3.1.0 REQUIRED)
  if(USE_PANGOLIN)
    find_package(Pangolin REQUIRED)
  endif()
endif()

# ROS DEPENDENCIES
set(CATKIN_COMPONENTS
        roscpp
        cv_bridge
        image_transport
        std_msgs
        tf)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

# MRPT COMPILES ON C++17 AND MAKE EVERYTHING BE IN C++17 I THINK SO I HAD TO CHANGE CODE IN
# LOOPCLOSING TO COMPILE.
find_package(MRPT REQUIRED poses gui vision)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

if(USE_PANGOLIN)
  include_directories(${Pangolin_INCLUDE_DIRS})
endif()

if(USE_ANDROID)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${ANDROID_ABI})
else()
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
endif()

# Set SD-SLAM sourcefiles
LIST(APPEND SOURCEFILES
  # SD-SLAM system
  src/System.cc
  src/Tracking.cc
  src/ImageAlign.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Config.cc
  src/PatternDetector.cc

  # Sensors
  src/sensors/EKF.cc
  src/sensors/Sensor.cc
  src/sensors/ConstantVelocity.cc
  src/sensors/IMU.cc

  # Rosify Difodo
  src/rosify_difodo/CROSDifodo.cc

  # Extra
  src/extra/utils.cc
)

if(NOT USE_ANDROID AND USE_PANGOLIN)
  LIST(APPEND SOURCEFILES
    # UI
    src/ui/FrameDrawer.cc
    src/ui/MapDrawer.cc
    src/ui/Viewer.cc
    src/ui/Plane.cc
  )
endif()

# Set g2o sourcefiles
LIST(APPEND SOURCEFILES
  # g2o types
  src/extra/g2o/types/types_sba.h
  src/extra/g2o/types/types_six_dof_expmap.h
  src/extra/g2o/types/types_sba.cpp
  src/extra/g2o/types/types_six_dof_expmap.cpp
  src/extra/g2o/types/types_seven_dof_expmap.cpp
  src/extra/g2o/types/types_seven_dof_expmap.h
  src/extra/g2o/types/se3quat.h
  src/extra/g2o/types/se3_ops.h
  src/extra/g2o/types/se3_ops.hpp
  # g2o core
  src/extra/g2o/core/base_edge.h
  src/extra/g2o/core/base_binary_edge.h
  src/extra/g2o/core/hyper_graph_action.cpp
  src/extra/g2o/core/base_binary_edge.hpp
  src/extra/g2o/core/hyper_graph_action.h
  src/extra/g2o/core/base_multi_edge.h
  src/extra/g2o/core/hyper_graph.cpp
  src/extra/g2o/core/base_multi_edge.hpp
  src/extra/g2o/core/hyper_graph.h
  src/extra/g2o/core/base_unary_edge.h
  src/extra/g2o/core/linear_solver.h
  src/extra/g2o/core/base_unary_edge.hpp
  src/extra/g2o/core/marginal_covariance_cholesky.cpp
  src/extra/g2o/core/base_vertex.h
  src/extra/g2o/core/marginal_covariance_cholesky.h
  src/extra/g2o/core/base_vertex.hpp
  src/extra/g2o/core/matrix_structure.cpp
  src/extra/g2o/core/batch_stats.cpp
  src/extra/g2o/core/matrix_structure.h
  src/extra/g2o/core/batch_stats.h
  src/extra/g2o/core/openmp_mutex.h
  src/extra/g2o/core/block_solver.h
  src/extra/g2o/core/block_solver.hpp
  src/extra/g2o/core/parameter.cpp
  src/extra/g2o/core/parameter.h
  src/extra/g2o/core/cache.cpp
  src/extra/g2o/core/cache.h
  src/extra/g2o/core/optimizable_graph.cpp
  src/extra/g2o/core/optimizable_graph.h
  src/extra/g2o/core/solver.cpp
  src/extra/g2o/core/solver.h
  src/extra/g2o/core/creators.h
  src/extra/g2o/core/optimization_algorithm_factory.cpp
  src/extra/g2o/core/estimate_propagator.cpp
  src/extra/g2o/core/optimization_algorithm_factory.h
  src/extra/g2o/core/estimate_propagator.h
  src/extra/g2o/core/factory.cpp
  src/extra/g2o/core/optimization_algorithm_property.h
  src/extra/g2o/core/factory.h
  src/extra/g2o/core/sparse_block_matrix.h
  src/extra/g2o/core/sparse_optimizer.cpp
  src/extra/g2o/core/sparse_block_matrix.hpp
  src/extra/g2o/core/sparse_optimizer.h
  src/extra/g2o/core/hyper_dijkstra.cpp
  src/extra/g2o/core/hyper_dijkstra.h
  src/extra/g2o/core/parameter_container.cpp
  src/extra/g2o/core/parameter_container.h
  src/extra/g2o/core/optimization_algorithm.cpp
  src/extra/g2o/core/optimization_algorithm.h
  src/extra/g2o/core/optimization_algorithm_with_hessian.cpp
  src/extra/g2o/core/optimization_algorithm_with_hessian.h
  src/extra/g2o/core/optimization_algorithm_levenberg.cpp
  src/extra/g2o/core/optimization_algorithm_levenberg.h
  src/extra/g2o/core/jacobian_workspace.cpp
  src/extra/g2o/core/jacobian_workspace.h
  src/extra/g2o/core/robust_kernel.cpp
  src/extra/g2o/core/robust_kernel.h
  src/extra/g2o/core/robust_kernel_factory.cpp
  src/extra/g2o/core/robust_kernel_factory.h
  src/extra/g2o/core/robust_kernel_impl.cpp
  src/extra/g2o/core/robust_kernel_impl.h
  # g2o stuff
  src/extra/g2o/stuff/string_tools.h
  src/extra/g2o/stuff/color_macros.h
  src/extra/g2o/stuff/macros.h
  src/extra/g2o/stuff/timeutil.cpp
  src/extra/g2o/stuff/misc.h
  src/extra/g2o/stuff/timeutil.h
  src/extra/g2o/stuff/string_tools.cpp
  src/extra/g2o/stuff/property.cpp
  src/extra/g2o/stuff/property.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY src/ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

if(USE_ANDROID)
  LIST(APPEND REQUIRED_LIBRARIES lib_opencv log)
else()
  LIST(APPEND REQUIRED_LIBRARIES ${OpenCV_LIBS})
  if(USE_PANGOLIN)
    LIST(APPEND REQUIRED_LIBRARIES ${Pangolin_LIBRARIES})
  endif()
endif()

target_link_libraries(${PROJECT_NAME} ${REQUIRED_LIBRARIES} ${catkin_LIBRARIES} ${MRPT_LIBRARIES})

if(NOT USE_ANDROID)
  # Build examples
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

  add_executable(monocular
  Examples/Monocular/monocular.cc)
  target_link_libraries(monocular ${PROJECT_NAME})

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

  add_executable(rgbd
  Examples/RGB-D/rgbd.cc)
  target_link_libraries(rgbd ${PROJECT_NAME})

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Fusion)

  add_executable(monocular_imu
  Examples/Fusion/monocular_imu.cc)
  target_link_libraries(monocular_imu ${PROJECT_NAME})

  # Calibration
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)

  add_executable(calibration
  Examples/Calibration/calibration.cc)
  target_link_libraries(calibration ${PROJECT_NAME})
endif()
