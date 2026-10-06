# Static DBoW2 + g2o + ORB-SLAM3 core for iOS (arm64) and, with the same sources and flags,
# for the macOS host (regression check and vocabulary conversion). See BUILD_IOS.md.
cmake_minimum_required(VERSION 3.20)
project(ORB_SLAM3_ios CXX C)

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/..)

option(ORBSLAM3_NO_VIEWER "Compile without Pangolin (Viewer/MapDrawer drawing compiled out)" ON)
option(ORBSLAM3_NO_SERIALIZATION "Compile without Boost serialization and OpenSSL (no atlas save/load)" ON)
option(ORBSLAM3_HOST_TOOLS "Build the host driver and vocabulary converter (macOS only)" OFF)
set(EIGEN3_INCLUDE_DIR "" CACHE PATH "Eigen 3.4 include dir (contains Eigen/)")
set(OPENCV_FRAMEWORK "" CACHE PATH "iOS: path to opencv2.framework")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
  message(FATAL_ERROR "EIGEN3_INCLUDE_DIR must point at Eigen 3.4 headers")
endif()

# OpenCV: the prebuilt static framework on iOS, an explicit OpenCV_DIR on the host.
add_library(orbslam3_opencv INTERFACE)
if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  if(NOT EXISTS "${OPENCV_FRAMEWORK}/Headers/core.hpp")
    message(FATAL_ERROR "OPENCV_FRAMEWORK must point at opencv2.framework")
  endif()
  get_filename_component(_cv_fw_dir "${OPENCV_FRAMEWORK}" DIRECTORY)
  target_compile_options(orbslam3_opencv INTERFACE "-F${_cv_fw_dir}")
  target_link_libraries(orbslam3_opencv INTERFACE "${OPENCV_FRAMEWORK}")
else()
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)
  target_include_directories(orbslam3_opencv INTERFACE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(orbslam3_opencv INTERFACE ${OpenCV_LIBS})
endif()

set(ORBSLAM3_DEFS COMPILEDWITHC11)
if(ORBSLAM3_NO_VIEWER)
  list(APPEND ORBSLAM3_DEFS ORBSLAM3_NO_VIEWER)
endif()
if(ORBSLAM3_NO_SERIALIZATION)
  list(APPEND ORBSLAM3_DEFS ORBSLAM3_NO_SERIALIZATION)
endif()
# Only Pangolin and Boost/OpenSSL are compiled out; everything below is plain source.
if(NOT ORBSLAM3_NO_VIEWER OR NOT ORBSLAM3_NO_SERIALIZATION)
  message(FATAL_ERROR "This build file only supports the minimal configuration; use ORB_SLAM3/CMakeLists.txt for the full one")
endif()

set(WARN -Wall -Wno-deprecated-declarations -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare)

# --- DBoW2 ---------------------------------------------------------------
add_library(DBoW2 STATIC
  ${SRC}/Thirdparty/DBoW2/DBoW2/BowVector.cpp
  ${SRC}/Thirdparty/DBoW2/DBoW2/FORB.cpp
  ${SRC}/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp
  ${SRC}/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp
  ${SRC}/Thirdparty/DBoW2/DUtils/Random.cpp
  ${SRC}/Thirdparty/DBoW2/DUtils/Timestamp.cpp)
target_include_directories(DBoW2 PUBLIC ${SRC}/compat)
target_compile_definitions(DBoW2 PUBLIC ${ORBSLAM3_DEFS})
target_compile_options(DBoW2 PRIVATE ${WARN})
target_link_libraries(DBoW2 PUBLIC orbslam3_opencv)

# --- g2o (same file list as Thirdparty/g2o/CMakeLists.txt; config.h has G2O_OPENMP off) ---
file(GLOB_RECURSE G2O_SRCS ${SRC}/Thirdparty/g2o/g2o/*.cpp ${SRC}/Thirdparty/g2o/g2o/*.c)
list(FILTER G2O_SRCS EXCLUDE REGEX "(optimization_algorithm_dogleg|sparse_block_matrix_test|se3mat)\\.cpp$")
configure_file(${SRC}/Thirdparty/g2o/config.h.in ${SRC}/Thirdparty/g2o/config.h)
add_library(g2o STATIC ${G2O_SRCS})
target_include_directories(g2o PUBLIC ${SRC}/Thirdparty/g2o ${EIGEN3_INCLUDE_DIR})
target_compile_definitions(g2o PRIVATE UNIX)
target_compile_options(g2o PRIVATE ${WARN} -W)

# --- ORB-SLAM3 core --------------------------------------------------------
set(ORBSLAM3_SRCS
  System.cc Tracking.cc LocalMapping.cc LoopClosing.cc ORBextractor.cc ORBmatcher.cc
  FrameDrawer.cc Converter.cc MapPoint.cc KeyFrame.cc Atlas.cc Map.cc MapDrawer.cc
  Optimizer.cc Frame.cc KeyFrameDatabase.cc Sim3Solver.cc Viewer.cc ImuTypes.cc
  G2oTypes.cc CameraModels/Pinhole.cpp CameraModels/KannalaBrandt8.cpp
  OptimizableTypes.cpp MLPnPsolver.cpp GeometricTools.cc TwoViewReconstruction.cc
  Config.cc Settings.cc)
list(TRANSFORM ORBSLAM3_SRCS PREPEND ${SRC}/src/)
add_library(ORB_SLAM3 STATIC ${ORBSLAM3_SRCS})
target_include_directories(ORB_SLAM3 PUBLIC
  ${SRC} ${SRC}/include ${SRC}/include/CameraModels ${SRC}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR})
target_compile_options(ORB_SLAM3 PRIVATE ${WARN} -Wno-reorder-ctor -Wno-unused-private-field -Wno-overloaded-virtual)
target_link_libraries(ORB_SLAM3 PUBLIC DBoW2 g2o orbslam3_opencv)

# One archive with all three libraries, for dropping into an Xcode project.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/lib/liborbslam3_all.a
  COMMAND libtool -static -no_warning_for_no_symbols -o ${CMAKE_BINARY_DIR}/lib/liborbslam3_all.a
          $<TARGET_FILE:ORB_SLAM3> $<TARGET_FILE:DBoW2> $<TARGET_FILE:g2o>
  DEPENDS ORB_SLAM3 DBoW2 g2o VERBATIM)
add_custom_target(orbslam3_all ALL DEPENDS ${CMAKE_BINARY_DIR}/lib/liborbslam3_all.a)

if(CMAKE_SYSTEM_NAME STREQUAL "iOS")
  # Link test: an unsigned app bundle that references System/TrackRGBD/Shutdown.
  add_executable(orbslam3_linktest MACOSX_BUNDLE ${CMAKE_CURRENT_SOURCE_DIR}/linktest/linktest.cc)
  set_target_properties(orbslam3_linktest PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER org.example.orbslam3linktest
    MACOSX_BUNDLE_BUNDLE_NAME orbslam3_linktest)
  target_link_options(orbslam3_linktest PRIVATE "-Wl,-dead_strip")
  # Nothing beyond the OpenCV framework, libc++ and libSystem is needed for the symbols used.
  target_link_libraries(orbslam3_linktest PRIVATE ORB_SLAM3)
elseif(ORBSLAM3_HOST_TOOLS)
  add_executable(rgbd_inertial_euroc_folder ${SRC}/Examples/RGB-D-Inertial/rgbd_inertial_euroc_folder.cc)
  target_link_libraries(rgbd_inertial_euroc_folder PRIVATE ORB_SLAM3)
  add_executable(voc_convert ${CMAKE_CURRENT_SOURCE_DIR}/tools/voc_convert.cc)
  target_link_libraries(voc_convert PRIVATE ORB_SLAM3)
endif()
