cmake_minimum_required(VERSION 3.18)
project(darknet_ros LANGUAGES CXX C CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Define path of darknet folder here.
find_path(DARKNET_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../darknet/")
message(STATUS "Darknet path dir = ${DARKNET_PATH}")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

# Check for CUDA support using modern CMake
# Try to enable CUDA language first
enable_language(CUDA)

# Check if CUDA compiler is available
if(CMAKE_CUDA_COMPILER)
  message(STATUS "CUDA found - Building with GPU support")
  message(STATUS "CUDA Version: ${CMAKE_CUDA_COMPILER_VERSION}")
  message(STATUS "CUDA Compiler: ${CMAKE_CUDA_COMPILER}")
  
  # Find CUDA toolkit for linking libraries
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    message(STATUS "CUDAToolkit found: ${CUDAToolkit_VERSION}")
  endif()
  
  # Set CUDA architectures for modern CMake (CMake 3.18+)
  # compute_75 = sm_75 (RTX 20xx series, GTX 16xx series)
  # compute_86 = sm_86 (RTX 30xx series)
  # compute_87 = sm_87 (RTX 40xx series)
  set(CMAKE_CUDA_ARCHITECTURES "75;86;87")
  
  # Set CUDA compilation flags
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")
  set(CMAKE_CUDA_FLAGS_RELEASE "-O3")
  
  # Enable separable compilation for device linking
  set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
  
  add_definitions(-DGPU)
  set(GPU_ENABLED ON)
else()
  message(WARNING "CUDA not found - building without GPU support. Install CUDA toolkit to enable GPU acceleration.")
  set(GPU_ENABLED OFF)
endif()

# Find X11
message ( STATUS "Searching for X11..." )
find_package ( X11 REQUIRED )
if ( X11_FOUND )
  include_directories ( ${X11_INCLUDE_DIR} )
  link_libraries ( ${X11_LIBRARIES} )
  message ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  message ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
endif ( X11_FOUND )

# Find rquired packeges
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
    rclcpp
    rclpy
    rclcpp_action
    std_msgs
    image_transport
    cv_bridge
    sensor_msgs
    darknet_ros_msgs
    ament_index_cpp
)

# Enable OPENCV in darknet
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

include_directories(
  ${DARKNET_PATH}/src
  ${DARKNET_PATH}/include
  include
  ${catkin_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
    src/YoloObjectDetector.cpp                    src/image_interface.c
    src/opencv4_compat.cpp
)

set(DARKNET_CORE_FILES
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/activations.c             ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/avgpool_layer.c           ${DARKNET_PATH}/src/layer.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/list.c
    ${DARKNET_PATH}/src/blas.c                    ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/lstm_layer.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/connected_layer.c         ${DARKNET_PATH}/src/maxpool_layer.c
    ${DARKNET_PATH}/src/convolutional_layer.c     ${DARKNET_PATH}/src/network.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/normalization_layer.c
    ${DARKNET_PATH}/src/crnn_layer.c              ${DARKNET_PATH}/src/option_list.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/cuda.c                    ${DARKNET_PATH}/src/region_layer.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/deconvolutional_layer.c   ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/demo.c                    ${DARKNET_PATH}/src/route_layer.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/dropout_layer.c           ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/upsample_layer.c          ${DARKNET_PATH}/src/logistic_layer.c
    ${DARKNET_PATH}/src/l2norm_layer.c            ${DARKNET_PATH}/src/yolo_layer.c

    ${DARKNET_PATH}/examples/art.c                ${DARKNET_PATH}/examples/lsd.c
    ${DARKNET_PATH}/examples/attention.c          ${DARKNET_PATH}/examples/nightmare.c
    ${DARKNET_PATH}/examples/captcha.c            ${DARKNET_PATH}/examples/regressor.c
    ${DARKNET_PATH}/examples/cifar.c              ${DARKNET_PATH}/examples/rnn.c
    ${DARKNET_PATH}/examples/classifier.c         ${DARKNET_PATH}/examples/segmenter.c
    ${DARKNET_PATH}/examples/coco.c               ${DARKNET_PATH}/examples/super.c
    # ${DARKNET_PATH}/examples/darknet.c            # Excluded: has main() function, conflicts with ROS node
    ${DARKNET_PATH}/examples/tag.c
    ${DARKNET_PATH}/examples/detector.c           ${DARKNET_PATH}/examples/yolo.c
    ${DARKNET_PATH}/examples/go.c
)

set(DARKNET_CUDA_FILES
    ${DARKNET_PATH}/src/activation_kernels.cu     ${DARKNET_PATH}/src/crop_layer_kernels.cu
    ${DARKNET_PATH}/src/avgpool_layer_kernels.cu  ${DARKNET_PATH}/src/deconvolutional_kernels.cu
    ${DARKNET_PATH}/src/blas_kernels.cu           ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/col2im_kernels.cu         ${DARKNET_PATH}/src/im2col_kernels.cu
    ${DARKNET_PATH}/src/convolutional_kernels.cu  ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
)

# Set C files to compile as C++ to support OpenCV 4.x headers
# Note: cuda.c will be compiled as C++ but cuda_gridsize needs extern "C" linkage
set_source_files_properties(${PROJECT_LIB_FILES} ${DARKNET_CORE_FILES} PROPERTIES LANGUAGE CXX COMPILE_FLAGS "-fpermissive")

if(GPU_ENABLED)
  # Create static library WITHOUT CUDA files
  # CUDA files will be linked directly to the executable for proper device linking
  # The library contains only host code that calls CUDA functions
  add_library(${PROJECT_NAME}_lib STATIC
    ${PROJECT_LIB_FILES} ${DARKNET_CORE_FILES}
  )
  
  ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

  # Link CUDA libraries using modern CMake approach
  if(CUDAToolkit_FOUND)
    target_link_libraries(${PROJECT_NAME}_lib
      CUDA::cuda_driver
      CUDA::cudart
      CUDA::cublas
      CUDA::curand
    )
  else()
    # Fallback to finding libraries manually
    find_library(CUDA_LIBRARY cuda PATHS ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
    find_library(CUDART_LIBRARY cudart PATHS ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
    find_library(CUBLAS_LIBRARY cublas PATHS ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
    find_library(CURAND_LIBRARY curand PATHS ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
    
    target_link_libraries(${PROJECT_NAME}_lib
      ${CUDA_LIBRARY}
      ${CUDART_LIBRARY}
      ${CUBLAS_LIBRARY}
      ${CURAND_LIBRARY}
    )
  endif()

  # Create executable with CUDA support using modern CMake
  # Include CUDA files directly in executable for proper device linking
  # The static library provides host code, executable provides CUDA device code
  add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp
    ${DARKNET_CUDA_FILES}
  )
  
  # Set CUDA language standard for executable
  set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_STANDARD 17)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_STANDARD_REQUIRED ON)
  
  # Enable CUDA separable compilation for proper device linking
  set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_RESOLVE_DEVICE_SYMBOLS ON)
  
  # Set CUDA architectures for the executable target
  set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES "75;86;87")
  
  # IMPORTANT: Link the library BEFORE other libraries to ensure device linking works
  # The --whole-archive flag ensures all symbols (including CUDA device code) are included
  
  # Link the static library FIRST (before CUDA libraries) to ensure symbols are available
  # for device linking. The --whole-archive flag ensures all symbols are included.
  target_link_libraries(${PROJECT_NAME}
    -Wl,--whole-archive
    ${PROJECT_NAME}_lib
    -Wl,--no-whole-archive
  )
  
  # Link CUDA libraries to executable using modern CMake approach
  if(CUDAToolkit_FOUND)
    target_link_libraries(${PROJECT_NAME}
      CUDA::cuda_driver
      CUDA::cudart
      CUDA::cublas
      CUDA::curand
    )
  else()
    # Fallback to finding libraries manually
    target_link_libraries(${PROJECT_NAME}
      ${CUDA_LIBRARY}
      ${CUDART_LIBRARY}
      ${CUBLAS_LIBRARY}
      ${CURAND_LIBRARY}
    )
  endif()
  
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})

else()
  # Non-GPU build
  add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES} ${DARKNET_CORE_FILES}
  )
  
  ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

  add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp
  )
  ament_target_dependencies(${PROJECT_NAME} ${dependencies})

endif()

target_link_libraries(${PROJECT_NAME}_lib
  m
  pthread
  stdc++
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# For non-GPU builds, link the library normally
if(NOT GPU_ENABLED)
  target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_lib
  )
endif()
# For GPU builds, the library is already linked above (before CUDA libraries)


install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h"
)

# Download yolov2-tiny.weights
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
set(FILE "${PATH}/yolov2-tiny.weights")
message(STATUS "Checking and downloading yolov2-tiny.weights if needed ...")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget http://pjreddie.com/media/files/yolov2-tiny.weights -P ${PATH})
endif()

# Download yolov3.weights
set(FILE "${PATH}/yolov3.weights")
message(STATUS "Checking and downloading yolov3.weights if needed ...")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget http://pjreddie.com/media/files/yolov3.weights -P ${PATH})
endif()

install(DIRECTORY yolo_network_config/cfg yolo_network_config/weights DESTINATION share/${PROJECT_NAME}/yolo_network_config/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

#############
## Testing ##
#############

if(BUILD_TESTING)
  # Download yolov2.weights
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
  set(FILE "${PATH}/yolov2.weights")
  message(STATUS "Checking and downloading yolov2.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget http://pjreddie.com/media/files/yolov2.weights -P ${PATH})
  endif()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  
  #ament_add_gtest(${PROJECT_NAME}_object_detection-test
  #  test/object_detection.test
  #  test/test_main.cpp
  #  test/ObjectDetection.cpp
  #)
  #target_link_libraries(${PROJECT_NAME}_object_detection-test
  #  ${PROJECT_NAME}_lib
  #)
endif()


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${dependencies})

ament_package()