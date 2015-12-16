cmake_minimum_required(VERSION 2.8.7)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0009 NEW)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
project(pipeline)
#-----------------------------------------------------------------------
# CPM configuration
#-----------------------------------------------------------------------
set(CPM_MODULE_NAME Pipeline)
set(CPM_LIB_TARGET_NAME ${CPM_MODULE_NAME})

if ((DEFINED CPM_DIR) AND (DEFINED CPM_UNIQUE_ID) AND (DEFINED CPM_TARGET_NAME))
  set(CPM_LIB_TARGET_NAME ${CPM_TARGET_NAME})
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CPM_DIR})
  include(CPM)
else()
  set(CPM_DIR "${CMAKE_CURRENT_BINARY_DIR}/cpm-packages" CACHE TYPE STRING)
  find_package(Git)
  if(NOT GIT_FOUND)
    message(FATAL_ERROR "CPM requires Git.")
  endif()
  if (NOT EXISTS ${CPM_DIR}/CPM.cmake)
    message(STATUS "Cloning repo (https://github.com/iauns/cpm)")
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" clone https://github.com/iauns/cpm ${CPM_DIR}
      RESULT_VARIABLE error_code
      OUTPUT_QUIET ERROR_QUIET)
    if(error_code)
      message(FATAL_ERROR "CPM failed to get the hash for HEAD")
    endif()
  endif()
  include(${CPM_DIR}/CPM.cmake)
endif()

if(NOT DEFINED CMAKECONFIG_PATH)
    CPM_AddModule("cmakeconfig"
        GIT_REPOSITORY "https://github.com/BioroboticsLab/cmakeconfig.git"
        GIT_TAG "master")
else()
    CPM_AddModule("cmakeconfig"
        SOURCE_DIR "${CMAKECONFIG_PATH}")
endif()

if(NOT DEFINED USE_DEEPLOCALIZER)
    set(USE_DEEPLOCALIZER "1")

endif()

if(${USE_DEEPLOCALIZER})
    message("**********************************************************************************************")
    message("Using neuronal network to improve localizer")
    message("Make sure you adapt the following settings:")
    message("   Set LOCALIZER.DEEPLOCALIZER_MODEL_FILE to your local caffe *.prototxt file")
    message("   Set LOCALIZER.DEEPLOCALIZER_PARAM_FILE to your local caffe *.caffemodel file")
    message("To disable this feature set USE_DEEPLOCALIZER to 0 before calling CPM_AddModule(Pipeline ...).")
    message("**********************************************************************************************")
    include_deeplocalizer_classifier()
    CPM_ExportAdditionalDefinition("-DUSE_DEEPLOCALIZER=1")
else()
    CPM_ExportAdditionalDefinition("-DUSE_DEEPLOCALIZER=0")
endif()

CPM_InitModule(${CPM_MODULE_NAME})

biorobotics_config()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH})

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS system filesystem serialization program_options REQUIRED)

if(${USE_DEEPLOCALIZER})
    find_package(CUDA REQUIRED)
endif()


file(GLOB_RECURSE src RELEATIVE ${CMAKE_CURRENT_SOURCE_DIR} pipeline/*.cpp)
file(GLOB_RECURSE hdr RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} pipeline/*.h pipeline/*.hpp)

add_library(${CPM_LIB_TARGET_NAME} ${src} ${hdr})
add_definitions("-DUSE_DEEPLOCALIZER=${USE_DEEPLOCALIZER}")

include_directories(
    SYSTEM ${Boost_INCLUDE_DIRS}
    SYSTEM ${OpenCV_INCLUDE_DIRS}
    SYSTEM ${Threads_INCLUDE_DIRS}
)

set(LIBS
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${CPM_LIBRARIES})

if(${USE_DEEPLOCALIZER})
    set(LIBS ${LIBS} ${CUDA_LIBRARIES})
    include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
endif()

target_link_libraries(${CPM_LIB_TARGET_NAME} ${LIBS})
