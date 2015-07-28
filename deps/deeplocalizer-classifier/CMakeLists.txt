cmake_minimum_required(VERSION 2.8.8)
project(deeplocalizer-classifier-ext CXX)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    deeplocalizer-classifier-ext
    PREFIX ${CMAKE_BINARY_DIR}/deeplocalizer-classifier
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    GIT_REPOSITORY https://github.com/BioroboticsLab/deeplocalizer-classifier.git
    TIMEOUT 10
    BUILD_COMMAND "" 
    INSTALL_COMMAND 
        "@CMAKE_COMMAND@"
        --build .
        --config ${configuration}
        --
        ${jobs_option}
   )
   
   
ExternalProject_Get_Property(deeplocalizer-classifier-ext SOURCE_DIR)
set(DEEPLOCALIZER_CLASSIFIER_INCLUDE_DIR ${SOURCE_DIR}/include CACHE INTERNAL "Path to include dir for deeplocalizer classifier")

ExternalProject_Get_Property(deeplocalizer-classifier-ext BINARY_DIR)
set(DEEPLOCALIZER_CLASSIFIER_LIBRARY_DIR "${BINARY_DIR}/source/classifier/" CACHE INTERNAL "Path to lib for deeplocalizer classifier")
