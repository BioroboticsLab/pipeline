# BeesBook Image Analysis Pipeline

This is the computer vision pipeline used in the BeesBook project to 
automatically detect and decode the IDs of tagged Bees.

For details see: [Automatic methods for long-term tracking and the detection and decoding of communication dances in honeybees](http://journal.frontiersin.org/article/10.3389/fevo.2015.00103/full)

## Dependencies

* GCC >= 4.8 or clang >= 3.6
* boost
* OpenCV

## Setup

### Setup CPM and BioroboticsLab/cmakeconfig for your project
In your CMakeLists.txt, add:
```CMake
set(CPM_MODULE_NAME YourModuleNameHere)
set(CPM_LIB_TARGET_NAME ${CPM_MODULE_NAME})

if ((DEFINED BIOROBOTICS_CMAKE_CONFIG_DIR))
    include(BIOROBOTICS_CMAKE_CONFIG_DIR)
else()
    set(BIOROBOTICS_CMAKE_CONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/cmakeconfig" CACHE TYPE STRING)
    find_package(Git)
    if(NOT GIT_FOUND)
      message(FATAL_ERROR "CPM requires Git.")
    endif()
    if (NOT EXISTS ${BIOROBOTICS_CMAKE_CONFIG_DIR}/CMakeLists.txt)
      message(STATUS "Cloning repo (https://github.com/BioroboticsLab/cmakeconfig.git)")
      execute_process(
        COMMAND "${GIT_EXECUTABLE}" clone https://github.com/BioroboticsLab/cmakeconfig.git
        ${BIOROBOTICS_CMAKE_CONFIG_DIR}
        RESULT_VARIABLE error_code
        OUTPUT_QUIET ERROR_QUIET)
      if(error_code)
          message(FATAL_ERROR "CMAKECONFIG failed to get the hash for HEAD")
      endif()
    endif()
    include(${BIOROBOTICS_CMAKE_CONFIG_DIR}/CMakeLists.txt)
endif()
```

### Add the BeesBook Image Analysis Pipeline as a CPM dependency
In your CMakeLists.txt, add:
```CMake
include_pipeline()
```

### Initialize CPM and BioroboticsLab C++ compilation config
In your CMakeLists.txt, add:
```CMake
CPM_InitModule(${CPM_MODULE_NAME})

biorobotics_config()
```

### Usage
```C++
#include <cstdlib>
#include <vector>

#include <opencv2/opencv.hpp>

#include <pipeline/datastructure/Tag.h>
#include <pipeline/Preprocessor.h>
#include <pipeline/Localizer.h>
#include <pipeline/EllipseFitter.h>
#include <pipeline/GridFitter.h>
#include <pipeline/Decoder.h>

int main(int argc, char** argv) 
{
  const std::string filename(argv[1]);
  
  cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  
  cv::Mat preprocessedImg = _preprocessor.process(img);
  
  std::vector<pipeline::Tag> taglist = _localizer.process(std::move(img), std::move(preprocessedImg));
  taglist = _ellipseFitter.process(std::move(taglist));
  taglist = _gridFitter.process(std::move(taglist));
  taglist = _decoder.process(std::move(taglist));
  
  return EXIT_SUCCESS;
}
```

### Citation
> Wario, Fernando, et al. "Automatic methods for long-term tracking and the detection and decoding of communication dances in honeybees." Frontiers in Ecology and Evolution 3 (2015): 103.
