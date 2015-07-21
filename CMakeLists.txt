cmake_minimum_required(VERSION 2.6)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0009 NEW)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
project(pipeline)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH})

# project dependecies
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_package(Caffe REQUIRED)
find_package(LMDB REQUIRED)
find_package(CUDA REQUIRED)
find_package(Boost COMPONENTS system filesystem serialization program_options REQUIRED)

file(GLOB src_main RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
file(GLOB hdr_main RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)

file(GLOB src_common RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} common/*.cpp)
file(GLOB hdr_common RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} common/*.h common/*.hpp)

file(GLOB src_datastructure RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} datastructure/*.cpp)
file(GLOB hdr_datastructure RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} datastructure/*.h datastructure/*.hpp)

file(GLOB src_util RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} util/*.cpp)
file(GLOB hdr_util RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} util/*.h util/*.hpp)

add_subdirectory(deps)

# remove all previously set compiler and linker flags for debug build
if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "")
    set(CMAKE_EXE_LINKER_FLAGS "")
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # using Clang or GCC
    # minimal optimmization and debug symbols for debug builds
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
    # enable optimization for release builds
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -DQT_NO_DEBUG")
    # enable C++1y
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -fPIE")
    # enable all warnings
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Woverloaded-virtual -Wold-style-cast -Wnon-virtual-dtor -Wsign-promo -Wno-missing-braces")
    # warnings are errors
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wno-error=unused-variable -Wno-error=unused-parameter")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # using Visual Studio C++
    # Force to always compile with W3 and treat warnings as errors
    # W4 would be preferable, but causes to many warnings in included files
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
    endif()
	# warnings are errors
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
	# disable warning 4503 on visual studio (boost)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4503")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# determine clang version
	EXECUTE_PROCESS( COMMAND ${CMAKE_CXX_COMPILER} --version OUTPUT_VARIABLE clang_full_version_string )
	string (REGEX REPLACE ".*clang version ([0-9]+\\.[0-9]+).*" "\\1" CLANG_VERSION_STRING ${clang_full_version_string})
	if (CLANG_VERSION_STRING VERSION_GREATER 3.5)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=inconsistent-missing-override")
	endif()
	# gcc doesn't know about unused-private-field warning
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-private-field")
    # use Wdocumentation
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdocumentation")
    # enable thread safety analysis
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wthread-safety")
    # enable thread safety analysis
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wimplicit-fallthrough")
    # implicit conversion warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfloat-conversion")
    # osx clang3.6 throw additional warnings
    if(${APPLE})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inconsistent-missing-override -Wno-deprecated-declarations")
        set(CMAKE_CXX_LINK_FLAGS_DEBUG "-lc++abi")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_CXX_LINK_FLAGS_DEBUG}")
    endif()
endif()

message("${CLANG_VERSION}")

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # clang doesnt know about unused-but-set-variable warning
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-but-set-variable")

	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
					OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9)
		# implicit conversion warnings
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfloat-conversion")
	endif()
endif()

# use runtime analyzers when using clang/gcc in debug mode
if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND
		(GCC_VERSION VERSION_GREATER 4.9 OR GCC_VERSION VERSION_EQUAL 4.9))
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-optimize-sibling-calls")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined")
	endif()

	if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-optimize-sibling-calls")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined")
		set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=integer")
	endif()
endif()

add_library(Pipeline STATIC
    ${src_main} ${hdr_main}
    ${src_common} ${hdr_common}
    ${src_datastructure} ${hdr_datastructure}
    ${src_util} ${hdr_util}
)


message(${DEEPLOCALIZER_CLASSIFIER_INCLUDE_DIR})
message(${DEEPLOCALIZER_CLASSIFIER_LIBRARY_DIR})

include_directories(
    SYSTEM ${DEEPLOCALIZER_CLASSIFIER_INCLUDE_DIR}
    SYSTEM ${Caffe_INCLUDE_DIRS}
    SYSTEM ${LMDB_INCLUDE_DIRS}
    SYSTEM ${CUDA_INCLUDE_DIRS}
)

link_directories(${DEEPLOCALIZER_CLASSIFIER_LIBRARY_DIR})

target_link_libraries(Pipeline
    "deeplocalizer-classifier"
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
)

