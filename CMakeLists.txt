cmake_minimum_required(VERSION 2.8.11)
project(Tungsten)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

include(CXX11)
check_for_cxx11_compiler(CXX11_COMPILER)

if(CXX11_COMPILER)
    enable_cxx11()
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} seems to have no C++11 support. Please try again with a more recent compiler version.")
endif()

# AVX does not do much benefit at the moment, but breaks compilation on some platforms.
# Disabled for now until AVX becomes important enough to reconsider.
#SET(EMBREE_MAX_ISA "SSE4.2" CACHE STRING "Selects highest ISA to support.")
#set(USE_AVX FALSE CACHE BOOL "Use AVX.")
#set(USE_AVX2 FALSE CACHE BOOL "Use AVX2.")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

if (MSVC)
    # Needed by MSVC, but not added by OptimizeForArchitexture()
    add_definitions(-D__SSE__)
endif()


    message(STATUS "Compiling with AVX support")
    set(__AVX__ 1)

if (MSVC)
    add_definitions(-DCONSTEXPR=const -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS)
else()
    add_definitions(-DCONSTEXPR=constexpr)
endif()

IF(COMMAND cmake_policy)
  if (POLICY CMP0043)
    cmake_policy(SET CMP0043 NEW)
  endif()
ENDIF(COMMAND cmake_policy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Vc_ARCHITECTURE_FLAGS}")

#set(EMBREE_STATIC_LIB ON CACHE BOOL "Build Embree as a static library." FORCE)
#set(EMBREE_ISPC_SUPPORT OFF CACHE BOOL "Build Embree with support for ISPC applications." FORCE)
#set(EMBREE_TUTORIALS OFF CACHE BOOL "Enable to build Embree tutorials" FORCE)
#set(EMBREE_STAT_COUNTERS OFF CACHE BOOL "Enables statistic counters." FORCE)
#set(EMBREE_RAY_MASK OFF CACHE BOOL "Enables ray mask support." FORCE)
#set(EMBREE_BACKFACE_CULLING OFF CACHE BOOL "Enables backface culling." FORCE)
#set(EMBREE_INTERSECTION_FILTER ON CACHE BOOL "Enables intersection filter callback." FORCE)
#set(EMBREE_INTERSECTION_FILTER_RESTORE ON CACHE BOOL "Restores previous hit when hit is filtered out." FORCE)
#set(EMBREE_TASKING_SYSTEM "INTERNAL" CACHE STRING "Selects tasking system" FORCE)
#add_subdirectory(src/thirdparty/embree)
#add_definitions(-DEMBREE_STATIC_LIB=1)

add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)
add_definitions(-DSTBI_NO_STDIO=1)
add_definitions(-DLODEPNG_NO_COMPILE_DISK=1)
add_definitions(-DUSE_IPV6=1)
add_library(thirdparty STATIC
		src/thirdparty/lodepng/lodepng.cpp
		src/thirdparty/sobol/sobol.cpp
		src/thirdparty/stbi/stb_image.c
                #src/thirdparty/miniz/miniz.c
		src/thirdparty/skylight/ArHosekSkyModel.cpp)

if (CMAKE_COMPILER_IS_GNUCXX)
    set(CXX_WARNINGS "-Wall -Wextra -Wpointer-arith -Wcast-align -fstrict-aliasing -Wno-unused-local-typedefs -Wno-misleading-indentation")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_WARNINGS} -fvisibility-inlines-hidden")
endif()
set(core_libs core thirdparty embree)

include_directories(src/core src/thirdparty src/thirdparty/embree/include src)

message(STATUS "OpenEXR headers are incompatible with C++1z. Building without .exr support")

find_package(JPEG)
if (JPEG_FOUND)
    message(STATUS "JPEG library detected.")
    add_definitions(-DJPEG_AVAILABLE)
    include_directories(${JPEG_INCLUDE_DIR})
    set(core_libs ${core_libs} ${JPEG_LIBRARIES})
else()
    message(STATUS "No JPEG library detected. Falling back to stb_image JPG decoder")
endif()

find_package(OpenVDB)
find_package(TBB)
if (OPENVDB_FOUND AND TBB_FOUND)
    message(STATUS "OpenVDB detected. Building with .vdb support")
    add_definitions(-DOPENVDB_AVAILABLE)
    include_directories(${OPENVDB_INCLUDE_DIR})
    set(core_libs ${core_libs} ${OPENVDB_LIBRARIES} ${TBB_LIBRARIES})
else()
    if (NOT OPENVDB_FOUND)
        message(STATUS "No OpenVDB detected. Building without .vdb support")
    else()
        message(STATUS "No TBB detected. Building without .vdb support")
    endif()
endif()

file(GLOB_RECURSE Serenity_Core_SOURCES "serenity/core/*.cc")
add_library(serenity_core STATIC ${Serenity_Core_SOURCES})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -iquote ${PROJECT_SOURCE_DIR}/serenity/core")
set(core_libs ${core_libs} serenity_core)

include_directories(/usr/include/freetype2)
file(GLOB_RECURSE Serenity_IO_SOURCES "serenity/io/*.cc")
add_library(serenity_io STATIC ${Serenity_IO_SOURCES})
target_link_libraries(serenity_io freetype)
set(core_libs ${core_libs} serenity_io)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -iquote ${PROJECT_SOURCE_DIR}/serenity/io")

file(GLOB_RECURSE Serenity_UI_SOURCES "serenity/ui/*.cc")
add_library(serenity_ui STATIC ${Serenity_UI_SOURCES})
target_link_libraries(serenity_ui serenity_core serenity_io)
set(core_libs ${core_libs} serenity_ui)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -iquote ${PROJECT_SOURCE_DIR}/serenity/ui")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -iquote ${PROJECT_SOURCE_DIR}/serenity")

file(GLOB_RECURSE Core_SOURCES "src/core/*.cpp")
add_library(core STATIC ${Core_SOURCES})

add_executable(tungsten src/tungsten/tungsten.cpp)
target_link_libraries(tungsten ${core_libs} pthread)
