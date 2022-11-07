cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0017 NEW)
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.0.0)
    cmake_policy(SET CMP0042 NEW)
endif()
project(ompl_source CXX C)

find_package(catkin REQUIRED)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  # By default, use Release mode
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)

  # On 32bit architectures, use RelWithDebInfo
  if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
  endif()
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_CURRENT_SOURCE_DIR}/ompl/CMakeModules")
include(GNUInstallDirs)
include(CompilerSettings)
include(OMPLVersion)
include(OMPLUtils)

# OMPL include path
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ompl/src")

if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB)
  add_definitions(-DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)
if(IS_ICPC)
  set(Boost_USE_STATIC_LIBS ON CACHE STRING "Use statically linked Boost libraries")
else(IS_ICPC)
  # Ensure dynamic linking with boost unit_test_framework
  add_definitions(-DBOOST_TEST_DYN_LINK)
endif(IS_ICPC)

# Do one quiet find_package on Boost to see if it is recent enough to
# have the try_join_for call
find_package(Boost 1.54 COMPONENTS serialization filesystem system program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Add support in Boost::Python for std::shared_ptr
# This is a hack that replaces boost::shared_ptr related code with std::shared_ptr.
# Proper support for std::shared_ptr was added in Boost 1.63.
if(Boost_VERSION VERSION_LESS "106300")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/external")
endif()


# on OS X we need to check whether to use libc++ or libstdc++ with clang++
if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    include(GetPrerequisites)
    get_prerequisites("${Boost_SYSTEM_LIBRARY}" _libs 0 0 "/" "")
    set(CXXSTDLIB "")
    foreach(_lib ${_libs})
        if(_lib MATCHES "libc\\+\\+")
            set(CXXSTDLIB "libc++")
        elseif(_lib MATCHES "libstdc\\+\\+")
            set(CXXSTDLIB "libstdc++")
        endif()
    endforeach()
    if(CXXSTDLIB)
        add_definitions(-stdlib=${CXXSTDLIB})
    endif()
endif()

# Eigen is needed for the InformedStateSampler
find_package(Eigen3 QUIET)
if (EIGEN3_FOUND)
    set(OMPL_HAVE_EIGEN3 1)
    include_directories("${EIGEN3_INCLUDE_DIR}")
endif()

# If FLANN is installed, a wrapper for its nearest neighbor data structures can be used
find_package(flann 1.8.3 QUIET)
if (FLANN_FOUND)
    set(OMPL_HAVE_FLANN 1)
    include_directories("${FLANN_INCLUDE_DIRS}")
    link_directories(${FLANN_LIBRARY_DIRS})
endif()

# Barebones compilation only
set(OMPL_EXTENSION_MORSE 0)
set(OMPL_EXTENSION_OPENDE 0)
set(OMPL_EXTENSION_TRIANGLE 0)
option(OMPL_REGISTRATION OFF)

# CATKIN stuff
catkin_package(
  LIBRARIES
  ompl
  INCLUDE_DIRS
  ompl/src
  )

# Explicitly skipping the root level CMakeLists.txt in ompl.
# There is a conflict between catkin and ompl when setting the
# library output directory that causes a compilation error.
# Also, this allows us to skip compilation of tests, demos,
# and other superfluous code.
add_subdirectory(ompl/src)
