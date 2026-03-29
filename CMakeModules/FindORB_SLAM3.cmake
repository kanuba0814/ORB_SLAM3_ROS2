# Try to find ORB_SLAM3
# Set alternative paths to search for using ORB_SLAM3_DIR
# Once done this will define
# You should ensure your ORB_SLAM3 can run correctly
#
# To help the search set ORB_SLAM3_ROOT_DIR as the path to ORB_SLAM3 root folder
# e.g. `export ORB_SLAM3_ROOT_DIR=~/ORB_SLAM3`
if(NOT ORB_SLAM3_ROOT_DIR)
  if(DEFINED ENV{ORB_SLAM3_ROOT_DIR} AND NOT "$ENV{ORB_SLAM3_ROOT_DIR}" STREQUAL "")
    set(ORB_SLAM3_ROOT_DIR "$ENV{ORB_SLAM3_ROOT_DIR}")
  else()
    set(ORB_SLAM3_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3")
  endif()
endif()

set(_ORB_SLAM3_ROOT_HINTS
  "${ORB_SLAM3_ROOT_DIR}"
  "/usr/local"
  "/usr"
)

# Find built-in DBoW2
# Find ORB_SLAM3
find_path(ORB_SLAM3_INCLUDE_DIR NAMES System.h
          PATHS ${_ORB_SLAM3_ROOT_HINTS}
          PATH_SUFFIXES include)

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${_ORB_SLAM3_ROOT_HINTS}
             PATH_SUFFIXES lib)

if(ORB_SLAM3_INCLUDE_DIR)
  get_filename_component(_ORB_SLAM3_ROOT_FROM_INCLUDE "${ORB_SLAM3_INCLUDE_DIR}" DIRECTORY)
else()
  set(_ORB_SLAM3_ROOT_FROM_INCLUDE "${ORB_SLAM3_ROOT_DIR}")
endif()

find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS "${_ORB_SLAM3_ROOT_FROM_INCLUDE}" "${ORB_SLAM3_ROOT_DIR}")

find_library(DBoW2_LIBRARY NAMES DBoW2
          PATHS "${_ORB_SLAM3_ROOT_FROM_INCLUDE}/Thirdparty/DBoW2/lib"
                "${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib")

# Find built-in g2o
find_library(g2o_LIBRARY NAMES g2o
          PATHS "${_ORB_SLAM3_ROOT_FROM_INCLUDE}/Thirdparty/g2o/lib"
                "${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib")



include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM3  DEFAULT_MSG
                                  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY )

set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})
set(ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_INCLUDE_DIR} ${DBoW2_INCLUDE_DIR})
set(ORB_SLAM3_ROOT_DIR "${_ORB_SLAM3_ROOT_FROM_INCLUDE}" CACHE PATH "ORB_SLAM3 root directory")
