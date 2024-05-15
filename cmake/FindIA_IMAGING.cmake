#
#  Copyright (C) 2017-2021 Intel Corporation
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

if(TARGET ia_aiq)
    message("libcamhal found ia_aiq target")
    return()
endif()

# Get include and lib paths for IA_IMAGING from pkgconfig
include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
pkg_check_modules(IA_IMAGING ia_imaging)
if(NOT IA_IMAGING_FOUND)
    message(FATAL_ERROR "IA_IMAGING not found")
endif()

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${IA_IMAGING_LIBRARY_DIRS})

# Libraries
find_library(IA_CCA_LIB         ia_cca)
find_library(IA_LOG_LIB         ia_log)

set(IA_IMAGING_LIBS
    ${IA_CCA_LIB}
    ${IA_LOG_LIB}
)

# handle the QUIETLY and REQUIRED arguments and set EXPAT_FOUND to TRUE if
# all listed variables are TRUE
find_package_handle_standard_args(IA_IMAGING
                                  REQUIRED_VARS IA_IMAGING_INCLUDE_DIRS IA_IMAGING_LIBS)

if(NOT IA_IMAGING_FOUND)
    message(FATAL_ERROR "IA_IMAGING not found")
endif()
