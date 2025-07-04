#
#  Copyright (C) 2017-2022 Intel Corporation
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

cmake_minimum_required(VERSION 2.8)

project(libcamhal)

option(BUILD_CAMHAL_PLUGIN "Build libcamhal as plugins" OFF)
option(BUILD_CAMHAL_ADAPTOR "Build hal_adaptor as libcamhal" OFF)

#------------------------- Global settings -------------------------

#------------------------- folder settings -------------------------

# Source directories
set(SRC_ROOT_DIR        ${CMAKE_CURRENT_LIST_DIR}/src)

set(3A_DIR              ${SRC_ROOT_DIR}/3a)
set(CORE_DIR            ${SRC_ROOT_DIR}/core)
set(HAL_DIR             ${SRC_ROOT_DIR}/hal)
set(IIO_DIR             ${SRC_ROOT_DIR}/iio)
set(IUTILS_DIR          ${SRC_ROOT_DIR}/iutils)
set(METADATA_DIR        ${SRC_ROOT_DIR}/metadata)
set(PLATFORMDATA_DIR    ${SRC_ROOT_DIR}/platformdata)
set(SCHEDULER_DIR       ${SRC_ROOT_DIR}/scheduler)
set(V4L2_DIR            ${SRC_ROOT_DIR}/v4l2)
set(MODULES_DIR         ${CMAKE_CURRENT_LIST_DIR}/modules)
set(ALGOWRAPPER_DIR     ${MODULES_DIR}/algowrapper)
set(SANDBOXING_DIR      ${MODULES_DIR}/sandboxing)
set(IMAGE_PROCESS_DIR   ${SRC_ROOT_DIR}/image_process)

if (SUPPORT_LIVE_TUNING)
    #LiveTuning directories
    set(LIVE_TUNING_DIR         ${MODULES_DIR}/livetune)
endif() #SUPPORT_LIVE_TUNING

# CMake module directory
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Install directories
# CMAKE_INSTALL_PREFIX is defined when executing cmake and supposed to be ./out/libcamhal
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}" CACHE PATH "Target location for install command." FORCE)
endif()

#------------------------- compile and link options -------------------------

include(GNUInstallDirs)
# code coverage.
include(code-coverage)

message(STATUS "install prefix: " ${CMAKE_INSTALL_PREFIX})

set (CMAKE_CXX_STANDARD 11)
add_compile_options(
    -Wall
    -fstack-protector
    -fPIE
    -fPIC
    -DDCHECK_ALWAYS_ON
    -Wformat
    -Wformat-security
)
if (USE_STATIC_GRAPH)
    add_compile_options(-Wno-error=changes-meaning)
endif()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG(-faligned-new COMPILER_SUPPORTS_ALIGNED_NEW)
if (COMPILER_SUPPORTS_ALIGNED_NEW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -faligned-new")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package (Threads REQUIRED)

# check if _FORTIFY_SOURCE is default defined with -O
include(CheckFortifyLevel)
set(FORTIFY_SOURCE_VALUE)
check_fortify_source(FORTIFY_SOURCE_VALUE)
if(FORTIFY_SOURCE_VALUE)
    message(STATUS "compiler default _FORTIFY_SOURCE=${FORTIFY_SOURCE_VALUE}")
else()
    message(STATUS "_FORTIFY_SOURCE is not defined or could not be determined.")
    add_compile_options(-D_FORTIFY_SOURCE=2)
    message(STATUS "define _FORTIFY_SOURCE=2")
endif()

add_link_options(
    -fPIE
    -fPIC
    -Wformat
    -Wformat-security
    -Wl,-z,relro
    -Wl,-z,now
    -Wl,--no-as-needed
    -Wl,--no-undefined
)

# -fPIC in xos way
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#------------------- common definitions, includes and link --------------------

add_definitions(-D__STDC_FORMAT_MACROS -DHAVE_PTHREADS -DHAVE_LINUX_OS -DHAVE_IA_TYPES -DHAVE_PRCTL)

include_directories(
    include
    include/api include/utils
    ${CMAKE_CURRENT_LIST_DIR}
    src
    src/v4l2 src/core src/metadata
    src/platformdata src/platformdata/gc
    src/3a src/3a/intel3a src/3a/external
    src/fd
    src/image_process
)

set(LIBCAMHAL_LINK_LIBS ${LIBCAMHAL_LINK_LIBS} ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT} expat rt)

add_subdirectory(src)
add_subdirectory(modules)

if (USE_PG_LITE_PIPE)
    add_definitions(-DUSE_PG_LITE_PIPE)
endif() #USE_PG_LITE_PIPE

    set(LIBCAMHAL_SRCS ${LIBCAMHAL_SRCS} ${IIO_SRCS})

if (SUPPORT_LIVE_TUNING)
    include_directories(modules/livetune)
    set(LIBCAMHAL_SRCS ${LIBCAMHAL_SRCS} ${LIVE_TUNING_SRCS})
endif() #SUPPORT_LIVE_TUNING

if(FACE_DETECTION)
    add_definitions(-DFACE_DETECTION)
    set(LIBCAMHAL_SRCS ${LIBCAMHAL_SRCS} ${FACE_DETECTION_SRCS})
    set(LIBCAMHAL_LINK_LIBS
        ${LIBCAMHAL_LINK_LIBS}
        pvl_eye_detection
        pvl_face_detection
        pvl_mouth_detection
        ${WORK_DIR_TMP}/src/fd/libface_detection.a
    )
endif() #FACE_DETECTION

if (DYNAMIC_PAC_BUFFERS)
    add_definitions(-DMAX_PAC_BUFFERS=${DYNAMIC_PAC_BUFFERS})
else()
    add_definitions(-DMAX_PAC_BUFFERS=2U)
endif()


message("Linux build")
add_definitions(-DSUPPORT_MULTI_PROCESS)
add_definitions(-DLINUX_BUILD)
include_directories(${MODULES_DIR}/v4l2)
find_package(LIBDRM)
if(LIBDRM_FOUND AND ("${LIBDRM_VERSION}" VERSION_GREATER_EQUAL 2.4.114))
    include_directories(${LIBDRM_INCLUDE_DIRS})
    link_directories(${LIBDRM_LIBRARY_DIRS})
    set(LIBCAMHAL_LINK_LIBS ${LIBCAMHAL_LINK_LIBS} ${LIBDRM_LIBRARIES})
    add_definitions(-DLIBDRM_SUPPORT_MMAP_OFFSET)
endif()

set(LIBCAMHAL_SRCS
    ${LIBCAMHAL_SRCS}
    ${3A_SRCS}
    ${CORE_SRCS}
    ${HAL_SRCS}
    ${IPS_CTRL_SRCS}
    ${IUTILS_SRCS}
    ${METADATA_SRCS}
    ${PLATFORMDATA_SRCS}
    ${SCHEDULER_SRCS}
    ${V4L2_SRCS}
    ${ALGOWRAPPER_SRCS}
    ${IMAGE_PROCESS_SRCS}
    )

# #------------------------- Target settings -------------------------

if (NOT IPU_VERSIONS)
    set(IPU_VERSIONS ${IPU_VER})
endif()

if (BUILD_CAMHAL_ADAPTOR)
    add_subdirectory(src/hal/hal_adaptor)
    if (NOT BUILD_CAMHAL_PLUGIN)
        message(NOTICE "Will not build libcamhal plugins")
        set(IPU_VERSIONS "")
    endif()
endif()

# Add different targets according to array IPU_VERSIONS
foreach(IPU_VER ${IPU_VERSIONS})

if (BUILD_CAMHAL_PLUGIN)
    set(TARGET_SUFFIX "-${IPU_VER}")
endif()

set(TARGET_INCLUDE "")
set(TARGET_DEFINITIONS "")
set(TARGET_SRCS "")
set(TARGET_LINK_LIBS "")

if (IPU_VER MATCHES "ipu6")
    message(STATUS "add definition -DIPU_SYSVER_IPU6 for ${IPU_VER}")
    set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} -DIPU_SYSVER_IPU6)
    if (IPU_VER STREQUAL "ipu6sepla")
        message(STATUS "add definition -DIPU_SYSVER_ipu6v3 for ${IPU_VER}")
        set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} -DIPU_SYSVER_ipu6v3)
    elseif (IPU_VER STREQUAL "ipu6ep")
        message(STATUS "add definition -DIPU_SYSVER_ipu6v5 for ${IPU_VER}")
        set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} -DIPU_SYSVER_ipu6v5)
    elseif (IPU_VER STREQUAL "ipu6epmtl")
        message(STATUS "add definition -DIPU_SYSVER_ipu6v6 for ${IPU_VER}")
        set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} -DIPU_SYSVER_ipu6v6)
    endif()
    set(TARGET_INCLUDE ${TARGET_INCLUDE} src/core/psysprocessor)
elseif (IPU_VER MATCHES "ipu7.*")
    set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} -DSTATIC_GRAPH_USE_IA_AIC_TYPES -DPAC_ENABLE)
    set(TARGET_INCLUDE
        ${TARGET_INCLUDE}
        src/core/processingUnit
        src/scheduler
        src/jpeg
        src/iutils
    )
    set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} jsoncpp)
    if (IPU_VER STREQUAL "ipu7x")
        message(STATUS "add definition -DIPU_SYSVER_ipu7 for ${IPU_VER}")
        set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} -DIPU_SYSVER_ipu7 -DGRC_IPU7X)
        set(STATIC_GRAPH_IPU_PREFIXS "Ipu7x")
    elseif (IPU_VER STREQUAL "ipu75xa")
        message(STATUS "add definition -DIPU_SYSVER_ipu75 for ipu75xa")
        set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} -DIPU_SYSVER_ipu75 -DGRC_IPU75XA)
        set(STATIC_GRAPH_IPU_PREFIXS "Ipu75xa")
    endif()
endif()

set(TARGET_DEFINITIONS ${TARGET_DEFINITIONS} "-DCAMERA_DEFAULT_CFG_PATH=\"${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER}/\"")

# Link libraries, the building will be interrupted if libs/includes not found
    # Link ia_imaging
    find_package(IA_IMAGING)
    set(TARGET_INCLUDE ${TARGET_INCLUDE} ${IA_IMAGING${TARGET_SUFFIX}_INCLUDE_DIRS})
    set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} ${IA_IMAGING${TARGET_SUFFIX}_LIBRARIES})
    link_directories(${IA_IMAGING${TARGET_SUFFIX}_LIBRARY_DIRS})

    if(USE_PG_LITE_PIPE)
        find_package(LIBGCSS)
        set(TARGET_INCLUDE ${TARGET_INCLUDE} ${LIBGCSS${TARGET_SUFFIX}_INCLUDE_DIRS})
        set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} ${LIBGCSS${TARGET_SUFFIX}_LIBRARIES})
        link_directories(${LIBGCSS${TARGET_SUFFIX}_LIBRARY_DIRS})
        # Include libipu(4) headers
        find_package(LIBIPU)
        set(TARGET_INCLUDE ${TARGET_INCLUDE} ${LIBIPU${TARGET_SUFFIX}_INCLUDE_DIRS})
        set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} ${LIBIPU${TARGET_SUFFIX}_LIBRARIES})
        link_directories(${LIBIPU${TARGET_SUFFIX}_LIBRARY_DIRS})
    elseif (NOT USE_STATIC_GRAPH)
        # Link libiacss
        find_package(LIBIACSS)
        set(TARGET_INCLUDE ${TARGET_INCLUDE} ${LIBIACSS${TARGET_SUFFIX}_INCLUDE_DIRS})
        set(TARGET_LINK_LIBS ${TARGET_LINK_LIBS} ${LIBIACSS${TARGET_SUFFIX}_LIBRARIES})
        link_directories(${LIBIACSS${TARGET_SUFFIX}_LIBRARY_DIRS})
    endif()

if (USE_STATIC_GRAPH)
    set(TARGET_INCLUDE ${TARGET_INCLUDE} modules/ipu_desc modules/ipu_desc/${IPU_VER})
    if (USE_STATIC_GRAPH_AUTOGEN)
        message(STATUS "Use auto generated static graph code, prefix: ${STATIC_GRAPH_IPU_PREFIXS}")
        set(TARGET_SRCS ${TARGET_SRCS}
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/CBLayoutUtils.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/GraphResolutionConfigurator.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/${STATIC_GRAPH_IPU_PREFIXS}GraphResolutionConfiguratorAutogen.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/${STATIC_GRAPH_IPU_PREFIXS}TerminalDescriptorAutogen.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/${STATIC_GRAPH_IPU_PREFIXS}StaticGraphAutogen.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/${STATIC_GRAPH_IPU_PREFIXS}StaticGraphReaderAutogen.cpp
            )
    else ()
        message(STATUS "Use ${IPU_VER} static graph code")
        set(TARGET_SRCS ${TARGET_SRCS}
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/CBLayoutUtils.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/TerminalDescriptorAutogen.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/StaticGraphAutogen.cpp
            ${MODULES_DIR}/ipu_desc/${IPU_VER}/StaticGraphReaderAutogen.cpp
            )
    endif () # STATIC_GRAPH_IPU_PREFIXS
endif() # USE_STATIC_GRAPH

#--------------------------- Add libcamhal target  ---------------------------

if (BUILD_CAMHAL_PLUGIN)
    set(CAMHAL_TARGET camhal${IPU_VER})
    set(CAMHAL_STATIC_TARGET camhal${IPU_VER}_static)
else()
    set(CAMHAL_TARGET camhal)
    set(CAMHAL_STATIC_TARGET camhal_static)
endif()

# Add libcamhal using the specified sources
add_library(${CAMHAL_STATIC_TARGET} STATIC ${LIBCAMHAL_SRCS} ${TARGET_SRCS})

if (${CMAKE_VERSION} VERSION_LESS 3.11)
    add_library(${CAMHAL_TARGET} SHARED ${LIBCAMHAL_SRCS} ${TARGET_SRCS})
else()
    add_library(${CAMHAL_TARGET} SHARED $<TARGET_OBJECTS:${CAMHAL_STATIC_TARGET}>)
endif()

target_include_directories(${CAMHAL_STATIC_TARGET} PRIVATE ${TARGET_INCLUDE})
target_include_directories(${CAMHAL_TARGET} PRIVATE ${TARGET_INCLUDE})
target_compile_definitions(${CAMHAL_STATIC_TARGET} PRIVATE ${TARGET_DEFINITIONS})
target_compile_definitions(${CAMHAL_TARGET} PRIVATE ${TARGET_DEFINITIONS})
target_link_libraries(${CAMHAL_STATIC_TARGET} PRIVATE ${LIBCAMHAL_LINK_LIBS} ${TARGET_LINK_LIBS})
target_link_libraries(${CAMHAL_TARGET}  PRIVATE ${LIBCAMHAL_LINK_LIBS} ${TARGET_LINK_LIBS})

if (BUILD_CAMHAL_PLUGIN)
    set_target_properties(${CAMHAL_STATIC_TARGET} PROPERTIES PREFIX "")
    set_target_properties(${CAMHAL_TARGET} PROPERTIES PREFIX "")
    set_target_properties(${CAMHAL_STATIC_TARGET} PROPERTIES OUTPUT_NAME ${IPU_VER})
    set_target_properties(${CAMHAL_TARGET} PROPERTIES OUTPUT_NAME ${IPU_VER})
else()
    set_target_properties(${CAMHAL_STATIC_TARGET} PROPERTIES OUTPUT_NAME ${CAMHAL_TARGET})
    set_target_properties(${CAMHAL_TARGET} PROPERTIES SOVERSION 0)
    set_target_properties(${CAMHAL_TARGET} PROPERTIES VERSION "0.0.0")
    set_target_properties(${CAMHAL_TARGET} PROPERTIES OUTPUT_NAME ${CAMHAL_TARGET})
endif()


#--------------------------- Install settings ---------------------------
if (NOT BUILD_CAMHAL_PLUGIN)
    # Install headers
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libcamhal)
    if (SUPPORT_LIVE_TUNING)
        install(FILES modules/livetune/LiveTuning.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libcamhal/api)
    endif() #SUPPORT_LIVE_TUNING
endif()

# Install configure files, by default ipu4 is used
if (IPU_VER STREQUAL "ipu6")
    if ("${PRODUCTION_NAME}" STREQUAL "Andrews")
        message("Install TGL Andrews configure files")
        install(DIRECTORY config/ubuntu/tgl/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER})
    else()
        message("Install TGL IPU6 configure files")
        install(DIRECTORY config/linux/ipu6/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER})
    endif()
elseif (IPU_VER STREQUAL "ipu6ep")
    if ("${PRODUCTION_NAME}" STREQUAL "ccg_platform")
        message("Install ADL Platform configure files")
        install(DIRECTORY config/ubuntu/adl/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER})
    elseif ("${PRODUCTION_NAME}" STREQUAL "Tributo")
        message("Install ADL Tributo configure files")
        install(DIRECTORY config/ubuntu/adl/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER})
    else()
        message("Install ADL IPU6EP configure files")
        install(DIRECTORY config/linux/ipu6ep/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER})
    endif()
elseif (IPU_VER STREQUAL "ipu6sepla")
    if ("${PRODUCTION_NAME}" STREQUAL "penguin_peak")
        message("Install JSL penguin peak configure files")
        install(DIRECTORY config/ubuntu/jsl/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER})
    endif()
elseif (IPU_VER MATCHES "ipu7.*")
    message("Install ${IPU_VER} configure files")
    install(DIRECTORY config/linux/${IPU_VER}/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER} PATTERN "config/linux/*/gcss/*.xml" EXCLUDE)
else()
    message("Install APL IPU4 configure files")
    install(DIRECTORY config/linux/ipu4/ DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR}/camera/${IPU_VER})
endif()

# Install libraries
set(CAMHAL_TARGETS ${CAMHAL_TARGET} ${CAMHAL_STATIC_TARGET})
if (BUILD_CAMHAL_PLUGIN)
    set(CAMHAL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/libcamhal/plugins)
else()
    set(CAMHAL_DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

if (${CMAKE_VERSION} VERSION_LESS 3.11)
    install(TARGETS ${CAMHAL_TARGETS}
            LIBRARY DESTINATION ${CAMHAL_DESTINATION}
            ARCHIVE DESTINATION ${CAMHAL_DESTINATION}
            )
else()
    install(TARGETS ${CAMHAL_TARGETS} DESTINATION ${CAMHAL_DESTINATION})
endif()
if(NOT BUILD_CAMHAL_PLUGIN)
    # Install package config file
    configure_file(${PROJECT_SOURCE_DIR}/cmake/libcamhal.pc.cmakein
                    ${PROJECT_SOURCE_DIR}/libcamhal.pc @ONLY)
    install(FILES libcamhal.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
endif()

endforeach() #IPU_VERSIONS

set(CPACK_GENERATOR "RPM")
include(CPack)
