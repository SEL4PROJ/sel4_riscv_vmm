#
# Copyright 2019, Data61
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230.
#
# This software may be distributed and modified according to the terms of
# the BSD 2-Clause license. Note that NO WARRANTY is provided.
# See "LICENSE_BSD2.txt" for details.
#
# @TAG(DATA61_BSD)
#

cmake_minimum_required(VERSION 3.7.2)

set(project_dir "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(resolved_path ${CMAKE_CURRENT_LIST_FILE} REALPATH)
# repo_dir is distinct from project_dir as this file is symlinked.
# project_dir corresponds to the top level project directory, and
# repo_dir is the absolute path after following the symlink.
get_filename_component(repo_dir ${resolved_path} DIRECTORY)

include(${project_dir}/tools/seL4/cmake-tool/helpers/application_settings.cmake)

include(${repo_dir}/easy-settings.cmake)

correct_platform_strings()

include(${project_dir}/kernel/configs/seL4Config.cmake)

set(
    valid_platforms
    ${KernelPlatform_all_strings}
)
set_property(CACHE PLATFORM PROPERTY STRINGS ${valid_platforms})
list(FIND valid_platforms "${PLATFORM}" index)
if("${index}" STREQUAL "-1")
    message(FATAL_ERROR "Invalid PLATFORM selected: \"${PLATFORM}\"
Valid platforms are: \"${valid_platforms}\"")
endif()


if(SIMULATION)
    ApplyCommonSimulationSettings(${KernelArch})
endif()

set(LibSel4UtilsCSpaceSizeBits 17 CACHE STRING "" FORCE)
ApplyCommonReleaseVerificationSettings(${RELEASE} ${VERIFICATION})
set(HardwareDebugAPI OFF CACHE BOOL "" FORCE)
set(KernelNumDomains 1 CACHE STRING "" FORCE)

set(KernelRiscVHypervisorSupport ON CACHE BOOL "" FORCE)
set(KernelRootCNodeSizeBits 18 CACHE STRING "" FORCE)
set(KernelRiscVNumVTimers 1 CACHE STRING "" FORCE)

