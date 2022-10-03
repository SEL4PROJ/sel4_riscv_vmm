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

project(sel4-riscv-vmm C)

set(configure_string "")

file(GLOB static
    src/*.c
)

if(SMP)
    set(LinuxImage linux-smp)
else()
    set(LinuxImage linux)
endif()

MakeCPIO(archive.o "${CMAKE_SOURCE_DIR}/projects/sel4_riscv_vmm/linux/${LinuxImage};${CMAKE_SOURCE_DIR}/projects/sel4_riscv_vmm/linux/linux-dtb;${CMAKE_SOURCE_DIR}/projects/sel4_riscv_vmm/linux/linux-smp2.dtb;${CMAKE_SOURCE_DIR}/projects/sel4_riscv_vmm/linux/linux-smp3.dtb;${CMAKE_SOURCE_DIR}/projects/sel4_riscv_vmm/linux/linux-smp4.dtb")

add_executable(sel4-riscv-vmm EXCLUDE_FROM_ALL ${static} archive.o)
target_link_libraries(sel4-riscv-vmm muslc sel4 sel4runtime sel4allocman sel4vka sel4utils sel4muslcsys)
target_compile_options(sel4-riscv-vmm PRIVATE -Werror -g)

# Set this image as the rootserver
DeclareRootserver(sel4-riscv-vmm)
GenerateSimulateScript()
